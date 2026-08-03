#include "communication.h"

CommManager Comm;
static CommManager* instance = nullptr;
static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void CommManager::begin() {
    instance = this;
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;

    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    for (int i = 0; i < MAX_NEIGHBORS; i++) neighbors[i].active = false;
    for (int i = 0; i < MAX_PENDING_SYNCS; i++) syncs[i].active = false;

    current_role = ROLE_SEARCHING;
    last_sync_heard_us = esp_timer_get_time();
    synced = false;
    tx_ready = true;
}

void CommManager::handle_vision_data(const VisionPacket& vp) {
    push_tx(MAGIC_VISION, (uint8_t*)&vp, sizeof(vp));
}

void CommManager::update_state(float x, float y, float theta) {
    uint64_t now_us = get_global_time();
    float dx = x - last_tx_x;
    float dy = y - last_tx_y;
    float pos_delta = sqrt((dx * dx) + (dy * dy));
    
    float heading_delta = abs(theta - last_tx_theta);
    if (heading_delta > 180.0f) heading_delta = 360.0f - heading_delta;

    bool heartbeat = (now_us - last_tx_time >= 2000000); 
    bool moved = (pos_delta >= 0.05f) || (heading_delta >= 2.0f); 

    if (heartbeat || moved) {
        StatePayload s;
        s.timestamp = now_us;
        s.x = x; s.y = y; s.theta = theta;
        send_state(s);

        last_tx_x = x; last_tx_y = y; last_tx_theta = theta;
        last_tx_time = now_us;
    }
}

void CommManager::poll() {
    uint64_t now_us = esp_timer_get_time();

    if (now_us - last_sync_heard_us > 5000000) {
        current_role = ROLE_MASTER;
        synced = true; 
        offset_us = 0;
    }

    if (current_role == ROLE_MASTER) {
        static uint64_t last_sync_tx = 0;
        if (now_us - last_sync_tx >= 1000000) {
            last_sync_tx = now_us;
            send_sync();
        }
    }

    if (!tx_ready) return;
    TxPacket pkt;
    if (sync_tx.pop(pkt)) { tx_ready = false; esp_now_send(broadcast_mac, pkt.data, pkt.len); return; }
    if (collision_tx.pop(pkt)) { tx_ready = false; esp_now_send(broadcast_mac, pkt.data, pkt.len); return; }
    if (state_tx.pop(pkt)) { tx_ready = false; esp_now_send(broadcast_mac, pkt.data, pkt.len); return; }
}

void CommManager::on_recv(const uint8_t *mac, const uint8_t *data, int len) {
    if (!instance || len < sizeof(Header)) return;
    uint64_t rx_time = esp_timer_get_time();
    instance->last_rx_timestamp_us = rx_time;

    Header* h = (Header*)data;
    uint8_t* payload = (uint8_t*)(data + sizeof(Header));
    instance->process(h->magic, payload, h->len, rx_time, mac);
}

void CommManager::process(uint8_t magic, uint8_t* payload, uint16_t len, uint64_t rx_time, const uint8_t* src_mac) {
    
    // ---> MAILBOX LOGIC: Catch Vision Data <---
    if (magic == MAGIC_VISION && len == sizeof(VisionPacket)) {
        VisionPacket* vp = (VisionPacket*)payload;
        last_vision = *vp;
        new_vision_data = true;
        return; 
    }

    if (magic == MAGIC_SYNC || magic == MAGIC_DELAY_REQ || magic == MAGIC_DELAY_RESP) {
        handle_sync(magic, payload, len, rx_time, src_mac);
        return;
    }

    if (magic == MAGIC_STATE && len == sizeof(StatePayload)) {
        StatePayload* p = (StatePayload*)payload;
        int slot = -1;
        for (int i = 0; i < MAX_NEIGHBORS; i++) {
            if (neighbors[i].active && memcmp(neighbors[i].mac, src_mac, 6) == 0) { slot = i; break; }
        }
        if (slot == -1) {
            for (int i = 0; i < MAX_NEIGHBORS; i++) {
                if (!neighbors[i].active) { slot = i; memcpy(neighbors[slot].mac, src_mac, 6); neighbors[slot].active = true; break; }
            }
        }
        if (slot != -1) {
            neighbors[slot].x = p->x; neighbors[slot].y = p->y; neighbors[slot].theta = p->theta;
            neighbors[slot].last_seen_us = rx_time;
        }
    }
}

void CommManager::handle_sync(uint8_t magic, uint8_t* payload, uint16_t len, uint64_t rx_time, const uint8_t* src_mac) {
    if (magic == MAGIC_SYNC) {
        uint8_t my_mac[6]; WiFi.macAddress(my_mac);
        if (memcmp(src_mac, my_mac, 6) < 0) {
            current_role = ROLE_SLAVE;
            last_sync_heard_us = rx_time;
        } else if (current_role == ROLE_MASTER) return; 

        SyncPayload* p = (SyncPayload*)payload;
        int slot = p->seq % MAX_PENDING_SYNCS;
        syncs[slot].active = true; syncs[slot].seq = p->seq;
        syncs[slot].t1_master = p->t1_master; syncs[slot].t2 = rx_time;
        syncs[slot].t3 = esp_timer_get_time();

        DelayReqPayload req;
        req.seq = p->seq; req.t2_slave = syncs[slot].t2; req.t3_slave = syncs[slot].t3;
        push_tx(MAGIC_DELAY_REQ, (uint8_t*)&req, sizeof(req));
    }

    if (magic == MAGIC_DELAY_REQ && current_role == ROLE_MASTER) {
        DelayReqPayload* req = (DelayReqPayload*)payload;
        int slot = req->seq % MAX_PENDING_SYNCS;
        if (syncs[slot].active && syncs[slot].seq == req->seq) {
            int64_t master_delta = rx_time - syncs[slot].t1_master;
            int64_t slave_delta = req->t3_slave - req->t2_slave;
            last_rtt_us = master_delta - slave_delta;
            int64_t computed_offset = syncs[slot].t1_master + (last_rtt_us / 2) - req->t2_slave;

            if (last_slave_offset != 0) {
                current_jitter_us = abs((int32_t)(computed_offset - last_slave_offset));
                int64_t dt = rx_time - last_sync_time_master;
                if (dt > 0) drift_ppm = (0.8f * drift_ppm) + (0.2f * ((float)(computed_offset - last_slave_offset) * 1e6f / dt));
            }
            last_slave_offset = computed_offset; last_sync_time_master = rx_time;
            
            DelayRespPayload resp;
            resp.seq = req->seq; resp.t1_master = syncs[slot].t1_master; resp.t4_master = rx_time;
            push_tx(MAGIC_DELAY_RESP, (uint8_t*)&resp, sizeof(resp));
            syncs[slot].active = false;
        }
    }

    if (magic == MAGIC_DELAY_RESP && current_role == ROLE_SLAVE) {
        DelayRespPayload* r = (DelayRespPayload*)payload;
        int slot = r->seq % MAX_PENDING_SYNCS;
        if (syncs[slot].active && syncs[slot].seq == r->seq) {
            int64_t delay = ((r->t4_master - r->t1_master) - (syncs[slot].t3 - syncs[slot].t2)) / 2;
            offset_us = r->t1_master + delay - syncs[slot].t2;
            synced = true; syncs[slot].active = false;
        }
    }
}

void CommManager::prune_neighbors() {
    uint64_t now = esp_timer_get_time();
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active && (now - neighbors[i].last_seen_us > 3000000)) neighbors[i].active = false;
    }
}

void CommManager::on_sent(const uint8_t *mac, esp_now_send_status_t status) { if (instance) instance->tx_ready = true; }

void CommManager::push_tx(uint8_t magic, const uint8_t* payload, uint16_t len) {
    TxPacket pkt; Header h = {magic, len};
    memcpy(pkt.data, &h, sizeof(h)); memcpy(pkt.data + sizeof(h), payload, len); pkt.len = sizeof(h) + len;
    
    if (magic >= 0xA1 && magic <= 0xA3) sync_tx.push(pkt);
    else if (magic == MAGIC_COLLISION) collision_tx.push(pkt);
    else state_tx.push(pkt);
}

void CommManager::send_sync() {
    SyncPayload s; s.seq = ++seq_sync; s.t1_master = esp_timer_get_time();
    int slot = s.seq % MAX_PENDING_SYNCS;
    syncs[slot].active = true; syncs[slot].seq = s.seq; syncs[slot].t1_master = s.t1_master;
    push_tx(MAGIC_SYNC, (uint8_t*)&s, sizeof(s));
}

void CommManager::send_state(const StatePayload& s) {
    StatePayload pkt = s; pkt.seq = ++seq_state;
    push_tx(MAGIC_STATE, (uint8_t*)&pkt, sizeof(pkt));
}

uint64_t CommManager::get_global_time() { return synced ? esp_timer_get_time() + offset_us : esp_timer_get_time(); }
bool CommManager::is_synced() { return synced; }
const char* CommManager::get_current_role() {
    if (current_role == ROLE_MASTER) return "MASTER";
    if (current_role == ROLE_SLAVE) return "SLAVE";
    return "SEARCHING";
}