#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>

/* =========================
   BLACK BOX CONFIGURATION
========================= */
#define MAX_PACKET_SIZE   128
#define TX_BUFFER_SIZE    16
#define MAX_NEIGHBORS     10   
#define MAX_PENDING_SYNCS 4

/* =========================
   MAGIC NUMBERS (PROTOCOL)
========================= */
#define MAGIC_SYNC        0xA1
#define MAGIC_DELAY_REQ   0xA2
#define MAGIC_DELAY_RESP  0xA3
#define MAGIC_COLLISION   0xB2
#define MAGIC_STATE       0xB4
#define MAGIC_VISION      0xC1 // For Bridge-to-Car communication

/* =========================
   PACKED PROTOCOL STRUCTS
========================= */
#pragma pack(push,1)

struct Header {
    uint8_t  magic;
    uint16_t len;
};

// Matches Python: struct.pack('<BBQfff', sender, target, t_us, x, y, yaw)
struct VisionPacket {
    uint8_t  sender_id;
    uint8_t  target_id;
    uint64_t t_us;
    float    x;
    float    y;
    float    yaw;
};

struct SyncPayload {
    uint16_t seq;
    uint64_t t1_master;
};

struct DelayReqPayload {
    uint16_t seq;
    uint64_t t2_slave;
    uint64_t t3_slave;
};

struct DelayRespPayload {
    uint16_t seq;
    uint64_t t1_master;
    uint64_t t4_master;
};

struct StatePayload {
    uint16_t seq;
    uint64_t timestamp;
    float x;
    float y;
    float theta;
};

struct CollisionPayload {
    uint16_t seq;
    uint64_t timestamp;
    float rel_x;
    float rel_y;
    float ttc;
};

#pragma pack(pop)

/* =========================
   GENERIC RING BUFFER
========================= */
template<typename T, uint8_t SIZE>
struct RingBuffer {
    T buf[SIZE];
    volatile uint8_t head = 0;
    volatile uint8_t tail = 0;

    bool push(const T& d){
        uint8_t next = (head + 1) % SIZE;
        if(next == tail) return false;
        buf[head] = d;
        head = next;
        return true;
    }

    bool pop(T& out){
        if(head == tail) return false;
        out = buf[tail];
        tail = (tail + 1) % SIZE;
        return true;
    }

    bool empty() const { return head == tail; }
};

/* =========================
   V2V BLACK BOX API
========================= */
class CommManager {

public:
    void begin();
    void poll();
    void update_state(float x, float y, float theta);
    void prune_neighbors();
    void handle_vision_data(const VisionPacket& vp);

    /* ---- OUTPUTS ---- */
    uint64_t get_global_time();
    bool     is_synced();
    const char* get_current_role();

    /* ---- THE MAILBOX ---- */
    VisionPacket last_vision;
    volatile bool new_vision_data = false;

    struct Neighbor {
        uint8_t  mac[6];
        float    x, y, theta;
        uint64_t last_seen_us;
        bool     active = false;
    };
    
    Neighbor neighbors[MAX_NEIGHBORS]; 

private:
    /* ---- ROLE & NEGOTIATION ---- */
    enum NodeRole { ROLE_SEARCHING, ROLE_MASTER, ROLE_SLAVE };
    NodeRole current_role = ROLE_SEARCHING;
    uint64_t last_sync_heard_us = 0;

    /* ---- INTERNAL TIME SYNC ---- */
    int64_t  offset_us = 0;
    int64_t  last_slave_offset = 0; 
    float    drift_ppm = 0.0f;
    uint32_t current_jitter_us = 0;
    uint32_t last_rtt_us = 0;
    uint64_t last_sync_time_master = 0;
    bool     synced = false;

    /* ---- KINEMATIC MEMORY ---- */
    float    last_tx_x = 0, last_tx_y = 0, last_tx_theta = 0;
    uint64_t last_tx_time = 0;

    /* ---- NETWORK STATE ---- */
    uint64_t last_rx_timestamp_us = 0;
    volatile bool tx_ready = true;

    /* ---- CALLBACKS & INTERNAL HELPERS ---- */
    static void on_recv(const uint8_t *mac, const uint8_t *data, int len);
    static void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
    
    void process(uint8_t magic, uint8_t* payload, uint16_t len, uint64_t rx_time, const uint8_t* src_mac);
    void handle_sync(uint8_t magic, uint8_t* payload, uint16_t len, uint64_t rx_time, const uint8_t* src_mac);
    
    void send_state(const StatePayload& s); 
    void send_sync();
    void push_tx(uint8_t magic, const uint8_t* payload, uint16_t len);

    /* ---- QUEUES & SCHEDULING ---- */
    struct TxPacket { uint8_t data[MAX_PACKET_SIZE]; uint16_t len; };
    
    RingBuffer<TxPacket, TX_BUFFER_SIZE> sync_tx;
    RingBuffer<TxPacket, TX_BUFFER_SIZE> collision_tx;
    RingBuffer<TxPacket, TX_BUFFER_SIZE> state_tx;

    uint16_t seq_state = 0;
    uint16_t seq_sync = 0;

    struct Pending { uint16_t seq; uint64_t t1_master, t2, t3; bool active; };
    Pending syncs[MAX_PENDING_SYNCS];
};

extern CommManager Comm;

#endif