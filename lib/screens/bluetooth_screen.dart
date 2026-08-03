import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

// IMPORTANT: Change this import to match the actual name of your file!
import 'control_screen.dart';

class BluetoothScreen extends StatefulWidget {
  const BluetoothScreen({Key? key}) : super(key: key);

  @override
  State<BluetoothScreen> createState() => _BluetoothScreenState();
}

class _BluetoothScreenState extends State<BluetoothScreen> {
  BluetoothState _bluetoothState = BluetoothState.UNKNOWN;

  // We now store discovery results instead of just bonded devices
  List<BluetoothDiscoveryResult> _discoveryResults = [];
  bool _isConnecting = false;
  bool _isDiscovering = false;

  StreamSubscription<BluetoothDiscoveryResult>? _discoveryStreamSubscription;

  @override
  void initState() {
    super.initState();

    // Get current Bluetooth state
    FlutterBluetoothSerial.instance.state.then((state) {
      setState(() {
        _bluetoothState = state;
      });
    });

    // Listen for state changes (like user turning Bluetooth on/off)
    FlutterBluetoothSerial.instance.onStateChanged().listen((BluetoothState state) {
      setState(() {
        _bluetoothState = state;
      });
    });

    // Start scanning for nearby devices immediately
    _startDiscovery();
  }

  @override
  void dispose() {
    _discoveryStreamSubscription?.cancel(); // Important to prevent memory leaks
    super.dispose();
  }

  // Function to scan for nearby available devices WITH error handling
  void _startDiscovery() {
    _discoveryStreamSubscription?.cancel();

    setState(() {
      _isDiscovering = true;
      _discoveryResults.clear();
    });

    _discoveryStreamSubscription = FlutterBluetoothSerial.instance.startDiscovery().listen(
      (r) {
        setState(() {
          // If device is already in the list, update it. Otherwise, add it.
          final existingIndex = _discoveryResults.indexWhere((element) => element.device.address == r.device.address);
          if (existingIndex >= 0) {
            _discoveryResults[existingIndex] = r;
          } else {
            _discoveryResults.add(r);
          }
        });
      },
      onError: (error) {
        // Catch silent permission/location failures!
        debugPrint("Bluetooth Discovery Error: $error");

        if (mounted) {
          setState(() {
            _isDiscovering = false;
          });

          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: const Text('Scan failed. Please enable Location/GPS and check App Permissions.'),
              backgroundColor: Colors.red.shade800,
              duration: const Duration(seconds: 4),
            ),
          );
        }
      },
      onDone: () {
        if (mounted) {
          setState(() {
            _isDiscovering = false;
          });
        }
      },
    );
  }

  // Handle the connection and navigate to ControlScreen
  void _connectToDevice(BluetoothDevice device) async {
    // Cancel discovery when we try to connect to keep the connection stable
    _discoveryStreamSubscription?.cancel();

    setState(() {
      _isConnecting = true;
      _isDiscovering = false;
    });

    try {
      BluetoothConnection connection = await BluetoothConnection.toAddress(device.address);

      setState(() {
        _isConnecting = false;
      });

      if (mounted) {
        Navigator.push(
          context,
          MaterialPageRoute(
            builder: (context) => ControlScreen(connection: connection),
          ),
        ).then((_) {
          // Optional: Restart discovery if we come back to this screen
          _startDiscovery();
        });
      }
    } catch (exception) {
      setState(() {
        _isConnecting = false;
      });
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Cannot connect to ${device.name ?? "device"}. Is it powered on?'),
            backgroundColor: Colors.red.shade800,
          ),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black, // Matches your dark UI perfectly
      appBar: AppBar(
        backgroundColor: Colors.grey.shade900,
        elevation: 0,
        title: const Text('Nearby Devices', style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
        actions: [

          // REFRESH BUTTON (Shows a spinner while actively scanning)
          _isDiscovering
            ? const Padding(
                padding: EdgeInsets.symmetric(horizontal: 20.0),
                child: Center(
                  child: SizedBox(
                    width: 20, height: 20,
                    child: CircularProgressIndicator(color: Colors.white70, strokeWidth: 2)
                  )
                ),
              )
            : IconButton(
                icon: const Icon(Icons.refresh, color: Colors.white70),
                onPressed: _startDiscovery,
                tooltip: 'Scan for Devices',
              ),

          // BYPASS BUTTON (For the presentation simulator!)
          TextButton.icon(
            style: TextButton.styleFrom(foregroundColor: Colors.tealAccent),
            icon: const Icon(Icons.developer_mode),
            label: const Text("BYPASS", style: TextStyle(fontWeight: FontWeight.bold, letterSpacing: 1)),
            onPressed: () {
              // Cancel discovery before jumping to the next screen
              _discoveryStreamSubscription?.cancel();
              setState(() { _isDiscovering = false; });

              Navigator.push(
                context,
                MaterialPageRoute(
                  builder: (context) => const ControlScreen(connection: null), // Sends null connection safely
                ),
              );
            },
          ),
          const SizedBox(width: 8), // Tiny padding on the right edge
        ],
      ),

      // Dynamic Body: Handles Bluetooth Off, Connecting State, Empty List, and Device List
      body: _bluetoothState == BluetoothState.STATE_OFF
          ? const Center(
              child: Text(
                'Bluetooth is turned off.\nPlease turn it on to scan.',
                style: TextStyle(color: Colors.white70, fontSize: 18),
                textAlign: TextAlign.center,
              ),
            )
          : _isConnecting
              ? Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      const CircularProgressIndicator(color: Colors.tealAccent),
                      const SizedBox(height: 16),
                      Text("Establishing V2V Link...", style: TextStyle(color: Colors.teal.shade200, fontWeight: FontWeight.bold)),
                    ],
                  ),
                )
              : _discoveryResults.isEmpty
                  ? Center(
                      child: Text(
                        _isDiscovering
                          ? 'Scanning for nearby devices...'
                          : 'No devices found nearby.\nMake sure the ESP32 is powered on.',
                        style: const TextStyle(color: Colors.white54, fontSize: 16),
                        textAlign: TextAlign.center,
                      ),
                    )
                  : ListView.builder(
                      padding: const EdgeInsets.all(8),
                      itemCount: _discoveryResults.length,
                      itemBuilder: (context, index) {

                        // Extracting the device from the discovery result
                        BluetoothDiscoveryResult result = _discoveryResults[index];
                        BluetoothDevice device = result.device;

                        return Card(
                          color: Colors.grey.shade900,
                          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                          child: ListTile(
                            contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                            leading: const CircleAvatar(
                              backgroundColor: Colors.black,
                              child: Icon(Icons.bluetooth, color: Colors.blueAccent),
                            ),
                            title: Text(device.name ?? "Unknown Device", style: const TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
                            subtitle: Text(device.address, style: const TextStyle(color: Colors.white54)),
                            trailing: ElevatedButton(
                              style: ElevatedButton.styleFrom(
                                backgroundColor: Colors.teal.shade700,
                                foregroundColor: Colors.white,
                                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                              ),
                              child: const Text("CONNECT"),
                              onPressed: () => _connectToDevice(device),
                            ),
                          ),
                        );
                      },
                    ),
    );
  }
}