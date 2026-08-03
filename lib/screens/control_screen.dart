import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:flutter_vibrate/flutter_vibrate.dart';
import 'log_screen.dart';

class ControlScreen extends StatefulWidget {
  final BluetoothConnection? connection;

  const ControlScreen({Key? key, required this.connection}) : super(key: key);

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  // --- COLLISION THRESHOLDS & UI ---
  double dynamicCollisionRadius = 0.30;
  final double ttcThreshold = 2.0;
  final TextEditingController _radiusController = TextEditingController(text: "0.30");

  bool isWarning = false;
  bool isEmergency = false;
  double displayDistance = 999.0;
  double displayTtc = 999.0;

  double radarRelX = 0.0;
  double radarRelY = 0.0;
  double radarRelTheta = 0.0;

  List<Offset> positionHistory = [];

  // Smooth Tracking Variables
  double selfX = 0.0, selfY = 0.0, selfTheta = 0.0;
  double otherX = 0.0, otherY = 0.0, otherTheta = 0.0;
  bool isFirstOtherData = true;

  double selfVelocity = 0.0;
  double selfCov = 0.0;

  final double otherVelocity = 0.0;

  double lastDistance = -1.0;
  int lastTimeMs = 0;
  double smoothedClosingSpeed = 0.0;

  final double radarMaxDist = 5.0;

  String _dataBuffer = "";
  Timer? _commandTimer;
  Timer? _emergencyTimer;

  final ValueNotifier<List<String>> logNotifier = ValueNotifier<List<String>>([]);
  Map<String, Map<String, dynamic>> visionData = {};
  String? myId;

  void _addLog(String log) {
    final currentLogs = List<String>.from(logNotifier.value);
    currentLogs.insert(0, log);
    if (currentLogs.length > 200) currentLogs.removeLast();
    logNotifier.value = currentLogs;
  }

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    _startListeningToBluetooth();
  }

  @override
  void dispose() {
    _commandTimer?.cancel();
    _emergencyTimer?.cancel();
    _radiusController.dispose();
    widget.connection?.dispose();
    super.dispose();
  }

  void _startListeningToBluetooth() {
    widget.connection?.input?.listen((Uint8List data) {
      _dataBuffer += ascii.decode(data);
      while (_dataBuffer.contains('\n')) {
        int index = _dataBuffer.indexOf('\n');
        String completeLine = _dataBuffer.substring(0, index).trim();
        _dataBuffer = _dataBuffer.substring(index + 1);
        if (completeLine.isNotEmpty) {
          _addLog("RCV: $completeLine");
          _processIncomingData(completeLine);
        }
      }
    }).onDone(() {
      if (mounted) setState(() {});
    });
  }

  void _processIncomingData(String line) async {
    List<String> parts = line.split(',');
    if (parts.isEmpty) return;

    String type = parts[0];
    String extractStr(int index) => parts.length > index ? parts[index].split(':').last : "";
    double extractVal(int index) => parts.length > index ? (double.tryParse(parts[index].split(':').last) ?? 0.0) : 0.0;

    int currentTimeMs = DateTime.now().millisecondsSinceEpoch;

    if (type == 'S' && parts.length >= 8) {
      myId = extractStr(1);
      selfVelocity = extractVal(6);
      selfCov = extractVal(8);
      return;
    }

    if (type == 'V' && parts.length >= 7) {
      String vid = extractStr(1);
      double px = extractVal(4);
      double py = extractVal(5);
      double pth = extractVal(6);
      visionData[vid] = {'x': px, 'y': py, 'theta': pth, 'lastSeen': currentTimeMs};
    }

    setState(() {
      visionData.removeWhere((key, data) => currentTimeMs - (data['lastSeen'] as int) > 1000);

      if (myId == null && visionData.keys.isNotEmpty) {
        myId = visionData.keys.first;
      }

      if (myId != null && visionData.containsKey(myId) && visionData.length >= 2) {

        double rawSelfX = visionData[myId]!['x'] as double;
        double rawSelfY = visionData[myId]!['y'] as double;
        double rawSelfTheta = visionData[myId]!['theta'] as double;

        // EGO VEHICLE SMOOTHING
        const double selfAlpha = 0.4;
        selfX = selfX + selfAlpha * (rawSelfX - selfX);
        selfY = selfY + selfAlpha * (rawSelfY - selfY);
        selfTheta = rawSelfTheta;

        String targetOtherId = visionData.keys.firstWhere((k) => k != myId);
        double rawOtherX = visionData[targetOtherId]!['x'] as double;
        double rawOtherY = visionData[targetOtherId]!['y'] as double;

        // CHANGED: We must pull the rotation of the other car now!
        double rawOtherTheta = visionData[targetOtherId]!['theta'] as double;

        if (isFirstOtherData) {
          otherX = rawOtherX;
          otherY = rawOtherY;
          otherTheta = rawOtherTheta;
          isFirstOtherData = false;
        } else {
          // CHANGED: Increased from 0.05 to 0.4.
          // This makes the radar highly responsive to moving targets while still removing camera jitter.
          const double targetAlpha = 0.4;
          otherX = otherX + targetAlpha * (rawOtherX - otherX);
          otherY = otherY + targetAlpha * (rawOtherY - otherY);

          // Smooth the rotation of the other car
          otherTheta = otherTheta + targetAlpha * (rawOtherTheta - otherTheta);
        }

        double dx = otherX - selfX;
        double dy = otherY - selfY;
        double currentDistance = sqrt((dx * dx) + (dy * dy));

        if (lastDistance >= 0.0 && lastTimeMs > 0) {
          double dt = (currentTimeMs - lastTimeMs) / 1000.0;

          if (dt > 0.01) {
            double rawClosingSpeed = (lastDistance - currentDistance) / dt;
            const double speedAlpha = 0.15;
            smoothedClosingSpeed = (speedAlpha * rawClosingSpeed) + ((1.0 - speedAlpha) * smoothedClosingSpeed);

            if (smoothedClosingSpeed > 0.05) {
              displayTtc = currentDistance / smoothedClosingSpeed;
            } else {
              displayTtc = 999.0;
            }
          }
        }

        lastDistance = currentDistance;
        lastTimeMs = currentTimeMs;

        if (currentDistance <= dynamicCollisionRadius || displayTtc <= ttcThreshold) {
          isWarning = true;
          displayDistance = currentDistance;
          _triggerVibration();
        } else {
          isWarning = false;
          displayDistance = currentDistance;
        }

        double targetRelX = dx * cos(-selfTheta) - dy * sin(-selfTheta);
        double targetRelY = dx * sin(-selfTheta) + dy * cos(-selfTheta);
        double targetRelTheta = otherTheta - selfTheta;

        const double visualAlpha = 0.35; // Increased visual response speed
        radarRelX = radarRelX + visualAlpha * (targetRelX - radarRelX);
        radarRelY = radarRelY + visualAlpha * (targetRelY - radarRelY);

        double diffTheta = targetRelTheta - radarRelTheta;
        diffTheta = (diffTheta + pi) % (2 * pi) - pi;
        radarRelTheta = radarRelTheta + visualAlpha * diffTheta;

        positionHistory.add(Offset(radarRelX, radarRelY));
        if (positionHistory.length > 20) {
          positionHistory.removeAt(0);
        }

      } else {
        isWarning = false;
        displayDistance = 999.0;
        displayTtc = 999.0;
        smoothedClosingSpeed = 0.0;
        lastDistance = -1.0;
        isFirstOtherData = true;
        positionHistory.clear();
      }
    });
  }

  void _triggerVibration() async {
    if (!isEmergency && await Vibrate.canVibrate) {
      Vibrate.vibrate();
    }
  }

  void startCommand(String command) {
    _addLog("SND: Command '$command'");
    _commandTimer?.cancel();
    _commandTimer = Timer.periodic(const Duration(milliseconds: 50), (_) {
      if (widget.connection?.isConnected ?? false) {
        widget.connection?.output.add(Uint8List.fromList(command.codeUnits));
      }
    });
  }

  void stopCommand() {
    _addLog("SND: Command 'S'");
    _commandTimer?.cancel();
    if (widget.connection?.isConnected ?? false) {
      widget.connection?.output.add(Uint8List.fromList("S".codeUnits));
    }
  }

  void sendEmergencyAlert() async {
    setState(() {
      isEmergency = !isEmergency;
    });

    if (isEmergency) {
      _addLog("SND: SOS Hazard Mode 'STOP : 0'");

      if (widget.connection?.isConnected ?? false) {
        widget.connection?.output.add(Uint8List.fromList("STOP : 0".codeUnits));
      }

      bool canVibrate = await Vibrate.canVibrate;
      if (canVibrate) {
        Vibrate.vibrate();
      }

      _emergencyTimer = Timer.periodic(const Duration(milliseconds: 1000), (timer) {
        _addLog("SND: STOP : 0");
        if (widget.connection?.isConnected ?? false) {
          widget.connection?.output.add(Uint8List.fromList("STOP : 0".codeUnits));
        }
        if (canVibrate) {
          Vibrate.vibrate();
        }
      });

    } else {
      _emergencyTimer?.cancel();
      _addLog("EMERGENCY SITUATION EXITED");

      if (widget.connection?.isConnected ?? false) {
        widget.connection?.output.add(Uint8List.fromList("RESUME : 1".codeUnits));
      }
    }
  }

  Widget buildDriveButton(IconData icon, String command) {
    return GestureDetector(
      onTapDown: (_) => startCommand(command),
      onTapUp: (_) => stopCommand(),
      onTapCancel: () => stopCommand(),
      child: Icon(icon, size: 45, color: Colors.white),
    );
  }

  @override
  Widget build(BuildContext context) {
    bool isConnected = widget.connection?.isConnected ?? false;

    Color bannerColor = Colors.green.shade800;
    String bannerText = "STATUS: SAFE";
    bool showGlow = false;

    if (isEmergency) {
      bannerColor = Colors.orange.shade900;
      bannerText = "HAZARD BROADCASTING! (SOS)";
      showGlow = true;
    } else if (isWarning) {
      bannerColor = Colors.red.shade800;
      bannerText = "COLLISION ALERT! (${displayDistance.toStringAsFixed(2)}m | ${displayTtc > 99 ? 'N/A' : displayTtc.toStringAsFixed(1)}s)";
      showGlow = true;
    }

    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Row(
            children: [
              Expanded(
                flex: 2,
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Flexible(
                      child: SingleChildScrollView(
                        child: Container(
                          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                          decoration: BoxDecoration(
                            color: Colors.grey.shade900,
                            borderRadius: BorderRadius.circular(16),
                            border: Border.all(color: Colors.white12),
                          ),
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              const Text("TELEMETRY", style: TextStyle(color: Colors.white54, fontWeight: FontWeight.bold, letterSpacing: 1.2)),
                              const SizedBox(height: 6),
                              _buildTelemetryRow("My Speed", "${selfVelocity.toStringAsFixed(1)} m/s", Colors.white),
                              const SizedBox(height: 4),
                              _buildTelemetryRow("Covariance", selfCov.toStringAsFixed(4), Colors.tealAccent),

                              const Divider(color: Colors.white24, height: 12),

                              const Text("Collision Radius (m)", style: TextStyle(color: Colors.white54, fontSize: 12)),
                              const SizedBox(height: 2),
                              SizedBox(
                                height: 30,
                                child: TextField(
                                  controller: _radiusController,
                                  keyboardType: const TextInputType.numberWithOptions(decimal: true),
                                  style: const TextStyle(color: Colors.yellowAccent, fontWeight: FontWeight.bold, fontSize: 14),
                                  decoration: InputDecoration(
                                    filled: true,
                                    fillColor: Colors.black45,
                                    contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 0),
                                    border: OutlineInputBorder(borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none),
                                  ),
                                  onChanged: (val) {
                                    setState(() {
                                      dynamicCollisionRadius = double.tryParse(val) ?? 0.30;
                                    });
                                  },
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),
                    ),
                    const SizedBox(height: 8),
                    _buildJoystickContainer(Column(
                      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                      children: [
                        buildDriveButton(Icons.keyboard_arrow_up, "F"),
                        Container(height: 2, width: 60, color: Colors.teal.shade800),
                        buildDriveButton(Icons.keyboard_arrow_down, "B"),
                      ],
                    )),
                  ],
                ),
              ),

              Expanded(
                flex: 4,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Container(
                      margin: const EdgeInsets.only(left: 30.0, bottom: 24.0),
                      width: double.infinity,
                      padding: const EdgeInsets.symmetric(vertical: 8),
                      decoration: BoxDecoration(
                        color: bannerColor,
                        borderRadius: BorderRadius.circular(16),
                        boxShadow: [
                          if (showGlow) BoxShadow(color: bannerColor.withOpacity(0.5), blurRadius: 20, spreadRadius: 2)
                        ]
                      ),
                      child: Center(
                        child: Text(
                          bannerText,
                          style: const TextStyle(color: Colors.white, fontSize: 20, fontWeight: FontWeight.bold, letterSpacing: 1),
                        ),
                      ),
                    ),

                    Expanded(
                      child: Center(
                        child: SizedBox(
                          width: 280, height: 280,
                          child: CustomPaint(
                            painter: RadarPainter(
                              relX: radarRelX, relY: radarRelY, relTheta: radarRelTheta,
                              maxDist: radarMaxDist, isWarning: isWarning,
                              isEmergency: isEmergency, history: positionHistory,
                              collisionRadius: dynamicCollisionRadius,
                            ),
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),

              Expanded(
                flex: 2,
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.end,
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Flexible(
                      child: SingleChildScrollView(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.end,
                          children: [
                            Row(
                              mainAxisAlignment: MainAxisAlignment.end,
                              children: [
                                IconButton(
                                  icon: Icon(
                                    isConnected ? Icons.bluetooth_connected : Icons.bluetooth_disabled,
                                    color: isConnected ? Colors.blueAccent : Colors.redAccent,
                                    size: 28,
                                  ),
                                  onPressed: () => Navigator.pop(context),
                                ),
                                const SizedBox(width: 8),
                                ElevatedButton.icon(
                                  style: ElevatedButton.styleFrom(
                                    backgroundColor: Colors.red,
                                    foregroundColor: Colors.white,
                                    padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 14),
                                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
                                    elevation: 5,
                                  ),
                                  icon: const Icon(Icons.warning_amber_rounded, size: 24),
                                  label: const Text("SOS", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                                  onPressed: () {
                                    sendEmergencyAlert();
                                  },
                                ),
                              ],
                            ),
                            const SizedBox(height: 12),
                            OutlinedButton.icon(
                              style: OutlinedButton.styleFrom(
                                foregroundColor: Colors.white70,
                                side: const BorderSide(color: Colors.white24),
                                padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
                                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(30)),
                              ),
                              icon: const Icon(Icons.receipt_long),
                              label: const Text("VIEW LOGS", style: TextStyle(fontWeight: FontWeight.bold)),
                              onPressed: () {
                                Navigator.push(context, MaterialPageRoute(builder: (context) => LogScreen(logNotifier: logNotifier)));
                              },
                            ),
                          ],
                        ),
                      ),
                    ),
                    const SizedBox(height: 8),
                    _buildJoystickContainer(Row(
                      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                      children: [
                        buildDriveButton(Icons.keyboard_arrow_left, "L"),
                        Container(width: 2, height: 60, color: Colors.teal.shade800),
                        buildDriveButton(Icons.keyboard_arrow_right, "R"),
                      ],
                    )),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildTelemetryRow(String label, String value, Color valueColor) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Text(label, style: const TextStyle(color: Colors.white70, fontSize: 14)),
        Text(value, style: TextStyle(color: valueColor, fontSize: 14, fontWeight: FontWeight.bold)),
      ],
    );
  }

  Widget _buildJoystickContainer(Widget child) {
    return Container(
      width: 140, height: 140,
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        gradient: RadialGradient(colors: [Colors.teal.shade900, Colors.black]),
        border: Border.all(color: Colors.teal.shade700, width: 2),
        boxShadow: [
          BoxShadow(color: Colors.teal.withOpacity(0.1), blurRadius: 10, spreadRadius: 2)
        ]
      ),
      child: child,
    );
  }
}

class RadarPainter extends CustomPainter {
  final double relX, relY, relTheta, maxDist;
  final bool isWarning, isEmergency;
  final List<Offset> history;
  final double collisionRadius;

  RadarPainter({
    required this.relX, required this.relY, required this.relTheta,
    required this.maxDist, required this.isWarning,
    required this.isEmergency, required this.history, required this.collisionRadius
  });

  Offset _clampToRadius(double mx, double my, double maxR) {
    double d = sqrt(mx * mx + my * my);
    if (d > maxR && d > 0) {
      return Offset((mx / d) * maxR, (my / d) * maxR);
    }
    return Offset(mx, my);
  }

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;

    final gridPaint = Paint()..color = Colors.teal.withOpacity(0.3)..style = PaintingStyle.stroke..strokeWidth = 1;
    canvas.drawCircle(center, radius, gridPaint);
    canvas.drawCircle(center, radius * 0.5, gridPaint);
    canvas.drawLine(Offset(center.dx, 0), Offset(center.dx, size.height), gridPaint);
    canvas.drawLine(Offset(0, center.dy), Offset(size.width, center.dy), gridPaint);

    double visualWarningRadius = (collisionRadius / maxDist) * radius;
    final warningPaint = Paint()
      ..color = Colors.red.withOpacity(0.15)
      ..style = PaintingStyle.fill;
    final warningBorderPaint = Paint()
      ..color = Colors.red.withOpacity(0.5)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;

    canvas.drawCircle(center, visualWarningRadius, warningPaint);
    canvas.drawCircle(center, visualWarningRadius, warningBorderPaint);

    if (history.isNotEmpty) {
      for (int i = 0; i < history.length - 1; i++) {
        double hx1 = (history[i].dx / maxDist) * radius;
        double hy1 = (-history[i].dy / maxDist) * radius;
        Offset clamped1 = _clampToRadius(hx1, hy1, radius);
        Offset pt1 = Offset(center.dx + clamped1.dx, center.dy + clamped1.dy);

        double hx2 = (history[i + 1].dx / maxDist) * radius;
        double hy2 = (-history[i + 1].dy / maxDist) * radius;
        Offset clamped2 = _clampToRadius(hx2, hy2, radius);
        Offset pt2 = Offset(center.dx + clamped2.dx, center.dy + clamped2.dy);

        double opacity = (i / history.length) * 0.7;
        Color trailColor = isEmergency ? Colors.orange : (isWarning ? Colors.red : Colors.blueAccent);

        final trailPaint = Paint()
          ..color = trailColor.withOpacity(opacity)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 3
          ..strokeCap = StrokeCap.round;

        canvas.drawLine(pt1, pt2, trailPaint);
      }
    }

    _drawArrow(canvas, center, 0, Colors.blueAccent);

    double mappedX = (relX / maxDist) * radius;
    double mappedY = (-relY / maxDist) * radius;

    Offset clampedCar = _clampToRadius(mappedX, mappedY, radius - 12);
    Offset otherPos = Offset(center.dx + clampedCar.dx, center.dy + clampedCar.dy);

    Color otherCarColor = isEmergency ? Colors.orange : (isWarning ? Colors.red : Colors.orange);
    _drawArrow(canvas, otherPos, -relTheta, otherCarColor);
  }

  void _drawArrow(Canvas canvas, Offset position, double rotation, Color color) {
    canvas.save();
    canvas.translate(position.dx, position.dy);
    canvas.rotate(rotation + (pi / 2));

    final carPaint = Paint()..color = color;
    final Path carShape = Path()
      ..moveTo(-8, 12)
      ..lineTo(8, 12)
      ..lineTo(8, -8)
      ..lineTo(0, -16)
      ..lineTo(-8, -8)
      ..close();

    canvas.drawPath(carShape, carPaint);
    canvas.restore();
  }

  @override
  bool shouldRepaint(RadarPainter oldDelegate) => true;
}