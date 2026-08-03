import 'package:flutter/material.dart';
import 'screens/bluetooth_screen.dart';
//import 'screens/control_screen.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const V2VApp());
}

class V2VApp extends StatelessWidget {
  const V2VApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Co-Drive',
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: Colors.black,
      ),
      // TEMPORARY BYPASS: Passing null directly
      //home: ControlScreen(connection: null),
      home: const BluetoothScreen(),
    );
  }
}