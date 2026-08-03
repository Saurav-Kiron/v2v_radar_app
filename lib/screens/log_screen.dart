import 'package:flutter/material.dart';

class LogScreen extends StatelessWidget {
  final ValueNotifier<List<String>> logNotifier;

  const LogScreen({Key? key, required this.logNotifier}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        title: const Text("Live Data Stream", style: TextStyle(color: Colors.white)),
        backgroundColor: Colors.red.shade900,
        iconTheme: const IconThemeData(color: Colors.white),
        actions: [
          IconButton(
            icon: const Icon(Icons.delete_sweep),
            onPressed: () {
              logNotifier.value = [];
            },
          )
        ],
      ),
      body: SafeArea(
        child: ValueListenableBuilder<List<String>>(
          valueListenable: logNotifier,
          builder: (context, logs, child) {
            if (logs.isEmpty) {
              return const Center(
                child: Text("Waiting for data...", style: TextStyle(color: Colors.grey)),
              );
            }

            return ListView.builder(
              itemCount: logs.length,
              itemBuilder: (context, index) {
                String log = logs[index];

                Color textColor = Colors.white;
                if (log.startsWith("SND")) {
                  textColor = Colors.blueAccent;
                } else if (log.contains("S,")) {
                  textColor = Colors.greenAccent;
                } else if (log.contains("O,")) {
                  textColor = Colors.orangeAccent;
                }

                return Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
                  child: Text(
                    log,
                    style: TextStyle(
                      color: textColor,
                      fontFamily: 'monospace',
                      fontSize: 13
                    ),
                  ),
                );
              },
            );
          },
        ),
      ),
    );
  }
}