import 'dart:io';
import 'dart:convert';
import 'package:flutter/material.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Simple TCP Client',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const TcpClientScreen(),
    );
  }
}

class TcpClientScreen extends StatefulWidget {
  const TcpClientScreen({Key? key}) : super(key: key);

  @override
  _TcpClientScreenState createState() => _TcpClientScreenState();
}

class _TcpClientScreenState extends State<TcpClientScreen> {
  // TCP connection parameters
  final String serverAddress = '192.168.55.7'; // Change to your server IP
  final int serverPort = 8080; // Change to your server port
  
  // Connection state
  bool isConnecting = false;
  bool isConnected = false;
  Socket? socket;
  String statusMessage = 'Disconnected';
  List<String> receivedMessages = [];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Simple TCP Client'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Connection status
            Card(
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Server: $serverAddress:$serverPort',
                      style: const TextStyle(fontWeight: FontWeight.bold),
                    ),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Text('Status: '),
                        Text(
                          statusMessage,
                          style: TextStyle(
                            color: isConnected ? Colors.green : Colors.red,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                        if (isConnecting)
                          Padding(
                            padding: const EdgeInsets.only(left: 8.0),
                            child: SizedBox(
                              width: 16,
                              height: 16,
                              child: CircularProgressIndicator(strokeWidth: 2),
                            ),
                          ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 16),
            
            // Connect/Disconnect buttons
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                ElevatedButton(
                  onPressed: isConnected || isConnecting ? null : connectToServer,
                  child: const Text('Connect'),
                ),
                ElevatedButton(
                  onPressed: isConnected ? disconnectFromServer : null,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red,
                  ),
                  child: const Text('Disconnect'),
                ),
                ElevatedButton(
                  onPressed: isConnected ? sendTestMessage : null,
                  child: const Text('Send Test'),
                ),
              ],
            ),
            
            const SizedBox(height: 24),
            Text(
              'Received Messages:',
              style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
            ),
            const SizedBox(height: 8),
            Expanded(
              child: Container(
                padding: EdgeInsets.all(12),
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.grey),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: ListView.builder(
                  itemCount: receivedMessages.length,
                  itemBuilder: (context, index) {
                    return Padding(
                      padding: const EdgeInsets.symmetric(vertical: 2.0),
                      child: Text(receivedMessages[index]),
                    );
                  },
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Future<void> connectToServer() async {
    setState(() {
      isConnecting = true;
      statusMessage = 'Connecting...';
    });

    try {
      socket = await Socket.connect(
        serverAddress,
        serverPort,
        timeout: Duration(seconds: 5),
      );
      
      setState(() {
        isConnected = true;
        isConnecting = false;
        statusMessage = 'Connected';
      });
      
      // Add message listener
      socket!.listen(
        (List<int> data) {
          // Handle received data
          String message = utf8.decode(data);
          setState(() {
            receivedMessages.add('Received: $message');
          });
        },
        onError: (error) {
          print('Socket error: $error');
          setState(() {
            receivedMessages.add('Error: $error');
            disconnectFromServer();
          });
        },
        onDone: () {
          print('Server closed connection');
          setState(() {
            receivedMessages.add('Server closed connection');
            disconnectFromServer();
          });
        },
      );
      
      setState(() {
        receivedMessages.add('Connected to $serverAddress:$serverPort');
      });
    } catch (e) {
      print('Connection error: $e');
      setState(() {
        isConnected = false;
        isConnecting = false;
        statusMessage = 'Connection failed: ${e.toString()}';
        receivedMessages.add('Connection failed: ${e.toString()}');
      });
    }
  }
  
  void disconnectFromServer() {
    socket?.close();
    setState(() {
      isConnected = false;
      isConnecting = false;
      statusMessage = 'Disconnected';
      socket = null;
      receivedMessages.add('Disconnected from server');
    });
  }
  
  void sendTestMessage() {
    if (isConnected && socket != null) {
      String message = 'Hello from Flutter!';
      socket!.add(utf8.encode(message));
      setState(() {
        receivedMessages.add('Sent: $message');
      });
    }
  }
  
  @override
  void dispose() {
    socket?.close();
    super.dispose();
  }
}
