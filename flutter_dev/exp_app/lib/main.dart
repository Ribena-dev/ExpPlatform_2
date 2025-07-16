import 'dart:io';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart';
import 'package:flutter_ringtone_player/flutter_ringtone_player.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  runApp(MaterialApp(
    title: 'Navigation Basics',
    home: RouteSplash(),
  ));
}

class RouteSplash extends StatefulWidget {
  @override
  _RouteSplashState createState() => _RouteSplashState();
}

class _RouteSplashState extends State<RouteSplash> {
  bool shouldProceed = false;

  _fetchPrefs() async {
    await Future.delayed(Duration(seconds: 1));
    setState(() {
      shouldProceed = true;
    });
  }

  @override
  void initState() {
    super.initState();
    _fetchPrefs();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: shouldProceed
            ? ElevatedButton(
                onPressed: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute(builder: (context) => MyApp()),
                  );
                },
                child: Text("Start"),
              )
            : CircularProgressIndicator(),
      ),
    );
  }
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Node server demo',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: Scaffold(
        appBar: AppBar(
          title: Text('Monkey Experiment Image'),
          actions: [
            IconButton(
              icon: Icon(Icons.settings),
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => SettingsPage()),
                );
              },
            ),
          ],
        ),
        body: BodyWidget(),
      ),
    );
  }
}

class SettingsPage extends StatefulWidget {
  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final _ipController = TextEditingController();
  final _portController = TextEditingController();
  final _formKey = GlobalKey<FormState>();
  
  @override
  void initState() {
    super.initState();
    _loadSettings();
  }
  
  _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    setState(() {
      _ipController.text = prefs.getString('server_ip') ?? '192.168.55.7';
      _portController.text = prefs.getString('server_port') ?? '8080';
    });
  }
  
  _saveSettings() async {
    if (_formKey.currentState!.validate()) {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('server_ip', _ipController.text);
      await prefs.setString('server_port', _portController.text);
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Settings saved! Restart app to apply changes.'))
      );
    }
  }
  
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Connection Settings'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Form(
          key: _formKey,
          child: Column(
            children: [
              TextFormField(
                controller: _ipController,
                decoration: InputDecoration(
                  labelText: 'Server IP Address',
                  hintText: 'e.g. 192.168.55.7',
                ),
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter an IP address';
                  }
                  return null;
                },
              ),
              SizedBox(height: 16),
              TextFormField(
                controller: _portController,
                decoration: InputDecoration(
                  labelText: 'Server Port',
                  hintText: 'e.g. 8080',
                ),
                keyboardType: TextInputType.number,
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Please enter a port number';
                  }
                  final port = int.tryParse(value);
                  if (port == null || port <= 0 || port > 65535) {
                    return 'Please enter a valid port number (1-65535)';
                  }
                  return null;
                },
              ),
              SizedBox(height: 24),
              ElevatedButton(
                onPressed: _saveSettings,
                child: Text('Save Settings'),
              ),
            ],
          ),
        ),
      ),
    );
  }
  
  @override
  void dispose() {
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }
}

class LogViewPage extends StatelessWidget {
  final List<String> logs;
  
  LogViewPage({required this.logs});
  
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Connection Logs'),
        actions: [
          IconButton(
            icon: Icon(Icons.clear),
            onPressed: () {
              Navigator.pop(context, true);
            },
          ),
        ],
      ),
      body: ListView.builder(
        itemCount: logs.length,
        itemBuilder: (context, index) {
          final log = logs[index];
          final isError = log.contains('error') || log.contains('Error');
          
          return Padding(
            padding: const EdgeInsets.symmetric(vertical: 4.0, horizontal: 8.0),
            child: Text(
              log,
              style: TextStyle(
                color: isError ? Colors.red : Colors.black,
                fontFamily: 'monospace',
              ),
            ),
          );
        },
      ),
    );
  }
}

class BodyWidget extends StatefulWidget {
  @override
  BodyWidgetState createState() {
    return new BodyWidgetState();
  }
}

class BodyWidgetState extends State<BodyWidget> {
  String server_addr = '192.168.55.7';
  int server_port = 8080;
  
  static const String img_camel      = 'assets/pic_camel.png';
  static const String img_cat        = 'assets/pic_cat.png';
  static const String img_crocodile  = 'assets/pic_crocodile.png';
  static const String img_donkey     = 'assets/pic_donkey.png';
  static const String img_pig        = 'assets/pic_pig.png';
  static const String img_rabbit     = 'assets/pic_rabbit.png';

  bool isConnecting = false;
  bool isConnect = false;
  Socket? socket;
  List<String> logs = [];

  String img = "";
  bool isFullScreen = false;

  @override
  void initState() {
    super.initState();
    _loadSettings();
  }
  
  _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    setState(() {
      server_addr = prefs.getString('server_ip') ?? '192.168.55.7';
      server_port = int.parse(prefs.getString('server_port') ?? '8080');
    });
    _connect();
  }

  void _addLog(String message) {
    setState(() {
      logs.add("[${DateTime.now().toString().split('.').first}] $message");
    });
  }

  Widget _mainDisplay() {
    if (isConnect) return _imageWidget();
    else if (isConnecting) return _loadingWidget();
    else return _connectWidget();
  }  

  Widget _connectWidget() {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        ElevatedButton(
          child: Text('Reconnect: $server_addr:$server_port', 
                    style: TextStyle(color: Colors.red)),
          onPressed: () {
            _connect();
          },
        ),
        SizedBox(height: 16),
        ElevatedButton(
          child: Text('View Logs (${logs.length})'),
          onPressed: () {
            _showLogs();
          },
        ),
      ],
    );
  }  

  Widget _loadingWidget(){
    return Column(
      crossAxisAlignment: CrossAxisAlignment.center,
      mainAxisSize: MainAxisSize.max,
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        CircularProgressIndicator(),
        SizedBox(height: 16),
        Text('Connecting: $server_addr:$server_port'),
        SizedBox(height: 16),
        ElevatedButton(
          child: Text('View Logs (${logs.length})'),
          onPressed: () {
            _showLogs();
          },
        ),
      ],      
    );
  }

  Widget _imageWidget(){
    return GestureDetector(
      onTap: () {
        setState(() {
          isFullScreen = !isFullScreen;
        });
      },
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          if (img.isNotEmpty) 
            isFullScreen
              ? Expanded(
                  child: Container(
                    width: double.infinity,
                    child: Image.asset(
                      img,
                      fit: BoxFit.contain,
                    ),
                  ),
                )
              : Image.asset(img)
          else 
            Text('Ready', style: TextStyle(color: Colors.green)),
          
          if (!isFullScreen) 
            Padding(
              padding: const EdgeInsets.only(top: 16.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  IconButton(
                    icon: Icon(Icons.fullscreen),
                    onPressed: () {
                      setState(() {
                        isFullScreen = true;
                      });
                    },
                    tooltip: 'Full Screen',
                  ),
                  IconButton(
                    icon: Icon(Icons.list),
                    onPressed: () {
                      _showLogs();
                    },
                    tooltip: 'View Logs',
                  ),
                ],
              ),
            ),
        ],
      ),
    );
  }

  Future<void> _showLogs() async {
    final result = await Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => LogViewPage(logs: logs),
      ),
    );
    
    if (result == true) {
      setState(() {
        logs.clear();
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return isFullScreen
      ? _imageWidget()
      : Padding(
          padding: const EdgeInsets.all(16.0),
          child: Center(
            child: _mainDisplay(),
          ),
        );
  }

  _connect() async {
    _addLog('Connecting to $server_addr:$server_port');
    print('connecting');
    setState(() {
      isConnecting = true;
    });  

    try {
      socket = await Socket.connect(server_addr, server_port);
      _addLog('Connected successfully');
      print('connected');
      setState(() {
        isConnect = true;
        isConnecting = false;
      });
      socket?.listen(
        (List<int> event) {
          String msg = utf8.decode(event); 
          _addLog('Received message: $msg');
          print(msg);
          _processCmd(msg);
        },
        onError: (error) {
          _addLog('Socket error: $error');
          _disconnect();
        },
        onDone: () {
          _addLog('Connection closed by server');
          _disconnect();
        },
      );
    } catch (error) {
      _addLog('Connection error: $error');
      print(error);
      setState(() {
        isConnect = false;
        isConnecting = false;
      });
    }
  }

  _processCmd(String msg) async {
    String temp;
    switch(msg.trim()) {
      case '0': { 
          temp = img_cat;
          _addLog('Displaying cat image');
      } 
      break; 
      
      case '1': { 
          temp = img_camel;
          _addLog('Displaying camel image');
      } 
      break;

      case '2': { 
        temp = img_rabbit;
        _addLog('Displaying rabbit image');
      } 
      break;

      case '3': { 
        temp = img_donkey;
        _addLog('Displaying donkey image');
      } 
      break;

      case '4': { 
        temp = img_crocodile;
        _addLog('Displaying crocodile image');
      } 
      break;
      
      case '5': { 
        temp = img_pig;
        _addLog('Displaying pig image');
      } 
      break;

      default: { 
        temp = "";
        _addLog('Received unknown command: $msg');
      }
      break; 
    } 
    setState(() {
      this.img = temp;
    });
  }

  _makeTcpMsg() async {
    socket?.add(utf8.encode('hello'));
    _addLog('Sent message: hello');

    await Future.delayed(Duration(seconds: 5));
  }

  _disconnect() async {
    socket?.close();
    _addLog('Disconnected from server');
    print('disconnected');

    setState(() {
      isConnect = false;
      isConnecting = false;
    });    
  }
}