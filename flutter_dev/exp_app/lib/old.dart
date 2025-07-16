# 0 "main.dart"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 0 "<command-line>" 2
# 1 "main.dart"
import 'dart:io';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart';
import 'package:flutter_ringtone_player/flutter_ringtone_player.dart';


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
            ? RaisedButton(
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
        appBar: AppBar(title: Text('Monkey Experiment Image')),
        body: BodyWidget(),
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
  static const String server_addr = '192.168.1.130';
  static const int server_port = 8080;

  static const String img_camel = 'assets/pic_camel.png';
  static const String img_cat = 'assets/pic_cat.png';
  static const String img_crocodile = 'assets/pic_crocodile.png';
  static const String img_donkey = 'assets/pic_donkey.png';
  static const String img_pig = 'assets/pic_pig.png';
  static const String img_rabbit = 'assets/pic_rabbit.png';

  bool isConnecting = false;
  bool isConnect = false;
  Socket socket;

  String img ;

  @override
  void initState() {
    super.initState();
    _connect();
  }

  Widget _mainDisplay() {
    if (isConnect) return _imageWidget();
    else if (isConnecting) return _loadingWidget();
    else return _connectWidget();

  }

  Widget _connectWidget() {
    return RaisedButton(
                child: Text('Reconnect: ' + server_addr + ':' + server_port.toString(),
                              style: TextStyle(color: Colors.red)),
                onPressed: () {
                  _connect();
                },
              );
  }

  Widget _loadingWidget(){
    return Column(
      crossAxisAlignment: CrossAxisAlignment.center,
      mainAxisSize: MainAxisSize.max,
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        CircularProgressIndicator(),
        Text('Connecting: ' + server_addr + ':' + server_port.toString() ),
      ],
    );
  }

  Widget _imageWidget(){
    return img != null ? Image.asset(img) : Text('Ready',style: TextStyle(color: Colors.green));
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(32.0),
      child: Center(
         child: _mainDisplay(),

      ),
    );
  }

  _connect() async {
    print('connecting');
    setState(() {
      isConnecting = true;
    });

    await Socket.connect(server_addr, server_port).then( (newSocket) {
      socket = newSocket;
      print('connected');
      setState(() {
        isConnect = true;
        isConnecting = false;
      });
      socket.listen((List<int> event) {
        String msg = utf8.decode(event);
        print(msg);

        _processCmd(msg);

      });
    }).catchError( (error){
      print (error);
      setState(() {
        isConnect = false;
        isConnecting = false;
      });
      }
    );

  }

  _processCmd(String msg) async {
    String temp;
    switch(msg) {
      case '0': {
          temp = img_camel;
      }
      break;

      case '1': {
          temp = img_cat;
      }
      break;

      case '2': {
        temp = img_crocodile;
      }
      break;

      case '3': {
        temp = img_donkey;
      }
      break;

      case '4': {
        temp = img_pig;
      }
      break;
      case '5': {
        temp = img_rabbit;
      }
      break;

      default: {
        temp = null;
      }
      break;
    }
    setState(() {
      this.img = temp;
    });
  }

  _makeTcpMsg() async {

    socket.add(utf8.encode('hello'));




    await Future.delayed(Duration(seconds: 5));

  }

  _disconnect() async {

    socket.close();
    print('disconnected');

    setState(() {
      isConnect = false;
      isConnecting = false;
    });
  }

}
