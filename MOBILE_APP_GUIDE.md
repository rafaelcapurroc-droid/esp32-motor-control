# Mobile App Development Guide

## Technology Stack Options

### Option 1: Flutter (Recommended for Cross-Platform)
- **Pros**: Single codebase for iOS & Android, excellent BLE support, fast performance
- **Cons**: New framework if unfamiliar
- **Package**: `flutter_blue_plus` (formerly `flutter_blue`)

### Option 2: React Native
- **Pros**: JavaScript-based, code sharing between platforms
- **Cons**: Slightly slower than Flutter, steeper learning curve
- **Package**: `react-native-ble-plx` or `react-native-ble-plus`

### Option 3: Native Development
- **Pros**: Best performance, access to all platform features
- **Cons**: Maintain separate codebases for Android and iOS
- **Android**: Android Studio + Kotlin/Java + Android BLE API
- **iOS**: Xcode + Swift + Core Bluetooth

---

## Recommended: Flutter Implementation

### Setup
```bash
# Install Flutter
# https://flutter.dev/docs/get-started/install

# Create new project
flutter create motor_control_app
cd motor_control_app

# Add BLE package
flutter pub add flutter_blue_plus
flutter pub add charts_flutter    # For graphing
flutter pub add intl              # For date/time formatting
```

### Core BLE Connection Code

```dart
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'dart:convert';

class MotorController {
  late BluetoothDevice device;
  late BluetoothCharacteristic charVelocity;
  late BluetoothCharacteristic charCurrent;
  late BluetoothCharacteristic charStatus;
  
  // BLE UUIDs
  final String serviceUUID = '12345678-1234-5678-1234-56789abcdef0';
  final String charVelocityUUID = '87654321-4321-8765-4321-fedcba987654';
  final String charCurrentUUID = '11111111-2222-3333-4444-555555555555';
  final String charStatusUUID = '22222222-3333-4444-5555-666666666666';

  Future<void> connect(BluetoothDevice device) async {
    this.device = device;
    await device.connect();
    
    // Discover services
    List<BluetoothService> services = await device.discoverServices();
    
    for (var service in services) {
      if (service.uuid.toString() == serviceUUID) {
        for (var characteristic in service.characteristics) {
          switch (characteristic.uuid.toString()) {
            case charVelocityUUID:
              charVelocity = characteristic;
              break;
            case charCurrentUUID:
              charCurrent = characteristic;
              // Enable notifications
              await charCurrent.setNotifyValue(true);
              break;
            case charStatusUUID:
              charStatus = characteristic;
              // Enable notifications
              await charStatus.setNotifyValue(true);
              break;
          }
        }
      }
    }
  }

  Future<void> setVelocity(int velocity) async {
    Map<String, dynamic> data = {'velocity': velocity};
    String json = jsonEncode(data);
    await charVelocity.write(utf8.encode(json));
  }

  Future<void> setDirection(int direction) async {
    Map<String, dynamic> data = {'direction': direction};
    String json = jsonEncode(data);
    // Find direction characteristic and write
  }

  Stream<MotorStatus> getStatusUpdates() {
    return charStatus.onValueReceived.map((value) {
      Map<String, dynamic> data = jsonDecode(utf8.decode(value));
      return MotorStatus.fromJson(data);
    });
  }

  Stream<CurrentReading> getCurrentUpdates() {
    return charCurrent.onValueReceived.map((value) {
      Map<String, dynamic> data = jsonDecode(utf8.decode(value));
      return CurrentReading.fromJson(data);
    });
  }

  Future<void> disconnect() async {
    await device.disconnect();
  }
}

class MotorStatus {
  final int velocity;
  final int direction;
  final double current;
  final int timestamp;

  MotorStatus({
    required this.velocity,
    required this.direction,
    required this.current,
    required this.timestamp,
  });

  factory MotorStatus.fromJson(Map<String, dynamic> json) {
    return MotorStatus(
      velocity: json['velocity'] ?? 0,
      direction: json['direction'] ?? 1,
      current: (json['current'] ?? 0.0).toDouble(),
      timestamp: json['timestamp'] ?? 0,
    );
  }
}

class CurrentReading {
  final double current;
  final int timestamp;

  CurrentReading({
    required this.current,
    required this.timestamp,
  });

  factory CurrentReading.fromJson(Map<String, dynamic> json) {
    return CurrentReading(
      current: (json['current'] ?? 0.0).toDouble(),
      timestamp: json['timestamp'] ?? 0,
    );
  }
}
```

### UI Components

#### Scanner Screen
```dart
class ScannerScreen extends StatefulWidget {
  @override
  _ScannerScreenState createState() => _ScannerScreenState();
}

class _ScannerScreenState extends State<ScannerScreen> {
  List<ScanResult> results = [];
  bool isScanning = false;

  @override
  void initState() {
    super.initState();
    startScan();
  }

  void startScan() async {
    setState(() => isScanning = true);
    
    FlutterBluePlus.startScan(
      timeout: Duration(seconds: 4),
    );

    FlutterBluePlus.scanResults.listen((scanResults) {
      setState(() => results = scanResults);
    });

    FlutterBluePlus.isScanning.listen((isScanning) {
      setState(() => this.isScanning = isScanning);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('Scan BLE Devices')),
      body: ListView(
        children: results
            .where((r) => r.device.name.contains('Motor'))
            .map((r) => ListTile(
              title: Text(r.device.name),
              subtitle: Text(r.device.id.id),
              onTap: () {
                // Connect to device
                Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (_) => ControlScreen(device: r.device),
                  ),
                );
              },
            ))
            .toList(),
      ),
    );
  }
}
```

#### Control Screen with Graphs
```dart
class ControlScreen extends StatefulWidget {
  final BluetoothDevice device;

  const ControlScreen({required this.device});

  @override
  _ControlScreenState createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  late MotorController controller;
  List<FlSpot> velocityData = [];
  List<FlSpot> currentData = [];
  int dataPoints = 0;

  @override
  void initState() {
    super.initState();
    controller = MotorController();
    _connect();
  }

  void _connect() async {
    await controller.connect(widget.device);
    
    // Listen to status updates
    controller.getStatusUpdates().listen((status) {
      setState(() {
        velocityData.add(FlSpot(dataPoints.toDouble(), status.velocity.toDouble()));
        if (velocityData.length > 50) velocityData.removeAt(0);
      });
    });

    // Listen to current updates
    controller.getCurrentUpdates().listen((reading) {
      setState(() {
        currentData.add(FlSpot(dataPoints.toDouble(), reading.current));
        if (currentData.length > 50) currentData.removeAt(0);
        dataPoints++;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('Motor Control')),
      body: SingleChildScrollView(
        child: Column(
          children: [
            // Velocity Control
            Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('Velocity Control', style: Theme.of(context).textTheme.headline6),
                  Slider(
                    min: 0,
                    max: 255,
                    divisions: 255,
                    label: 'Velocity',
                    onChanged: (value) {
                      controller.setVelocity(value.toInt());
                    },
                  ),
                ],
              ),
            ),

            // Direction Control
            Padding(
              padding: EdgeInsets.symmetric(horizontal: 16),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  ElevatedButton(
                    onPressed: () => controller.setDirection(1),
                    child: Text('Forward'),
                  ),
                  ElevatedButton(
                    onPressed: () => controller.setDirection(-1),
                    child: Text('Reverse'),
                  ),
                ],
              ),
            ),

            // Velocity Chart
            Padding(
              padding: EdgeInsets.all(16),
              child: LineChart(
                LineChartData(
                  borderData: FlBorderData(show: true),
                  lineBarsData: [
                    LineChartBarData(
                      spots: velocityData,
                      isCurved: true,
                      color: Colors.blue,
                    ),
                  ],
                ),
              ),
            ),

            // Current Chart
            Padding(
              padding: EdgeInsets.all(16),
              child: LineChart(
                LineChartData(
                  borderData: FlBorderData(show: true),
                  lineBarsData: [
                    LineChartBarData(
                      spots: currentData,
                      isCurved: true,
                      color: Colors.red,
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    controller.disconnect();
    super.dispose();
  }
}
```

---

## Testing Checklist

- [ ] ESP32 advertises as `ESP32_MotorControl`
- [ ] App discovers and connects to device
- [ ] Velocity slider updates motor speed
- [ ] Direction buttons change rotation
- [ ] Current graph shows live readings
- [ ] Graphs update with correct timestamps
- [ ] Disconnect/reconnect works properly
- [ ] Works on both Android and iOS

---

## Resources

- Flutter BLE Package: https://pub.dev/packages/flutter_blue_plus
- Flutter Charts: https://pub.dev/packages/fl_chart
- Flutter Setup: https://flutter.dev/docs/get-started/install
- BLE Testing: Use `BLE Scanner` app from Google Play / App Store
