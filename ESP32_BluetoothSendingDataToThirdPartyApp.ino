/*
    Based on Neil Kolban's example for IDF: 
    https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    Updates by Chegewara
    Code modified by LapLogic to work with the WATS (Wireless Automotive Telemetry) device
*/

// Include necessary libraries for BLE functionality
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Define unique identifiers (UUIDs) for the service and characteristic.
// These UUIDs are used to identify the BLE service and the data being sent.
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Declare a global variable for the BLE characteristic.
BLECharacteristic *pCharacteristic;

// Initialize the engine RPM variable, starting at 5. This value will be incremented over time.
int engineRPM = 5;

// Callback class to handle characteristic events (e.g., when data is written)
class MyCallbacks : public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic *pCharacteristic) {
    // Get the data written by the phone and convert it to std::string
    std::string receivedData = pCharacteristic->getValue().c_str(); // Convert to std::string

    // Check if the received data is not empty
    if (receivedData.length() > 0) {
        Serial.print("Received data: ");
        Serial.println(receivedData.c_str());  // Print the received data
    }
}
};



void setup() {
  // Start the Serial communication at 115200 baud rate for debugging.
  Serial.begin(115200);
  Serial.println("Starting BLE work!");  // Print a message indicating BLE setup is starting.

  // Initialize the BLE device with the name "WATS".
  BLEDevice::init("WATS");

  // Create a BLE server instance that handles connections and services.
  BLEServer *pServer = BLEDevice::createServer();

  // Create a BLE service using the defined SERVICE_UUID.
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic for the service, with both read and write properties.
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  // Assign our custom callback class to handle incoming data.
  pCharacteristic->setCallbacks(new MyCallbacks());  // Use setCallbacks with a new instance of MyCallbacks

  // Set the initial value of the characteristic to "Engine RPM: 0".
  pCharacteristic->setValue("Engine RPM: 0");
  
  // Start the BLE service to make it available to clients (e.g., a phone).
  pService->start();

  // Get the advertising object for the BLE server.
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // Add the service UUID to the advertising data so clients can discover the service.
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // Enable scan response to improve compatibility with iPhones and other devices.
  pAdvertising->setScanResponse(true);

  // Set preferred connection parameters to improve compatibility with iPhones.
  pAdvertising->setMinPreferred(0x06);  // Minimum connection interval (in units of 1.25ms)
  pAdvertising->setMinPreferred(0x12);  // Maximum connection interval (in units of 1.25ms)

  // Start advertising the BLE device, making it discoverable by clients.
  BLEDevice::startAdvertising();

  // Print a message indicating that the characteristic is ready and the device is advertising.
  Serial.println("WATS is ready to connect!");
}

void loop() {
  // Increment the engine RPM by 100 each time the loop runs (every 2 seconds).
  engineRPM += 100;

  // Send updated RPM value over Bluetooth.
  sendBluetoothData();

  // Wait for 2 seconds before repeating the loop.
  delay(2000);
}

// Function to send Bluetooth data
void sendBluetoothData() {
  String rpmStr = "Engine RPM: " + String(engineRPM);  // Create a string representation of the current engine RPM value.
  pCharacteristic->setValue(rpmStr.c_str());  // Convert to C-string and set as characteristic value
  pCharacteristic->notify();                  // Notify connected devices with the updated value
  Serial.println(rpmStr);                     // Print for debugging
}

