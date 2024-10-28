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
bool alreadySent = true; //Illia, you do not need to add this

//Need to add these two classes. 
// Server callback class to handle connection and disconnection events
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
    alreadySent=false; //Illia, you dont need to add this
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();  // Restart advertising after disconnection
    Serial.println("Restarted advertising");
  }
};

// Characteristic callback class to handle characteristic events (e.g., when data is written)
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Get the data written by the phone and convert it to std::string
    std::string receivedData = pCharacteristic->getValue().c_str();  // Convert to std::string

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
  Serial.println("Starting BLE work!");

  // Initialize the BLE device with the name "WATS".
  BLEDevice::init("WATS");

  // Create a BLE server instance that handles connections and services.
  BLEServer *pServer = BLEDevice::createServer();

//Illia have to add this line
  // Set custom server callbacks for connection and disconnection events
  pServer->setCallbacks(new MyServerCallbacks());

  // Create a BLE service using the defined SERVICE_UUID.
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic for the service, with both read and write properties.
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

//Illia you will have to modify this line 
  // Assign our custom characteristic callback class to handle incoming data.
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Set the initial value of the characteristic to "Engine RPM: 0".
  pCharacteristic->setValue("Engine RPM: 0");

  // Start the BLE service to make it available to clients.
  pService->start();

  // Get the advertising object for the BLE server.
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // Add the service UUID to the advertising data so clients can discover the service.
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // Enable scan response to improve compatibility with iPhones and other devices.
  pAdvertising->setScanResponse(true);

  // Set preferred connection parameters.
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  // Start advertising the BLE device.
  BLEDevice::startAdvertising();

  Serial.println("WATS is ready to connect!");
}


//Illia, there is nothing new to add in the loop
void loop() {
  if (!alreadySent) {
    delay(10000);
    // Example: Sending 31 bits (all 0's for this example)
    uint8_t bits[4] = { 0 };  // Create an array to hold the bytes (4 bytes = 32 bits)

    // Set the first 3 bytes to 0x00 (8 bits each)
    // The last byte will contain only 7 bits, so we set it to 0x00 as well
    bits[0] = 0b10101010;  // 8 bits
    bits[1] = 0b10101010;  // 8 bits
    bits[2] = 0b10101010;  // 8 bits
    bits[3] = 0b1010101;   // 7 bits (to represent the 31st bit, the 8th bit is unused)

    // Send the bits
    pCharacteristic->setValue(bits, sizeof(bits));                    // Send the byte array
    pCharacteristic->notify();                                        // Notify connected devices with the updated value
    Serial.println("Sent 31 bits: 0000000000000000000000000000000");  // Debug output
    alreadySent = true;  // Set the flag to true to avoid resending
  }
}

