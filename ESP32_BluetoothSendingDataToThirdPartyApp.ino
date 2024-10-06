/*
    Based on Neil Kolban's example for IDF: 
    https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    Updates by Chegewara
    Code modified by LapLogic to work with the WATS (Wireless Automotive Telemetry) device

/*This program allow an ESP32-C3-Wroom-02 to send bluetooth packets and be connected to a bluetooth-capable device (for Apple Devices, you must use a Bluetooth Scanner application such as nrf Connect)*/

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

// Initialize the engine RPM variable, starting at 5. This value will be replaced with the actual RPM value received from a vehicle.
int engineRPM = 5;


void setup() {
  // Start the Serial communication at 115200 baud rate.
  Serial.begin(115200);
  Serial.println("Starting BLE work!");  // Print a message indicating BLE setup is starting.

  // Initialize the BLE device with a name (WATS).
  BLEDevice::init("WATS");

  // Create a BLE server instance.
  BLEServer *pServer = BLEDevice::createServer();

  // Create a BLE service using the defined SERVICE_UUID.
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic for the service with read and write properties.
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  // Set the initial value of the characteristic to "Engine RPM: 0".
  // This value will be sent over Bluetooth in UTF-8 format.
  pCharacteristic->setValue("Engine RPM: 0");
  
  // Start the service to make it available to clients.
  pService->start();

  // Get the advertising object for the BLE server.
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // Add the service UUID to the advertising data, allowing clients to discover the service.
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // Enable scan response for iPhone connection compatibility.
  pAdvertising->setScanResponse(true);

  // Set preferred connection parameters to improve compatibility with iPhones.
  pAdvertising->setMinPreferred(0x06);  // Minimum connection interval (in units of 1.25ms)
  pAdvertising->setMinPreferred(0x12);  // Maximum connection interval (in units of 1.25ms)

  // Start advertising the BLE device, making it discoverable by clients.
  BLEDevice::startAdvertising();

  // Print a message indicating that the characteristic is defined and ready to be read.
  Serial.println("WATS is ready to connect!");
}


void loop() {
  // Create a string representation of the current engine RPM value.
  String rpmStr = "Engine RPM: " + String(engineRPM);

  // Update the characteristic value with the new RPM string.
  // The c_str() method converts the String to a C-style string (null-terminated).
  pCharacteristic->setValue(rpmStr.c_str());

  // Optionally notify connected devices about the updated RPM value. This line may not be needed! 
  pCharacteristic->notify();  // This sends a notification to any connected clients.

  // Print the current RPM value to the Serial Monitor for debugging purposes.
  Serial.println(rpmStr);

  // Wait for 2 seconds before the next loop iteration.
  delay(2000);
}
