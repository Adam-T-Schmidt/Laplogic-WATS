/**
 * Project: OBD2 reder for Camosun CAPSTONE project for ESP32-C3-DevKitC-02, group LAPLOGIC
 * Description: Simulation send request car for PIDs option, send to phione via BLE, and waith for any requst from phone
 * and simulate send request to car, get responce and send back to phone
 * Board: ESP32-C3-DevKitC-02
 * WATSesp32firmwareFinal
 * Author: Illia Pavych, Adam Schmidt, Matthew Pearson, Alex Moore
 * Date: 12.12.2024
 */

#include "driver/twai.h"

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

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Polling interval (ms)
#define POLLING_RATE_MS 1000

#define MAX_PACKETS 100  // Maximum number of packets
#define PACKET_SIZE 2    // Size of each data packet
#define DATA_TIMEOUT 6   // Timeout for data transfer completion (in milliseconds)

char dataBuffer[MAX_PACKETS][PACKET_SIZE + 1];  // Array for storing packets (add 1 for the terminating '\0' character)
int packetIndex = 0;                            // Current packet index
unsigned long lastDataTime = 0;                 // Time of last received packet

int countBLE = 0;

static bool driver_installed = false;
unsigned long lastRequestTime = 0;

static String currentDataType = "";  // Current request type (DTCs, RPM, etc.)
static bool isRunning = false;      // Flag for monitoring the system state

static bool pidExecuted = false;  // Flag for tracking the execution of PIDs

String lastReceivedData = "";

uint8_t SaveDataType;
int32_t SaveDataValue;

String blePIDMessage = "";

int PIDsHEX = 0;
int CountPIDs = 0;
uint32_t binaryString;

bool GGreset = 0;

// Variable for storing data received via BLE
String receivedData = "";


// Function for processing received data
void processReceivedData() {
  Serial.println("Analyze data...");

  for (int i = 0; i < packetIndex; i++) {
    String test = "";
    test = dataBuffer[i];
    Serial.print("Packet ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(dataBuffer[i]);  // Output the packet as a string
    if(test == "GG"){
    resetDataBuffer();
    GGreset = 1;
    Serial.println("I reset buffer and set that to GG");
    }
  }

  // Data processing logic can be added here
}

// Function to clear the data buffer after the transfer is complete
void resetDataBuffer() {
  memset(dataBuffer, 0, sizeof(dataBuffer));  // Clear the array
  packetIndex = 0;                            // Reset the packet index
}


// Server callback class to handle connection and disconnection events
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();  // Restart advertising after disconnection
    Serial.println("Restarted advertising");
    ESP.restart();
  }
};


// Callback for processing received data
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() == PACKET_SIZE) {  // Check that the packet contains 2 characters
      //Copy two characters to the buffer and add the final character
      strncpy(dataBuffer[packetIndex], rxValue.c_str(), PACKET_SIZE);
      dataBuffer[packetIndex][PACKET_SIZE] = '\0';  // Add a null character to the string

      // Increment the packet index and update the last data reception time
      packetIndex++;
      lastDataTime = millis();
    }

    // Check if more time has passed than the timeout
    if (millis() - lastDataTime > DATA_TIMEOUT) {
      // All packets received, data can be processed
      processReceivedData();

      // Clearing data after processing
      //resetDataBuffer();
    }
  }
};


void setup() {
  // Start Serial:
  Serial.begin(500000);

  // Initialize the BLE device with a name (WATS).
  BLEDevice::init("WATS");
  // Create a BLE server instance.
  BLEServer *pServer = BLEDevice::createServer();
  // Set custom server callbacks for connection and disconnection events
  pServer->setCallbacks(new MyServerCallbacks());
  // Create a BLE service using the defined SERVICE_UUID.
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE characteristic for the service with read and write properties.
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  // Устанавливаем колбэк для обработки записи данных в характеристику
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Set the initial value of the characteristic to "".
  // This value will be sent over Bluetooth in UTF-8 format.
  pCharacteristic->setValue("");
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
  Serial.println("");
  Serial.println("WATS is ready to connect!");

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // You can adjust the speed if necessary
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  driver_installed = true;
  randomSeed(analogRead(0));
}


void sendPIDsQuery() {
  // Create a CAN message for the OBD-II request
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  queryMessage.data[2] = 0x00;  // PIDs supported [01-20]
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("PIDs[01-20] query sent successfully");
  } else {
    Serial.println("Failed to send next PIDs[01-20] query");
  }
}

void sendSecondPIDsQuery() {
  // Create a CAN message for the next range of PIDs (20-40)
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  queryMessage.data[2] = 0x20;  // PID for PIDs supported [21-40]
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("PIDs[21-40] query sent successfully");
  } else {
    Serial.println("Failed to send next PIDs[21-40] query");
  }
}

void sendThirdPIDsQuery() {
  // Create a CAN message for the next range of PIDs (40-60)
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  queryMessage.data[2] = 0x40;  // PID for PIDs supported [41-60]
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("PIDs[41-60] query sent successfully");
  } else {
    Serial.println("Failed to send next PIDs[41-60] query");
  }
}

void sendFourthPIDsQuery() {
  // Create a CAN message for the next range of PIDs (60-80)
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  queryMessage.data[2] = 0x60;  // PID for PIDs supported [61-80]
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("PIDs[61-80] query sent successfully");
  } else {
    Serial.println("Failed to send next PIDs[61-80] query");
  }
}

void sendFifthPIDsQuery() {
  // Create a CAN message for the next range of PIDs (80-A0)
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  queryMessage.data[2] = 0x80;  // PID for PIDs supported [81-A0]
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("PIDs[81-A0] query sent successfully");
  } else {
    Serial.println("Failed to send next PIDs[81-A0] query");
  }
}

void sendSixthPIDsQuery() {
  // Create a CAN message for the next range of PIDs (A1-C0)
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  queryMessage.data[2] = 0xA0;  // PID for PIDs supported [A1-C0]
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("PIDs[A2-C0] query sent successfully");
  } else {
    Serial.println("Failed to send next PIDs[A2-C0] query");
  }
}


void sendQuery(const String &dataType) {
  twai_message_t queryMessage;
  if (dataType.equalsIgnoreCase("DTCs")) {
    queryMessage.data[2] = 0x01;
  } else if (dataType.equalsIgnoreCase("Freeze DTC")) {
    queryMessage.data[2] = 0x02;
  } else if (dataType.equalsIgnoreCase("Fuel system status")) {
    queryMessage.data[2] = 0x03;
  } else if (dataType.equalsIgnoreCase("Calculated engine load")) {
    queryMessage.data[2] = 0x04;
  } else if (dataType.equalsIgnoreCase("Coolant")) {
    queryMessage.data[2] = 0x05;
  } else if (dataType.equalsIgnoreCase("STFT1")) {
    queryMessage.data[2] = 0x06;
  } else if (dataType.equalsIgnoreCase("LTFT1")) {
    queryMessage.data[2] = 0x07;
  } else if (dataType.equalsIgnoreCase("STFT2")) {
    queryMessage.data[2] = 0x08;
  } else if (dataType.equalsIgnoreCase("LTFT2")) {
    queryMessage.data[2] = 0x09;
  } else if (dataType.equalsIgnoreCase("Fuel pressure")) {
    queryMessage.data[2] = 0x0A;
  } else if (dataType.equalsIgnoreCase("IMAP")) {
    queryMessage.data[2] = 0x0B;
  } else if (dataType.equalsIgnoreCase("RPM")) {
    queryMessage.data[2] = 0x0C;
  } else if (dataType.equalsIgnoreCase("Speed")) {
    queryMessage.data[2] = 0x0D;
  } else if (dataType.equalsIgnoreCase("Timing advance")) {
    queryMessage.data[2] = 0x0E;
  } else if (dataType.equalsIgnoreCase("IAT")) {
    queryMessage.data[2] = 0x0F;
  } else if (dataType.equalsIgnoreCase("MAFSAFR")) {
    queryMessage.data[2] = 0x10;
  } else if (dataType.equalsIgnoreCase("Throttle position")) {
    queryMessage.data[2] = 0x11;
  } else if (dataType.equalsIgnoreCase("CSAS")) {
    queryMessage.data[2] = 0x12;
  } else if (dataType.equalsIgnoreCase("OS1V")) {
    queryMessage.data[2] = 0x14;
  } else if (dataType.equalsIgnoreCase("OS2V")) {
    queryMessage.data[2] = 0x15;
  } else if (dataType.equalsIgnoreCase("OS3V")) {
    queryMessage.data[2] = 0x16;
  } else if (dataType.equalsIgnoreCase("OS4V")) {
    queryMessage.data[2] = 0x17;
  } else if (dataType.equalsIgnoreCase("OS5V")) {
    queryMessage.data[2] = 0x18;
  } else if (dataType.equalsIgnoreCase("OS6V")) {
    queryMessage.data[2] = 0x19;
  } else if (dataType.equalsIgnoreCase("OS7V")) {
    queryMessage.data[2] = 0x1A;
  } else if (dataType.equalsIgnoreCase("OS8V")) {
    queryMessage.data[2] = 0x1B;
  } else if (dataType.equalsIgnoreCase("OBD standards")) {
    queryMessage.data[2] = 0x1C;
  } else if (dataType.equalsIgnoreCase("Run time")) {
    queryMessage.data[2] = 0x1F;
  } else if (dataType.equalsIgnoreCase("Distance traveled")) {
    queryMessage.data[2] = 0x21;
  } else if (dataType.equalsIgnoreCase("Fuel rail pres")) {
    queryMessage.data[2] = 0x22;
  } else if (dataType.equalsIgnoreCase("Fuel rail gauge pres")) {
    queryMessage.data[2] = 0x23;
  } else if (dataType.equalsIgnoreCase("OS1A")) {
    queryMessage.data[2] = 0x24;
  } else if (dataType.equalsIgnoreCase("OS2A")) {
    queryMessage.data[2] = 0x25;
  } else if (dataType.equalsIgnoreCase("OS3A")) {
    queryMessage.data[2] = 0x26;
  } else if (dataType.equalsIgnoreCase("OS4A")) {
    queryMessage.data[2] = 0x27;
  } else if (dataType.equalsIgnoreCase("OS5A")) {
    queryMessage.data[2] = 0x28;
  } else if (dataType.equalsIgnoreCase("OS6A")) {
    queryMessage.data[2] = 0x29;
  } else if (dataType.equalsIgnoreCase("OS7A")) {
    queryMessage.data[2] = 0x2A;
  } else if (dataType.equalsIgnoreCase("OS8A")) {
    queryMessage.data[2] = 0x2B;
  } else if (dataType.equalsIgnoreCase("Commanded EGR")) {
    queryMessage.data[2] = 0x2C;
  } else if (dataType.equalsIgnoreCase("EGR Error")) {
    queryMessage.data[2] = 0x2D;
  } else if (dataType.equalsIgnoreCase("CEP")) {
    queryMessage.data[2] = 0x2E;
  } else if (dataType.equalsIgnoreCase("FTLI")) {
    queryMessage.data[2] = 0x2F;
  } else if (dataType.equalsIgnoreCase("WSDTCsC")) {
    queryMessage.data[2] = 0x30;
  } else if (dataType.equalsIgnoreCase("DTSDTCsC")) {
    queryMessage.data[2] = 0x31;
  } else if (dataType.equalsIgnoreCase("ESVP")) {
    queryMessage.data[2] = 0x32;
  } else if (dataType.equalsIgnoreCase("ABP")) {
    queryMessage.data[2] = 0x33;
  } else if (dataType.equalsIgnoreCase("OS1A2")) {
    queryMessage.data[2] = 0x34;
  } else if (dataType.equalsIgnoreCase("OS2A2")) {
    queryMessage.data[2] = 0x35;
  } else if (dataType.equalsIgnoreCase("OS3A2")) {
    queryMessage.data[2] = 0x36;
  } else if (dataType.equalsIgnoreCase("OS4A2")) {
    queryMessage.data[2] = 0x37;
  } else if (dataType.equalsIgnoreCase("OS5A2")) {
    queryMessage.data[2] = 0x38;
  } else if (dataType.equalsIgnoreCase("OS6A2")) {
    queryMessage.data[2] = 0x39;
  } else if (dataType.equalsIgnoreCase("OS7A2")) {
    queryMessage.data[2] = 0x3A;
  } else if (dataType.equalsIgnoreCase("OS8A2")) {
    queryMessage.data[2] = 0x3B;
  } else if (dataType.equalsIgnoreCase("CTb1s1")) {
    queryMessage.data[2] = 0x3C;
  } else if (dataType.equalsIgnoreCase("CTb2s1")) {
    queryMessage.data[2] = 0x3D;
  } else if (dataType.equalsIgnoreCase("CTb1s2")) {
    queryMessage.data[2] = 0x3E;
  } else if (dataType.equalsIgnoreCase("CTb2s2")) {
    queryMessage.data[2] = 0x3F;
  } else if (dataType.equalsIgnoreCase("MSTDC")) {
    queryMessage.data[2] = 0x41;
  } else if (dataType.equalsIgnoreCase("CMV")) {
    queryMessage.data[2] = 0x42;
  } else if (dataType.equalsIgnoreCase("ALV")) {
    queryMessage.data[2] = 0x43;
  } else if (dataType.equalsIgnoreCase("CAFER")) {
    queryMessage.data[2] = 0x44;
  } else if (dataType.equalsIgnoreCase("RTP")) {
    queryMessage.data[2] = 0x45;
  } else if (dataType.equalsIgnoreCase("AAT")) {
    queryMessage.data[2] = 0x46;
  } else if (dataType.equalsIgnoreCase("ATPB")) {
    queryMessage.data[2] = 0x47;
  } else if (dataType.equalsIgnoreCase("ATPC")) {
    queryMessage.data[2] = 0x48;
  } else if (dataType.equalsIgnoreCase("ATPD")) {
    queryMessage.data[2] = 0x49;
  } else if (dataType.equalsIgnoreCase("ATPE")) {
    queryMessage.data[2] = 0x4A;
  } else if (dataType.equalsIgnoreCase("ATPBF")) {
    queryMessage.data[2] = 0x4B;
  } else if (dataType.equalsIgnoreCase("CTA")) {
    queryMessage.data[2] = 0x4C;
  } else if (dataType.equalsIgnoreCase("TRWMon")) {
    queryMessage.data[2] = 0x4D;
  } else if (dataType.equalsIgnoreCase("TSDTCsC")) {
    queryMessage.data[2] = 0x4E;
  } else if (dataType.equalsIgnoreCase("MFAER")) {
    queryMessage.data[2] = 0x4F;
  } else if (dataType.equalsIgnoreCase("MAFRFMAFS")) {
    queryMessage.data[2] = 0x50;
  } else if (dataType.equalsIgnoreCase("Fuel type")) {
    queryMessage.data[2] = 0x51;
  } else if (dataType.equalsIgnoreCase("Ethanol fuel percentage")) {
    queryMessage.data[2] = 0x52;
  } else if (dataType.equalsIgnoreCase("AESVP")) {
    queryMessage.data[2] = 0x53;
  } else if (dataType.equalsIgnoreCase("ESVP54")) {
    queryMessage.data[2] = 0x54;
  } else if (dataType.equalsIgnoreCase("STSOTB1")) {
    queryMessage.data[2] = 0x55;
  } else if (dataType.equalsIgnoreCase("LTSOTB1")) {
    queryMessage.data[2] = 0x56;
  } else if (dataType.equalsIgnoreCase("STSOTB2")) {
    queryMessage.data[2] = 0x57;
  } else if (dataType.equalsIgnoreCase("LTSOTB2")) {
    queryMessage.data[2] = 0x58;
  } else if (dataType.equalsIgnoreCase("FRAP")) {
    queryMessage.data[2] = 0x59;
  } else if (dataType.equalsIgnoreCase("RAPP")) {
    queryMessage.data[2] = 0x5A;
  } else if (dataType.equalsIgnoreCase("HBPRL")) {
    queryMessage.data[2] = 0x5B;
  } else if (dataType.equalsIgnoreCase("EOT")) {
    queryMessage.data[2] = 0x5C;
  } else if (dataType.equalsIgnoreCase("FIT")) {
    queryMessage.data[2] = 0x5D;
  } else if (dataType.equalsIgnoreCase("ER")) {
    queryMessage.data[2] = 0x5F;
  } else {
    Serial.println("Unknown data type requested.");
  }
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;  // Number of additional data bytes
  queryMessage.data[1] = 0x01;  // Service ID for "Show Current Data"
  //queryMessage.data[2] = 0x05;      // PID for Engine RPM
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;  // Fill the rest of the bytes with 0
  }

  // Send the request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("Query sent successfully");
  } else {
    Serial.println("Failed to send query");
  }
}
// //////////////////////////////////////////////////
// //////////////////////////////////////////////////
// //////////////////////////////////////////////////



void handleReceivedMessage(twai_message_t &message) {
  //check if we got right package
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x00 && message.data_length_code >= 6) {
    //add all bit data to one number
    uint32_t PIDs0120 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


    //print raw number for debuting
    Serial.printf("ID: %x\nByte:", message.identifier);
    Serial.print("PIDs0120 sum of data: ");
    Serial.println(PIDs0120);

    Serial.print("Binary: ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((PIDs0120 >> i) & 1);
    }
    Serial.println();

    binaryString = PIDs0120;
    pCharacteristic->setValue((uint8_t *)&binaryString, sizeof(binaryString));
    pCharacteristic->notify();

    // Check bits:
    Serial.println("Options :");

    Serial.print("Monitor status since DTCs cleared: ");
    Serial.println((PIDs0120 & (1 << 31)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Freeze DTC: ");
    Serial.println((PIDs0120 & (1 << 30)) ? "yes" : "no");

    Serial.print("Fuel system status: ");
    Serial.println((PIDs0120 & (1 << 29)) ? "yes" : "no");

    Serial.print("Calculated engine load: ");
    Serial.println((PIDs0120 & (1 << 28)) ? "yes" : "no");

    Serial.print("Engine coolant temperature: ");
    Serial.println((PIDs0120 & (1 << 27)) ? "yes" : "no");

    Serial.print("Short term fuel trim (bank 1): ");
    Serial.println((PIDs0120 & (1 << 26)) ? "yes" : "no");

    Serial.print("Long term fuel trim (bank 1): ");
    Serial.println((PIDs0120 & (1 << 25)) ? "yes" : "no");

    Serial.print("Short term fuel trim (bank 2): ");
    Serial.println((PIDs0120 & (1 << 24)) ? "yes" : "no");

    Serial.print("Long term fuel trim (bank 2): ");
    Serial.println((PIDs0120 & (1 << 23)) ? "yes" : "no");

    Serial.print("Fuel pressure (gauge pressure): ");
    Serial.println((PIDs0120 & (1 << 22)) ? "yes" : "no");

    Serial.print("Intake manifold absolute pressure: ");
    Serial.println((PIDs0120 & (1 << 21)) ? "yes" : "no");

    Serial.print("Engine speed: ");
    Serial.println((PIDs0120 & (1 << 20)) ? "yes" : "no");

    Serial.print("Vehicle speed: ");
    Serial.println((PIDs0120 & (1 << 19)) ? "yes" : "no");

    Serial.print("Timing advance: ");
    Serial.println((PIDs0120 & (1 << 18)) ? "yes" : "no");

    Serial.print("Intake air temperature: ");
    Serial.println((PIDs0120 & (1 << 17)) ? "yes" : "no");

    Serial.print("Mass air flow sensor air flow rate: ");
    Serial.println((PIDs0120 & (1 << 16)) ? "yes" : "no");

    Serial.print("Throttle position: ");
    Serial.println((PIDs0120 & (1 << 15)) ? "yes" : "no");

    Serial.print("Commanded secondary air status: ");
    Serial.println((PIDs0120 & (1 << 14)) ? "yes" : "no");

    Serial.print("Oxygen sensors present (2 banks): ");
    Serial.println((PIDs0120 & (1 << 13)) ? "yes" : "no");

    Serial.print("Oxygen sensor 1 (voltage): ");
    Serial.println((PIDs0120 & (1 << 12)) ? "yes" : "no");

    Serial.print("Oxygen sensor 2 (voltage): ");
    Serial.println((PIDs0120 & (1 << 11)) ? "yes" : "no");

    Serial.print("Oxygen sensor 3 (voltage): ");
    Serial.println((PIDs0120 & (1 << 10)) ? "yes" : "no");

    Serial.print("Oxygen sensor 4 (voltage): ");
    Serial.println((PIDs0120 & (1 << 9)) ? "yes" : "no");

    Serial.print("Oxygen sensor 5 (voltage): ");
    Serial.println((PIDs0120 & (1 << 8)) ? "yes" : "no");

    Serial.print("Oxygen sensor 6 (voltage): ");
    Serial.println((PIDs0120 & (1 << 7)) ? "yes" : "no");

    Serial.print("Oxygen sensor 7 (voltage): ");
    Serial.println((PIDs0120 & (1 << 6)) ? "yes" : "no");

    Serial.print("Oxygen sensor 8 (voltage): ");
    Serial.println((PIDs0120 & (1 << 5)) ? "yes" : "no");

    Serial.print("OBD standards the vehicle conforms to: ");
    Serial.println((PIDs0120 & (1 << 4)) ? "yes" : "no");

    Serial.print("Oxygen sensors present (4 banks): ");
    Serial.println((PIDs0120 & (1 << 3)) ? "yes" : "no");

    Serial.print("Auxiliary input status: ");
    Serial.println((PIDs0120 & (1 << 2)) ? "yes" : "no");

    Serial.print("Run time since engine start: ");
    Serial.println((PIDs0120 & (1 << 1)) ? "yes" : "no");

    Serial.print("PIDs supported [21 - 40] ");
    Serial.println((PIDs0120 & (1 << 0)) ? "yes" : "no");


    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }
    if (PIDs0120 & (1 << 0)) {
      Serial.println("");
      Serial.println("PIDs supported [21-40] are available, sending next query.");
      delay(100);
      sendSecondPIDsQuery();
      //}
      //}
    } else {
      isRunning = false;    // Set the flag that the system has stopped
      pidExecuted = false;  // Reset the command execution flag
      countBLE = 0;
      receivedData = "";
      currentDataType = "";
    }
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x20 && message.data_length_code >= 6) {
    //add all bit data to one number
    uint32_t PIDs2140 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    //print raw number for debuting
    Serial.printf("ID: %x\nByte:", message.identifier);
    Serial.print("PIDs2140 sum of data: ");
    Serial.println(PIDs2140);

    Serial.print("Binary: ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((PIDs2140 >> i) & 1);
    }
    Serial.println();

    //binaryString = PIDs2140;
    //pCharacteristic->setValue((uint8_t*)&binaryString, sizeof(binaryString));
    //pCharacteristic->notify();

    Serial.println("Options :");

    Serial.print("Distance traveled with MIL on: ");
    Serial.println((PIDs2140 & (1 << 31)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Fuel rail pres. (rel. to manifold vacuum): ");
    Serial.println((PIDs2140 & (1 << 30)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Fuel rail gauge pres. (diesel, gas inject): ");
    Serial.println((PIDs2140 & (1 << 29)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 1 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 28)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 2 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 27)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 3 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 26)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 4 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 25)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 5 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 24)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 6 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 23)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 7 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 22)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 8 (air-fuel equiv. ratio):");
    Serial.println((PIDs2140 & (1 << 21)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Commanded EGR: ");
    Serial.println((PIDs2140 & (1 << 20)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("EGR Error: ");
    Serial.println((PIDs2140 & (1 << 19)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Commanded evaporative purge: ");
    Serial.println((PIDs2140 & (1 << 18)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Fuel tank level input: ");
    Serial.println((PIDs2140 & (1 << 17)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Warmups since DTCs cleared: ");
    Serial.println((PIDs2140 & (1 << 16)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Distance traveled since DTCs cleared: ");
    Serial.println((PIDs2140 & (1 << 15)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Evap. system vapor pressure: ");
    Serial.println((PIDs2140 & (1 << 14)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Absolute barometric pressure: ");
    Serial.println((PIDs2140 & (1 << 13)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 1 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 12)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 2 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 11)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 3 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 10)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 4 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 9)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 5 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 8)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 6 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 7)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 7 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 6)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Oxygen sensor 8 (air-fuel equiv. ratio): ");
    Serial.println((PIDs2140 & (1 << 5)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Catalyst temperature (bank 1, sensor 1): ");
    Serial.println((PIDs2140 & (1 << 4)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Catalyst temperature (bank 2, sensor 1): ");
    Serial.println((PIDs2140 & (1 << 3)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Catalyst temperature (bank 1, sensor 2): ");
    Serial.println((PIDs2140 & (1 << 2)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Catalyst temperature (bank 2, sensor 2): ");
    Serial.println((PIDs2140 & (1 << 1)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("PIDs supported [21 - 40]: ");
    Serial.println((PIDs2140 & (1 << 0)) ? "yes" : "no");

    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }

    if (PIDs2140 & (1 << 0)) {
      Serial.println("");
      Serial.println("PIDs supported [41-60] are available, sending next query.");
      delay(100);

      sendThirdPIDsQuery();
    } else {
      isRunning = false;    // Set the flag that the system has stopped
      pidExecuted = false;  // Reset the command execution flag
      countBLE = 0;
      receivedData = "";
      currentDataType = "";
    }
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x40 && message.data_length_code >= 6) {
    //add all bit data to one number
    uint32_t PIDs4160 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);

    //print raw number for debuting
    Serial.printf("ID: %x\nByte:", message.identifier);
    Serial.print("PIDs4160 sum of data: ");
    Serial.println(PIDs4160);

    Serial.print("Binary: ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((PIDs4160 >> i) & 1);
    }
    Serial.println();

    //binaryString = PIDs4160;
    //pCharacteristic->setValue((uint8_t*)&binaryString, sizeof(binaryString));
    //pCharacteristic->notify();

    Serial.println("Options :");

    Serial.print("Monitor status this drive cycle: ");
    Serial.println((PIDs4160 & (1 << 31)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Control module voltage: ");
    Serial.println((PIDs4160 & (1 << 30)) ? "yes" : "no");

    Serial.print("Absolute load value: ");
    Serial.println((PIDs4160 & (1 << 29)) ? "yes" : "no");

    Serial.print("Commanded air-fuel equiv. ratio: ");
    Serial.println((PIDs4160 & (1 << 28)) ? "yes" : "no");

    Serial.print("Relative throttle position: ");
    Serial.println((PIDs4160 & (1 << 27)) ? "yes" : "no");

    Serial.print("Ambient air temperature: ");
    Serial.println((PIDs4160 & (1 << 26)) ? "yes" : "no");

    Serial.print("Absolute throttle position B: ");
    Serial.println((PIDs4160 & (1 << 25)) ? "yes" : "no");

    Serial.print("Absolute throttle position C: ");
    Serial.println((PIDs4160 & (1 << 24)) ? "yes" : "no");

    Serial.print("Accelerator pedal position D: ");
    Serial.println((PIDs4160 & (1 << 23)) ? "yes" : "no");

    Serial.print("Accelerator pedal position E: ");
    Serial.println((PIDs4160 & (1 << 22)) ? "yes" : "no");

    Serial.print("Accelerator pedal position F: ");
    Serial.println((PIDs4160 & (1 << 21)) ? "yes" : "no");

    Serial.print("Commanded throttle actuator: ");
    Serial.println((PIDs4160 & (1 << 20)) ? "yes" : "no");

    Serial.print("Time run with MIL on: ");
    Serial.println((PIDs4160 & (1 << 19)) ? "yes" : "no");

    Serial.print("Time since DTCs cleared: ");
    Serial.println((PIDs4160 & (1 << 18)) ? "yes" : "no");

    Serial.print("Max fuel-air equiv. ratio: ");
    Serial.println((PIDs4160 & (1 << 17)) ? "yes" : "no");

    Serial.print("Max air flow rate from MAF sensor: ");
    Serial.println((PIDs4160 & (1 << 16)) ? "yes" : "no");

    Serial.print("Fuel type: ");
    Serial.println((PIDs4160 & (1 << 15)) ? "yes" : "no");

    Serial.print("Ethanol fuel percentage: ");
    Serial.println((PIDs4160 & (1 << 14)) ? "yes" : "no");

    Serial.print("Absolute evap system vapor pressure: ");
    Serial.println((PIDs4160 & (1 << 13)) ? "yes" : "no");

    Serial.print("Evap system vapor pressure: ");
    Serial.println((PIDs4160 & (1 << 12)) ? "yes" : "no");

    Serial.print("Short term sec. oxygen trim (bank 1): ");
    Serial.println((PIDs4160 & (1 << 11)) ? "yes" : "no");

    Serial.print("Long term sec. oxygen trim (bank 1): ");
    Serial.println((PIDs4160 & (1 << 10)) ? "yes" : "no");

    Serial.print("Short term sec. oxygen trim (bank 2): ");
    Serial.println((PIDs4160 & (1 << 9)) ? "yes" : "no");

    Serial.print("Long term sec. oxygen trim (bank 2): ");
    Serial.println((PIDs4160 & (1 << 8)) ? "yes" : "no");

    Serial.print("Fuel rail absolute pressure: ");
    Serial.println((PIDs4160 & (1 << 7)) ? "yes" : "no");

    Serial.print("Relative accelerator pedal position: ");
    Serial.println((PIDs4160 & (1 << 6)) ? "yes" : "no");

    Serial.print("Hybrid battery pack remaining life: ");
    Serial.println((PIDs4160 & (1 << 5)) ? "yes" : "no");

    Serial.print("Engine oil temperature: ");
    Serial.println((PIDs4160 & (1 << 4)) ? "yes" : "no");

    Serial.print("Fuel injection timing: ");
    Serial.println((PIDs4160 & (1 << 3)) ? "yes" : "no");

    Serial.print("Engine fuel rate: ");
    Serial.println((PIDs4160 & (1 << 2)) ? "yes" : "no");

    Serial.print("Emission requirements: ");
    Serial.println((PIDs4160 & (1 << 1)) ? "yes" : "no");

    Serial.print("PIDs supported [61 - 80] ");
    Serial.println((PIDs4160 & (1 << 0)) ? "yes" : "no");

    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }

    if (PIDs4160 & (1 << 0)) {
      Serial.println("");
      Serial.println("PIDs supported [61-80] are available, sending next query.");
      delay(100);
      sendFourthPIDsQuery();
      //  }
      //}
    } else {
      isRunning = false;    // Set the flag that the system has stopped
      pidExecuted = false;  // Reset the command execution flag
      countBLE = 0;
      receivedData = "";
      currentDataType = "";
    }
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x60 && message.data_length_code >= 6) {
    //add all bit data to one number
    uint32_t PIDs6180 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);

    //print raw number for debuting
    Serial.printf("ID: %x\nByte:", message.identifier);
    Serial.print("PIDs6180 sum of data: ");
    Serial.println(PIDs6180);

    Serial.print("Binary: ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((PIDs6180 >> i) & 1);
    }
    Serial.println();

    //binaryString = PIDs6180;
    //pCharacteristic->setValue((uint8_t*)&binaryString, sizeof(binaryString));
    //pCharacteristic->notify();

    Serial.println("Options :");

    Serial.print("Demanded engine percent torque: ");
    Serial.println((PIDs6180 & (1 << 31)) ? "yes" : "no");

    Serial.print("Actual engine percent torque: ");
    Serial.println((PIDs6180 & (1 << 30)) ? "yes" : "no");

    Serial.print("Engine reference torque: ");
    Serial.println((PIDs6180 & (1 << 29)) ? "yes" : "no");

    Serial.print("Engine pct. torque (idle): ");
    Serial.println((PIDs6180 & (1 << 28)) ? "yes" : "no");

    Serial.print("Auxiliary input/output supported: ");
    Serial.println((PIDs6180 & (1 << 27)) ? "yes" : "no");

    Serial.print("Mass air flow sensor (A): ");
    Serial.println((PIDs6180 & (1 << 26)) ? "yes" : "no");

    Serial.print("Engine coolant temperature (sensor 1): ");
    Serial.println((PIDs6180 & (1 << 25)) ? "yes" : "no");

    Serial.print("Intake air temperature (sensor 1): ");
    Serial.println((PIDs6180 & (1 << 24)) ? "yes" : "no");

    Serial.print("Commanded EGR and EGR error: ");
    Serial.println((PIDs6180 & (1 << 23)) ? "yes" : "no");

    Serial.print("Com. diesel intake air flow ctr/position: ");
    Serial.println((PIDs6180 & (1 << 22)) ? "yes" : "no");

    Serial.print("Exhaust gas recirculation temperature: ");
    Serial.println((PIDs6180 & (1 << 21)) ? "yes" : "no");

    Serial.print("Com. throttle actuator ctr./position: ");
    Serial.println((PIDs6180 & (1 << 20)) ? "yes" : "no");

    Serial.print("Fuel pressure control system: ");
    Serial.println((PIDs6180 & (1 << 19)) ? "yes" : "no");

    Serial.print("Injection pressure control system: ");
    Serial.println((PIDs6180 & (1 << 18)) ? "yes" : "no");

    Serial.print("Turbocharger compressor inlet pres.: ");
    Serial.println((PIDs6180 & (1 << 17)) ? "yes" : "no");

    Serial.print("Boost pressure control: ");
    Serial.println((PIDs6180 & (1 << 16)) ? "yes" : "no");

    Serial.print("Variable geometry turbo control: ");
    Serial.println((PIDs6180 & (1 << 15)) ? "yes" : "no");

    Serial.print("Wastegate control: ");
    Serial.println((PIDs6180 & (1 << 14)) ? "yes" : "no");

    Serial.print("Exhaust pressure: ");
    Serial.println((PIDs6180 & (1 << 13)) ? "yes" : "no");

    Serial.print("Turbocharger RPM: ");
    Serial.println((PIDs6180 & (1 << 12)) ? "yes" : "no");

    Serial.print("Turbocharger temperature: ");
    Serial.println((PIDs6180 & (1 << 11)) ? "yes" : "no");

    Serial.print("Turbocharger temperature: ");
    Serial.println((PIDs6180 & (1 << 10)) ? "yes" : "no");

    Serial.print("Charge air cooler temperature: ");
    Serial.println((PIDs6180 & (1 << 9)) ? "yes" : "no");

    Serial.print("EGT (bank 1): ");
    Serial.println((PIDs6180 & (1 << 8)) ? "yes" : "no");

    Serial.print("EGT (bank 2): ");
    Serial.println((PIDs6180 & (1 << 7)) ? "yes" : "no");

    Serial.print("Diesel particulate filter - diff. pressure: ");
    Serial.println((PIDs6180 & (1 << 6)) ? "yes" : "no");

    Serial.print("Diesel particulate filter: ");
    Serial.println((PIDs6180 & (1 << 5)) ? "yes" : "no");

    Serial.print("Diesel particulate filter - temperature: ");
    Serial.println((PIDs6180 & (1 << 4)) ? "yes" : "no");

    Serial.print("NOx NTE control area status: ");
    Serial.println((PIDs6180 & (1 << 3)) ? "yes" : "no");

    Serial.print("PM NTE control area status: ");
    Serial.println((PIDs6180 & (1 << 2)) ? "yes" : "no");

    Serial.print("Engine run time: ");
    Serial.println((PIDs6180 & (1 << 1)) ? "yes" : "no");

    Serial.print("PIDs supported [81 - A0]: ");
    Serial.println((PIDs6180 & (1 << 0)) ? "yes" : "no");

    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }

    if (PIDs6180 & (1 << 0)) {
      Serial.println("");
      Serial.println("PIDs supported [61-80] are available, sending next query.");
      delay(100);
      sendFifthPIDsQuery();
    } else {
      isRunning = false;    // Set the flag that the system has stopped
      pidExecuted = false;  // Reset the command execution flag
      countBLE = 0;
      receivedData = "";
      currentDataType = "";
    }
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x80 && message.data_length_code >= 6) {
    //add all bit data to one number
    uint32_t PIDs81A0 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    //print raw number for debuting
    Serial.printf("ID: %x\nByte:", message.identifier);
    Serial.print("PIDs4160 sum of data: ");
    Serial.println(PIDs81A0);

    Serial.print("Binary: ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((PIDs81A0 >> i) & 1);
    }
    Serial.println();


    //binaryString = PIDs81A0;
    //pCharacteristic->setValue((uint8_t*)&binaryString, sizeof(binaryString));
    //pCharacteristic->notify();

    Serial.println("Options :");

    Serial.print("Engine run time for AECD: ");
    Serial.println((PIDs81A0 & (1 << 31)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Engine run time for AECD: ");
    Serial.println((PIDs81A0 & (1 << 30)) ? "yes" : "no");

    Serial.print("NOx sensor: ");
    Serial.println((PIDs81A0 & (1 << 29)) ? "yes" : "no");

    Serial.print("Manifold surface temperature: ");
    Serial.println((PIDs81A0 & (1 << 28)) ? "yes" : "no");

    Serial.print("NOx reagent system: ");
    Serial.println((PIDs81A0 & (1 << 27)) ? "yes" : "no");

    Serial.print("Particulate matter sensor: ");
    Serial.println((PIDs81A0 & (1 << 26)) ? "yes" : "no");

    Serial.print("Intake manifold absolute pressure: ");
    Serial.println((PIDs81A0 & (1 << 25)) ? "yes" : "no");

    Serial.print("SCR induce system: ");
    Serial.println((PIDs81A0 & (1 << 24)) ? "yes" : "no");

    Serial.print("Run time for AECD #11-#15: ");
    Serial.println((PIDs81A0 & (1 << 23)) ? "yes" : "no");

    Serial.print("Run time for AECD #16-#20: ");
    Serial.println((PIDs81A0 & (1 << 22)) ? "yes" : "no");

    Serial.print("Diesel aftertreatment: ");
    Serial.println((PIDs81A0 & (1 << 21)) ? "yes" : "no");

    Serial.print("O2 sensor (wide range): ");
    Serial.println((PIDs81A0 & (1 << 20)) ? "yes" : "no");

    Serial.print("Throttle position G: ");
    Serial.println((PIDs81A0 & (1 << 19)) ? "yes" : "no");

    Serial.print("Engine friction percent torque: ");
    Serial.println((PIDs81A0 & (1 << 18)) ? "yes" : "no");

    Serial.print("Particulate matter sensor (bank 1 & 2): ");
    Serial.println((PIDs81A0 & (1 << 17)) ? "yes" : "no");

    Serial.print("WWH-OBD vehicle OBD system Info: ");
    Serial.println((PIDs81A0 & (1 << 16)) ? "yes" : "no");

    Serial.print("WWH-OBD vehicle OBD system Info: ");
    Serial.println((PIDs81A0 & (1 << 15)) ? "yes" : "no");

    Serial.print("Fuel system control: ");
    Serial.println((PIDs81A0 & (1 << 14)) ? "yes" : "no");

    Serial.print("WWH-OBD counters support: ");
    Serial.println((PIDs81A0 & (1 << 13)) ? "yes" : "no");

    Serial.print("NOx warning and inducement system: ");
    Serial.println((PIDs81A0 & (1 << 12)) ? "yes" : "no");

    //Serial.print("Engine run time for AECD: ");
    //Serial.println((PIDs81A0 & (1 << 11)) ? "yes" : "no");

    //Serial.print("Engine run time for AECD: ");
    //Serial.println((PIDs81A0 & (1 << 10)) ? "yes" : "no");

    //Serial.print("Engine run time for AECD: ");
    //Serial.println((PIDs81A0 & (1 << 9)) ? "yes" : "no");

    Serial.print("EGT sensor: ");
    Serial.println((PIDs81A0 & (1 << 8)) ? "yes" : "no");

    Serial.print("EGT sensor: ");
    Serial.println((PIDs81A0 & (1 << 7)) ? "yes" : "no");

    Serial.print("Hybrid/EV sys. data, battery, voltage: ");
    Serial.println((PIDs81A0 & (1 << 6)) ? "yes" : "no");

    Serial.print("Diesel exhaust fluid sensor data: ");
    Serial.println((PIDs81A0 & (1 << 5)) ? "yes" : "no");

    Serial.print("O2 sensor data: ");
    Serial.println((PIDs81A0 & (1 << 4)) ? "yes" : "no");

    Serial.print("Engine fuel rate: ");
    Serial.println((PIDs81A0 & (1 << 3)) ? "yes" : "no");

    Serial.print("Engine exhaust flow rate: ");
    Serial.println((PIDs81A0 & (1 << 2)) ? "yes" : "no");

    Serial.print("Fuel system percentage use: ");
    Serial.println((PIDs81A0 & (1 << 1)) ? "yes" : "no");

    Serial.print("PIDs supported [A1 - C0]: ");
    Serial.println((PIDs81A0 & (1 << 0)) ? "yes" : "no");

    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }

    if (PIDs81A0 & (1 << 0)) {
      Serial.println("");
      Serial.println("PIDs supported [A1-C0] are available, sending next query.");
      delay(100);
      sendSixthPIDsQuery();
    } else {
      isRunning = false;    // Set the flag that the system has stopped
      pidExecuted = false;  // Reset the command execution flag
      countBLE = 0;
      receivedData = "";
      currentDataType = "";
    }
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0xA0 && message.data_length_code >= 6) {
    //add all bit data to one number
    uint32_t PIDsA1C0 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    //print raw number for debuting
    Serial.printf("ID: %x\nByte:", message.identifier);
    Serial.print("PIDsA1C0 sum of data: ");
    Serial.println(PIDsA1C0);

    Serial.print("Binary: ");
    for (int i = 31; i >= 0; i--) {
      Serial.print((PIDsA1C0 >> i) & 1);
    }
    Serial.println();


    //binaryString = PIDsA1C0;
    //pCharacteristic->setValue((uint8_t*)&binaryString, sizeof(binaryString));
    //pCharacteristic->notify();

    Serial.println("Options :");

    Serial.print("NOx sensor corrected data: ");
    Serial.println((PIDsA1C0 & (1 << 31)) ? "yes" : "no");  // Check first bit(bit 31)

    Serial.print("Cylinder fuel rate: ");
    Serial.println((PIDsA1C0 & (1 << 30)) ? "yes" : "no");

    Serial.print("Evap system vapor pressure: ");
    Serial.println((PIDsA1C0 & (1 << 29)) ? "yes" : "no");

    Serial.print("Transmission actual gear: ");
    Serial.println((PIDsA1C0 & (1 << 28)) ? "yes" : "no");

    Serial.print("Cmd. diesel exhaust fluid dosing: ");
    Serial.println((PIDsA1C0 & (1 << 27)) ? "yes" : "no");

    Serial.print("Odometer: ");
    Serial.println((PIDsA1C0 & (1 << 26)) ? "yes" : "no");

    Serial.print("NOx concentration 3, 4: ");
    Serial.println((PIDsA1C0 & (1 << 25)) ? "yes" : "no");

    Serial.print("NOx corrected concentration (3, 4): ");
    Serial.println((PIDsA1C0 & (1 << 24)) ? "yes" : "no");

    Serial.print("PIDs supported [C1 - E0]: ");
    Serial.println((PIDsA1C0 & (1 << 0)) ? "yes" : "no");


    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }

    if (PIDsA1C0 & (1 << 0)) {
      Serial.println("PIDs supported [C1-E0] are available, but i dont know how decode that, bmw pls send me manual for car 0_0.");
    }
    isRunning = false;    // Set the flag that the system has stopped
    pidExecuted = false;  // Reset the command execution flag
    countBLE = 0;
    receivedData = "";
    currentDataType = "";
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x01 && message.data_length_code >= 6) {
    int DTCs = (message.data[3]);
    Serial.printf("Monitor status since DTCs cleared: %d \n", DTCs);
    SaveDataType = message.data[2];
    SaveDataValue = DTCs * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x02 && message.data_length_code >= 6) {
    int FreezeDTC = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Freeze DTC: %d \n", FreezeDTC);
    SaveDataType = message.data[2];
    SaveDataValue = FreezeDTC * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x03 && message.data_length_code >= 6) {
    int FuelSystemStatus = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Fuel system status: %d \n", FuelSystemStatus);
    SaveDataType = message.data[2];
    SaveDataValue = FuelSystemStatus * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x04 && message.data_length_code >= 4) {
    float CalculatedEngineLoad = (message.data[3] * (1.0 / 2.55));
    Serial.printf("Calculated engine load: %.2f %% \n", CalculatedEngineLoad);
    SaveDataType = message.data[2];
    SaveDataValue = CalculatedEngineLoad * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x05 && message.data_length_code >= 4) {
    int Coolant = (-40 + (message.data[3]));
    Serial.printf("Coolant Temp: %d degC\n", Coolant);
    SaveDataType = message.data[2];
    SaveDataValue = Coolant * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x06 && message.data_length_code >= 4) {
    float STFT1 = (-100 + (message.data[3] * (1.0 / 1.28)));
    Serial.printf("Short term fuel trim(bank 1): %.2f %% \n", STFT1);
    SaveDataType = message.data[2];
    SaveDataValue = STFT1 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x07 && message.data_length_code >= 4) {
    float LTFT1 = (-100 + (message.data[3] * (1.0 / 1.28)));
    Serial.printf("Long term fuel trim(bank 1): %.2f %% \n", LTFT1);
    SaveDataType = message.data[2];
    SaveDataValue = LTFT1 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x08 && message.data_length_code >= 4) {
    float STFT2 = (-100 + (message.data[3] * (1.0 / 1.28)));
    Serial.printf("Short term fuel trim(bank 2): %.2f %% \n", STFT2);
    SaveDataType = message.data[2];
    SaveDataValue = STFT2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x09 && message.data_length_code >= 4) {
    float LTFT2 = (-100 + (message.data[3] * (1.0 / 1.28)));
    Serial.printf("Long term fuel trim(bank 2): %.2f %% \n", LTFT2);
    SaveDataType = message.data[2];
    SaveDataValue = LTFT2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0A && message.data_length_code >= 4) {
    int FuelPressure = ((message.data[3]) * 3);
    Serial.printf("Fuel Pressure: %d kPa \n", FuelPressure);
    SaveDataType = message.data[2];
    SaveDataValue = FuelPressure * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0B && message.data_length_code >= 4) {
    int IntakeManifoldAbsolutePressure = (message.data[3]);
    Serial.printf("Intake manifold absolute pressure: %d kPa \n", IntakeManifoldAbsolutePressure);
    SaveDataType = message.data[2];
    SaveDataValue = IntakeManifoldAbsolutePressure * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x0C && message.data_length_code >= 4) {
    int rpm = ((message.data[3] << 8) | message.data[4]) / 4;
    Serial.printf("Engine RPM: %d\n", rpm);
    SaveDataType = message.data[2];
    SaveDataValue = rpm * 100;
    Serial.printf("SaveDataValue: %d\n", SaveDataValue);
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0D && message.data_length_code >= 4) {
    int Speed = (message.data[3]);
    Serial.printf("Vehicle speed: %d km/h \n", Speed);
    SaveDataType = message.data[2];
    SaveDataValue = Speed * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0E && message.data_length_code >= 4) {
    int TimingAdvance = (-64 + (message.data[3] * 0.5));
    Serial.printf("Timing advance: %d deg \n", TimingAdvance);
    SaveDataType = message.data[2];
    SaveDataValue = TimingAdvance * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0F && message.data_length_code >= 4) {
    int IntakeAirTemperature = (-40 + (message.data[3]));
    Serial.printf("Intake air temperature: %d degC \n", IntakeAirTemperature);
    SaveDataType = message.data[2];
    SaveDataValue = IntakeAirTemperature * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x10 && message.data_length_code >= 6) {
    float MassAirFlowSensorAirFlowRate = ((message.data[3] << 8) | message.data[4]) * 0.01;
    Serial.printf("Mass air flow sensor air flow rate: %.2f grams/sec \n", MassAirFlowSensorAirFlowRate);
    SaveDataType = message.data[2];
    SaveDataValue = MassAirFlowSensorAirFlowRate * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x11 && message.data_length_code >= 4) {
    float ThrottlePosition = ((message.data[3]) * (1.00 / 2.55));
    Serial.printf("Throttle position: %.2f %% \n", ThrottlePosition);
    SaveDataType = message.data[2];
    SaveDataValue = ThrottlePosition * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x12 && message.data_length_code >= 4) {
    int CommandedSecondaryAirStatus = (message.data[3]);
    Serial.printf("Commanded secondary air status: %d \n", CommandedSecondaryAirStatus);
    SaveDataType = message.data[2];
    SaveDataValue = CommandedSecondaryAirStatus * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x14 && message.data_length_code >= 4) {
    float OxygenSensor1Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 1 voltage: %.2f V \n", OxygenSensor1Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor1Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x15 && message.data_length_code >= 4) {
    float OxygenSensor2Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 2 voltage: %.2f V \n", OxygenSensor2Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor2Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x16 && message.data_length_code >= 4) {
    float OxygenSensor3Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 3 voltage: %.2f V \n", OxygenSensor3Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor3Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x17 && message.data_length_code >= 4) {
    float OxygenSensor4Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 4 voltage: %.2f V \n", OxygenSensor4Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor4Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x18 && message.data_length_code >= 4) {
    float OxygenSensor5Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 5 voltage: %.2f V \n", OxygenSensor5Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor5Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x19 && message.data_length_code >= 4) {
    float OxygenSensor6Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 6 voltage: %.2f V \n", OxygenSensor6Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor6Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x1A && message.data_length_code >= 4) {
    float OxygenSensor7Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 7 voltage: %.2f V \n", OxygenSensor7Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor7Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x1B && message.data_length_code >= 4) {
    float OxygenSensor8Voltage = ((message.data[3]) * 0.005);
    Serial.printf("Oxygen sensor 8 voltage: %.2f V \n", OxygenSensor8Voltage);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor8Voltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x1C && message.data_length_code >= 4) {
    int OBDstanddards = (message.data[3]);
    Serial.printf("OBD standards the vehicle conforms to: %d \n", OBDstanddards);
    SaveDataType = message.data[2];
    SaveDataValue = OBDstanddards * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x1F && message.data_length_code >= 4) {
    int RunTime = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Run time since engine start: %d sec \n", RunTime);
    SaveDataType = message.data[2];
    SaveDataValue = RunTime * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x21 && message.data_length_code >= 6) {
    int DistanceTraveled = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Distance traveled with MIL on: %d km \n", DistanceTraveled);
    SaveDataType = message.data[2];
    SaveDataValue = DistanceTraveled * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x22 && message.data_length_code >= 6) {
    float FuelRailPres = (((message.data[3] << 8) | message.data[4]) * 0.079);
    Serial.printf("Fuel rail pres.: %.2f kPa \n", FuelRailPres);
    SaveDataType = message.data[2];
    SaveDataValue = FuelRailPres * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x23 && message.data_length_code >= 6) {
    int FuelRailGaugePres = (((message.data[3] << 8) | message.data[4]) * 10);
    Serial.printf("Fuel rail gauge pres: %d kPa \n", FuelRailGaugePres);
    SaveDataType = message.data[2];
    SaveDataValue = FuelRailGaugePres * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x24 && message.data_length_code >= 6) {
    float OxygenSensor1AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 1 air fuel: %.2f ratio \n", OxygenSensor1AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor1AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x25 && message.data_length_code >= 6) {
    float OxygenSensor2AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 2 air fuel: %.2f ratio \n", OxygenSensor2AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor2AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x26 && message.data_length_code >= 6) {
    float OxygenSensor3AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 3 air fuel: %.2f ratio \n", OxygenSensor3AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor3AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x27 && message.data_length_code >= 6) {
    float OxygenSensor4AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 4 air fuel: %.2f ratio \n", OxygenSensor4AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor4AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x28 && message.data_length_code >= 6) {
    float OxygenSensor5AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 5 air fuel: %.2f ratio \n", OxygenSensor5AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor5AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x29 && message.data_length_code >= 6) {
    float OxygenSensor6AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 6 air fuel: %.2f ratio \n", OxygenSensor6AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor6AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x2A && message.data_length_code >= 6) {
    float OxygenSensor7AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 7 air fuel: %.2f ratio \n", OxygenSensor7AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor7AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x2B && message.data_length_code >= 6) {
    float OxygenSensor8AirFuel = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 8 air fuel: %.2f ratio \n", OxygenSensor8AirFuel);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor8AirFuel * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2C && message.data_length_code >= 4) {
    float CommandedEGR = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Commanded EGR: %.2f %% \n", CommandedEGR);
    SaveDataType = message.data[2];
    SaveDataValue = CommandedEGR * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2D && message.data_length_code >= 4) {
    float EGRerror = (-100 + ((message.data[3]) * (1 / 2.55)));
    Serial.printf("EGR Error: %.2f %% \n", EGRerror);
    SaveDataType = message.data[2];
    SaveDataValue = EGRerror * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2E && message.data_length_code >= 4) {
    float CommandedEvaporativePurge = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Commanded evaporative purge: %.2f %% \n", CommandedEvaporativePurge);
    SaveDataType = message.data[2];
    SaveDataValue = CommandedEvaporativePurge * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2F && message.data_length_code >= 4) {
    float FuelTankLevelInput = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Fuel tank level input: %.2f %% \n", FuelTankLevelInput);
    SaveDataType = message.data[2];
    SaveDataValue = FuelTankLevelInput * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x30 && message.data_length_code >= 4) {
    int WarmupsSinceDTCsCleared = (message.data[3]);
    Serial.printf("Warmups since DTCs cleared: %d count \n", WarmupsSinceDTCsCleared);
    SaveDataType = message.data[2];
    SaveDataValue = WarmupsSinceDTCsCleared * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x31 && message.data_length_code >= 6) {
    int DistanceTraveledSinceDTCsCleared = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Distance traveled since DTCs cleared: %d km \n", DistanceTraveledSinceDTCsCleared);
    SaveDataType = message.data[2];
    SaveDataValue = DistanceTraveledSinceDTCsCleared * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x32 && message.data_length_code >= 6) {
    float ESVP = (((message.data[3] << 8) | message.data[4]) * 0.25);
    Serial.printf("Evap. system vapor pressure: %.2f Pa \n", ESVP);
    SaveDataType = message.data[2];
    SaveDataValue = ESVP * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x33 && message.data_length_code >= 4) {
    int AbsoluteBarometricPressure = (message.data[3]);
    Serial.printf("Absolute barometric pressure: %d kPa \n", AbsoluteBarometricPressure);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteBarometricPressure * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x34 && message.data_length_code >= 6) {
    float OxygenSensor1AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 1 air fuel 2: %.2f ratio \n", OxygenSensor1AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor1AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x35 && message.data_length_code >= 6) {
    float OxygenSensor2AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 2 air fuel 2: %.2f ratio \n", OxygenSensor2AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor2AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x36 && message.data_length_code >= 6) {
    float OxygenSensor3AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 3 air fuel 2: %.2f ratio \n", OxygenSensor3AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor3AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x47 && message.data_length_code >= 6) {
    float OxygenSensor4AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 4 air fuel 2: %.2f ratio \n", OxygenSensor4AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor4AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x38 && message.data_length_code >= 6) {
    float OxygenSensor5AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 5 air fuel 2: %.2f ratio \n", OxygenSensor5AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor5AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x39 && message.data_length_code >= 6) {
    float OxygenSensor6AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 6 air fuel 2: %.2f ratio \n", OxygenSensor6AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor6AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3A && message.data_length_code >= 6) {
    float OxygenSensor7AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 7 air fuel 2: %.2f ratio \n", OxygenSensor7AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor7AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3B && message.data_length_code >= 6) {
    float OxygenSensor8AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Oxygen sensor 8 air fuel 2: %.2f ratio \n", OxygenSensor8AirFuel2);
    SaveDataType = message.data[2];
    SaveDataValue = OxygenSensor8AirFuel2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3C && message.data_length_code >= 6) {
    int CatalystTempBank1Sensor1 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
    Serial.printf("Catalyst temperature bank 1, sensor 1: %d degC \n", CatalystTempBank1Sensor1);
    SaveDataType = message.data[2];
    SaveDataValue = CatalystTempBank1Sensor1 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3D && message.data_length_code >= 6) {
    int CatalystTempBank2Sensor1 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
    Serial.printf("Catalyst temperature bank 2, sensor 1: %d degC \n", CatalystTempBank2Sensor1);
    SaveDataType = message.data[2];
    SaveDataValue = CatalystTempBank2Sensor1 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3E && message.data_length_code >= 6) {
    int CatalystTempBank1Sensor2 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
    Serial.printf("Catalyst temperature bank 1, sensor 1: %d degC \n", CatalystTempBank1Sensor2);
    SaveDataType = message.data[2];
    SaveDataValue = CatalystTempBank1Sensor2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3F && message.data_length_code >= 6) {
    int CatalystTempBank2Sensor2 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
    Serial.printf("Catalyst temperature bank 2, sensor 2: %d degC \n", CatalystTempBank2Sensor2);
    SaveDataType = message.data[2];
    SaveDataValue = CatalystTempBank2Sensor2 * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x41 && message.data_length_code >= 6) {
    unsigned long MonitorStatusThisDriveCycle = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);
    Serial.printf("Monitor status this drive cycle: %lu encoded \n", MonitorStatusThisDriveCycle);
    SaveDataType = message.data[2];
    SaveDataValue = MonitorStatusThisDriveCycle * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x42 && message.data_length_code >= 6) {
    float ControlModuleVoltage = (((message.data[3] << 8) | message.data[4]) * 0.001);
    Serial.printf("Control module voltage: %.2f V \n", ControlModuleVoltage);
    SaveDataType = message.data[2];
    SaveDataValue = ControlModuleVoltage * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x43 && message.data_length_code >= 6) {
    float AbsoluteLoadValue = (((message.data[3] << 8) | message.data[4]) * (1 / 2.55));
    Serial.printf("Absolute load value: %.2f %% \n", AbsoluteLoadValue);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteLoadValue * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x44 && message.data_length_code >= 6) {
    float CommanderAirFuelEquivRatio = (((message.data[3] << 8) | message.data[4]) * (1 / 32768));
    Serial.printf("Commander air/fuel equiv. ratio: %.2f ratio \n", CommanderAirFuelEquivRatio);
    SaveDataType = message.data[2];
    SaveDataValue = CommanderAirFuelEquivRatio * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x45 && message.data_length_code >= 4) {
    float RelativeThrottlePosition = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Relative throttle position: %.2f %% \n", RelativeThrottlePosition);
    SaveDataType = message.data[2];
    SaveDataValue = RelativeThrottlePosition * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x46 && message.data_length_code >= 4) {
    int AmbientAirTemperature = (-40 + (message.data[3]));
    Serial.printf("Ambient air temperature: %d degC \n", AmbientAirTemperature);
    SaveDataType = message.data[2];
    SaveDataValue = AmbientAirTemperature * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x47 && message.data_length_code >= 4) {
    float AbsoluteThrottlePositionB = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Absolute throttle position B: %.2f %% \n", AbsoluteThrottlePositionB);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteThrottlePositionB * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x48 && message.data_length_code >= 4) {
    float AbsoluteThrottlePositionC = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Absolute throttle position C: %.2f %% \n", AbsoluteThrottlePositionC);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteThrottlePositionC * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x49 && message.data_length_code >= 4) {
    float AbsoluteThrottlePositionD = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Absolute throttle position D: %.2f %% \n", AbsoluteThrottlePositionD);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteThrottlePositionD * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4A && message.data_length_code >= 4) {
    float AbsoluteThrottlePositionE = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Absolute throttle position E: %.2f %% \n", AbsoluteThrottlePositionE);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteThrottlePositionE * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4B && message.data_length_code >= 4) {
    float AbsoluteThrottlePositionF = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Absolute throttle position F: %.2f %% \n", AbsoluteThrottlePositionF);
    SaveDataType = message.data[2];
    SaveDataValue = AbsoluteThrottlePositionF * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4C && message.data_length_code >= 4) {
    float CommandedThrottleActuator = ((message.data[3]) * (1 / 2.55));
    Serial.printf("Commanded throttle actuator: %.2f %% \n", CommandedThrottleActuator);
    SaveDataType = message.data[2];
    SaveDataValue = CommandedThrottleActuator * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x4D && message.data_length_code >= 6) {
    int TimeRunWithMILon = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Time run with MIL on: %d min \n", TimeRunWithMILon);
    SaveDataType = message.data[2];
    SaveDataValue = TimeRunWithMILon * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x4E && message.data_length_code >= 6) {
    int TimeSinceDTCsCleared = ((message.data[3] << 8) | message.data[4]);
    Serial.printf("Time since DTCs cleared: %d min \n", TimeSinceDTCsCleared);
    SaveDataType = message.data[2];
    SaveDataValue = TimeSinceDTCsCleared * 100;
  }
  if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4F && message.data_length_code >= 4) {
    int MaxFuelAirEquivRatio = (message.data[3]);
    Serial.printf("Max fuel-air equiv. ratio: %d ratio \n", MaxFuelAirEquivRatio);
    SaveDataType = message.data[2];
    SaveDataValue = MaxFuelAirEquivRatio * 100;
  }
}





void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  if (packetIndex > 0 && (millis() - lastRequestTime > 150) || GGreset == 1) {
    lastRequestTime = millis();

    if (countBLE >= packetIndex) {
      countBLE = 0;
    }
    Serial.println(countBLE);
    Serial.println(packetIndex);
    receivedData = dataBuffer[countBLE];
    countBLE++;
    //delay(5);
    processReceivedData();
    Serial.println(receivedData);

    if (receivedData == "00") {  // If 'PIDs' is entered
      if (!pidExecuted) {        // Check if the command has already been executed

    Serial.println("I got 00");


    delay(100);

    uint8_t bits[4] = { 0 };  // Create an array to hold the bytes (4 bytes = 32 bits)

    // Set the first 3 bytes to 0x00 (8 bits each)
    // The last byte will contain only 7 bits, so we set it to 0x00 as well
    bits[0] = 0b11111111;  // 8 bits
    bits[1] = 0b11111111;  // 8 bits
    bits[2] = 0b11111111;  // 8 bits
    bits[3] = 0b11111110;   // 7 bits (to represent the 31st bit, the 8th bit is unused)

    // change last 0 to 1 for add more package and uncomment for function

    //for (int i = 0; i < 6; i++){
    // Send the bits
    pCharacteristic->setValue(bits, sizeof(bits));                    // Send the byte array
    pCharacteristic->notify();   

    delay(100);
    //}

    isRunning = false;    // Set the flag that the system has stopped
    countBLE = 0;
    receivedData = "";
    currentDataType = "";
    resetDataBuffer();
    receivedData = "";
    lastRequestTime = millis();
      } else {
        Serial.println("The PIDs command has already been executed. Wait for the next command.");
      }
    } else if (receivedData == "01") {
        isRunning = true;  // Set the flag that the system is running
        Serial.println("I got 01");
        int32_t randomNumber01 = random(0, 2);
        randomNumber01 = randomNumber01 * 100;
        String SaveDataTypeHex = "01";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber01);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "02") {  //
        isRunning = true;  // Set the flag that the system is running
        Serial.println("I got 02");
        int32_t randomNumber02 = random(0, 2);
        randomNumber02 = randomNumber02 * 100;
        String SaveDataTypeHex = "02";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber02);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "03") {  //
      //if (!pidExecuted) {               // Check if the command has already been executed
        //currentDataType = "Fuel system status";
        //sendQuery(currentDataType); // Send the query
        //lastRequestTime = millis(); // Save the execution time
        isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 03");
        int32_t randomNumber03 = random(0, 2);
        randomNumber03 = randomNumber03 * 100;
        String SaveDataTypeHex = "03";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber03);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "04") {  //
      //if (!pidExecuted) {               // Check if the command has already been executed
        //currentDataType = "Calculated engine load";
        //sendQuery(currentDataType); // Send the query
        //lastRequestTime = millis(); // Save the execution time
        isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 04");
        int32_t randomNumber04 = random(0, 101);
        randomNumber04 = randomNumber04 * 100;
        String SaveDataTypeHex = "04";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber04);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "05") {  // If 'Coolant' is entered
      //if (!pidExecuted) {               // Check if the command has already been executed
        //currentDataType = "Coolant";
        //sendQuery(currentDataType);  // Send the query for coolant data
        //lastRequestTime = millis();  // Save the execution time
        isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 05");
        int32_t randomNumber05 = random(-40, 216);
        randomNumber05 = randomNumber05 * 100;
        String SaveDataTypeHex = "05";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber05);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "06") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        //currentDataType = "STFT1";
        //sendQuery(currentDataType);  // Send the query
        //lastRequestTime = millis();  // Save the execution time
        isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 06");
        int32_t randomNumber06 = random(-100, 100);
        randomNumber06 = randomNumber06 * 100;
        String SaveDataTypeHex = "06";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber06);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "07") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        //currentDataType = "LTFT1";
        //sendQuery(currentDataType);  // Send the query
        //lastRequestTime = millis();  // Save the execution time
        isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 07");
        int32_t randomNumber07 = random(-100, 99);
        randomNumber07 = randomNumber07 * 100;
        String SaveDataTypeHex = "02";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber07);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "08") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        //currentDataType = "STFT2";
        //sendQuery(currentDataType);  // Send the query
        //lastRequestTime = millis();  // Save the execution time
        isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 08");
        int32_t randomNumber08 = random(-100, 100);
        randomNumber08 = randomNumber08 * 100;
        String SaveDataTypeHex = "08";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber08);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "09") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "LTFT2";
        //sendQuery(currentDataType);  // Send the query
        //lastRequestTime = millis();  // Save the execution time
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 09");
        int32_t randomNumber09 = random(-100, 100);
        randomNumber09 = randomNumber09 * 100;
        String SaveDataTypeHex = "09";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber09);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "0A") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Fuel Pressure";
        //sendQuery(currentDataType);  // Send the query
        //lastRequestTime = millis();  // Save the execution time
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 0A");
        int32_t randomNumber0A = random(0, 766);
        randomNumber0A = randomNumber0A * 100;
        String SaveDataTypeHex = "0A";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber0A);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "0B") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "IMAP";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 0B");
        int32_t randomNumber0B = random(0, 256);
        randomNumber0B = randomNumber0B * 100;
        String SaveDataTypeHex = "0B";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber0B);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "0C") {  // If 'RPM' is entered
      //if (!pidExecuted) {               // Check if the command has already been executed
        //currentDataType = "RPM";
        isRunning = true;  // Set the flag that the system is running
        Serial.println("I got 0C");
        int32_t randomNumber0C = random(800, 12001);
        Serial.print("RPM: " + String(randomNumber0C));
        randomNumber0C = randomNumber0C * 100;
        String SaveDataTypeHex = "0C";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber0C);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
        //}
      //}
    } else if (receivedData == "0D") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Speed";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 0D");
        int32_t randomNumber0D = random(0, 256);
        randomNumber0D = randomNumber0D * 100;
        String SaveDataTypeHex = "0D";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber0D);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "0E") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Timing advance";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 0E");
        int32_t randomNumber0E = random(-64, 65);
        randomNumber0E = randomNumber0E * 100;
        String SaveDataTypeHex = "0E";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber0E);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "0F") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "IAT";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 0F");
        int32_t randomNumber0F = random(-40, 216);
        randomNumber0F = randomNumber0F * 100;
        String SaveDataTypeHex = "0F";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber0F);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "10") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "MAFSAFR";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 10");
        int32_t randomNumber10 = random(0, 656);
        randomNumber10 = randomNumber10 * 100;
        String SaveDataTypeHex = "10";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber10);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "11") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Throttle position";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 11");
        int32_t randomNumber11 = random(0, 101);
        randomNumber11 = randomNumber11 * 100;
        String SaveDataTypeHex = "11";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber11);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "12") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CSAS";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 12");
        int32_t randomNumber12 = random(0, 2);
        randomNumber12 = randomNumber12 * 100;
        String SaveDataTypeHex = "12";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber12);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "14") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS1V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 14");
        int32_t randomNumber14 = random(0, 2);
        randomNumber14 = randomNumber14 * 100;
        String SaveDataTypeHex = "14";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber14);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "15") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS2V";
        //isRunning = true;  // Set the flag that the system is running




        Serial.println("I got 15");
        int32_t randomNumber15 = random(0, 2);
        randomNumber15 = randomNumber15 * 100;
        String SaveDataTypeHex = "15";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber15);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "16") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS3V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 16");
        int32_t randomNumber16 = random(0, 2);
        randomNumber16 = randomNumber16 * 100;
        String SaveDataTypeHex = "16";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber16);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "17") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS4V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 17");
        int32_t randomNumber17 = random(0, 2);
        randomNumber17 = randomNumber17 * 100;
        String SaveDataTypeHex = "17";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber17);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "18") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS5V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 18");
        int32_t randomNumber18 = random(0, 2);
        randomNumber18 = randomNumber18 * 100;
        String SaveDataTypeHex = "18";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber18);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "19") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS6V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 19");
        int32_t randomNumber19 = random(0, 2);
        randomNumber19 = randomNumber19 * 100;
        String SaveDataTypeHex = "19";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber19);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "1A") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS7V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 1A");
        int32_t randomNumber1A = random(0, 2);
        randomNumber1A = randomNumber1A * 100;
        String SaveDataTypeHex = "1A";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber1A);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "1B") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS8V";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 1B");
        int32_t randomNumber1B = random(0, 2);
        randomNumber1B = randomNumber1B * 100;
        String SaveDataTypeHex = "1B";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber1B);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "1C") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OBD standards";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 1C");
        int32_t randomNumber1C = random(0, 2);
        randomNumber1C = randomNumber1C * 100;
        String SaveDataTypeHex = "1C";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber1C);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "1F") {
      //if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Run time";
        //isRunning = true;  // Set the flag that the system is running



        Serial.println("I got 1F");
        int32_t randomNumber1F = random(0, 65536);
        randomNumber1F = randomNumber1F * 100;
        String SaveDataTypeHex = "1F";
        String SendDataBLE = String(SaveDataTypeHex) + String(randomNumber1F);
        SendDataBLE.trim();  // Trim any unnecessary spaces or new line characters
        Serial.println("Sending data via Bluetooth: " + SendDataBLE);
        pCharacteristic->setValue(SendDataBLE.c_str());
        pCharacteristic->notify();  // Send a notification to connected clients
        //delay(1000);                // delay between sending(for Adam:)
        lastRequestTime = millis();
      //}
    } else if (receivedData == "21") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Distance traveled";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "22") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Fuel rail pres";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "23") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Fuel rail gauge pres";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "24") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS1A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "25") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS2A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "26") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS3A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "27") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS4A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "28") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS5A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "29") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS6A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "2A") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS7A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "2B") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS8A";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "2C") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "Commanded EGR";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "2D") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "EGR Error";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "2E") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CEP";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "2F") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "FTLI";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "30") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "WSDTCsC";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "31") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "DTSDTCsC";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "32") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ESVP";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "33") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ABP";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "34") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS1A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "35") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS2A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "36") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS3A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "37") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS4A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "38") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS5A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "39") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS6A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "3A") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS7A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "3B") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "OS8A2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "3C") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CTb1s1";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "3D") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CTb2s1";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "3E") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CTb1s2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "3F") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CTb2s2";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "41") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "MSTDC";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "42") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CMV";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "43") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ALV";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "44") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CAFER";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "45") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "RTP";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "46") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "AAT";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "47") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ATPB";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "48") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ATPC";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "49") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ATPD";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "4A") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ATPE";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "4B") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "ATPF";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "4C") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "CTA";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "4D") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "TRWMon";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "4E") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "TSDTCsC";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "4F") {
      if (!pidExecuted) {  // Check if the command has already been executed
        currentDataType = "MFAER";
        isRunning = true;            // Set the flag that the system is running
      }
    } else if (receivedData == "GG" || GGreset == 1) {  // If 'STOP' is entered
      if (!pidExecuted) {
        isRunning = false;    // Set the flag that the system has stopped
        pidExecuted = false;  // Reset the command execution flag
        Serial.println("Data stream stopped. Use 'PIDs' or 'RPM' to start again.");
        resetDataBuffer();
        countBLE = 0;
        receivedData = "";
        currentDataType = "";
        GGreset = 0;
      } else {
        Serial.println("Unidentified command.");  // Message for an unknown command
      }
    }
  }
}

