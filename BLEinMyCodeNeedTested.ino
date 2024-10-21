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

static bool driver_installed = false;
unsigned long lastRequestTime = 0;

static String currentDataType = "";  // Текущий тип запроса (DTCs, RPM, etc.)
static bool isRunning = false;       // Флаг для контроля состояния системы

static bool pidExecuted = false; // Flag for tracking the execution of PIDs

String lastReceivedData = "";

uint8_t SaveDataType;
uint16_t SaveDataValue;



String blePIDMessage = "";

int PIDsHEX = 0;
int CountPIDs = 0;
uint32_t binaryString;  

// Переменная для хранения данных, полученных через BLE
String receivedData = "";
String dataBuffer = "";

unsigned long lastDataTime = 0;  // Время последнего принятого символа
const unsigned long DATA_TIMEOUT = 200; 
// Колбэк для обработки полученных данных
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++) {
          char receivedChar = rxValue[i];
          dataBuffer += receivedChar;  // Добавляем символ в буфер
          lastDataTime = millis();  // Обновляем время последнего принятого символа
        }
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
  // Create a BLE service using the defined SERVICE_UUID.
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE characteristic for the service with read and write properties.
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);


  // Создаём характеристику
  //BLECharacteristic *pCharacteristic = pService->createCharacteristic(
  //                                       CHARACTERISTIC_UUID,
  //                                       BLECharacteristic::PROPERTY_READ |
  //                                       BLECharacteristic::PROPERTY_WRITE
  //                                     );

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
    } else {
        Serial.println("Unknown data type requested.");
    }
    queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
    queryMessage.extd = 0;            // Standard Frame
    queryMessage.rtr = 0;             // No request for remote frame
    queryMessage.data_length_code = 8;
    queryMessage.data[0] = 0x02;      // Number of additional data bytes
    queryMessage.data[1] = 0x01;      // Service ID for "Show Current Data"
    //queryMessage.data[2] = 0x05;      // PID for Engine RPM
    for (int i = 3; i < 8; i++) {
      queryMessage.data[i] = 0x00;    // Fill the rest of the bytes with 0
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



void handleReceivedMessage(twai_message_t& message) 
{
  //check if we got right package
if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x00 && message.data_length_code >= 6) 
  {
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
    //for (int i = 31; i >= 0; i--) {
    // Append each bit to the binaryString
    //binaryString += String((PIDs0120 >> i) & 1);
    //}
    //binaryString = String(binaryString);
    //pCharacteristic->setValue(binaryString, sizeof(binaryString));  // Convert String to C-style string
    pCharacteristic->setValue((uint8_t*)&binaryString, sizeof(binaryString));
    pCharacteristic->notify();
    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////\

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


//    PIDsHEX = 1;
//    CountPIDs = 31;
//    while (CountPIDs >= 0) {
//    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDs0120 & (1 << CountPIDs)) ? "yes" : "no");
//    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
//    pCharacteristic->notify();
//    PIDsHEX++;
//    CountPIDs--;
//    }

    // Print the received message data
    for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
    }
    if (PIDs0120 & (1 << 0)) {
    Serial.println("");  
    Serial.println("PIDs supported [21-40] are available, sending next query.");
    //sendSecondPIDsQuery();
    // Check for user input to send the PIDs query
      //if (Serial.available()) {
        //String command = Serial.readString();
        //command.trim();  // Remove any extra whitespaces or newlines
          //if (command.equalsIgnoreCase("PIDs")) {
            sendSecondPIDsQuery();
          //}
      //}
    }

  } else {
    //Serial.println("Received message is not an response");
  }


  //Serial.println("");
  


if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x20 && message.data_length_code >= 6) 
  {
    //add all bit data to one number
    uint32_t PIDs2140 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


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

    //for (int i = 31; i >= 0; i--) {
    // Append each bit to the binaryString
    //binaryString += String((PIDs2140 >> i) & 1);
    //}
    //binaryString = "Binary representation: " + String(binaryString);
    //pCharacteristic->setValue(binaryString.c_str());  // Convert String to C-style string
    //pCharacteristic->notify();

    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


    // Check bits:
    // ///////////////||||||||||||||||||||///////////////
    // ///////////////VVVVVVVVVVVVVVVVVVVV///////////////
    // //////////////////////////////////////////////////
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

    Serial.print("Oxygen sensor 8 (air-fuel equiv. ratio):" );
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

    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////

    PIDsHEX++;
    CountPIDs = 31;
    while (CountPIDs >= 0) {
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDs2140 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;
    }


  // Print the received message data
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }

  if (PIDs2140 & (1 << 0)) {
    Serial.println("");  
    Serial.println("PIDs supported [41-60] are available, sending next query.");
    //sendThirdPIDsQuery();
     //if (Serial.available()) {
        //String command = Serial.readString();
        //command.trim();  // Remove any extra whitespaces or newlines
        //  if (command.equalsIgnoreCase("PIDs")) {
            sendThirdPIDsQuery();
        //  }
      //}
  }

  } else {
    //Serial.println("Received message is not an response");
  }
    //Serial.println("");



if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x40 && message.data_length_code >= 6)
  {
    //add all bit data to one number
    uint32_t PIDs4160 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////
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

    //binaryString = "";
    //for (int i = 31; i >= 0; i--) {
    // Append each bit to the binaryString
    //binaryString += String((PIDs4160 >> i) & 1);
    //}
    //binaryString = "Binary representation: " + String(binaryString);
    //pCharacteristic->setValue(binaryString.c_str());  // Convert String to C-style string
    //pCharacteristic->notify();
    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


    // Check bits:
    // ///////////////||||||||||||||||||||///////////////
    // ///////////////VVVVVVVVVVVVVVVVVVVV///////////////
    // //////////////////////////////////////////////////
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

    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////

    PIDsHEX++;
    CountPIDs = 31;
    while (CountPIDs >= 0) {
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDs4160 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;
    }



  // Print the received message data
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }

  if (PIDs4160 & (1 << 0)) {
  Serial.println("");  
  Serial.println("PIDs supported [61-80] are available, sending next query.");
  //sendFourthPIDsQuery();
    //if (Serial.available()) {
      //String command = Serial.readString();
      //command.trim();  // Remove any extra whitespaces or newlines
      //  if (command.equalsIgnoreCase("PIDs")) {
            sendFourthPIDsQuery();
      //  }
    //}
  }

  } else {
    //Serial.println("Received message is not an response");
  }
    //Serial.println("");


if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x60 && message.data_length_code >= 6)
  {
    //add all bit data to one number
    uint32_t PIDs6180 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


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


    //binaryString = "";
    //for (int i = 31; i >= 0; i--) {
    // Append each bit to the binaryString
    //binaryString += String((PIDs6180 >> i) & 1);
    //}
    //binaryString = "Binary representation: " + String(binaryString);
    //pCharacteristic->setValue(binaryString.c_str());  // Convert String to C-style string
    //pCharacteristic->notify();

    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


    // Check bits:
    // ///////////////||||||||||||||||||||///////////////
    // ///////////////VVVVVVVVVVVVVVVVVVVV///////////////
    // //////////////////////////////////////////////////
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
    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////

    PIDsHEX++;
    CountPIDs = 31;
    while (CountPIDs >= 0) {
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDs6180 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;
    }



  // Print the received message data
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }

  if (PIDs6180 & (1 << 0)) {
  Serial.println("");  
  Serial.println("PIDs supported [61-80] are available, sending next query.");
  //sendFifthPIDsQuery();
    //if (Serial.available()) {
      //String command = Serial.readString();
      //command.trim();  // Remove any extra whitespaces or newlines
        //if (command.equalsIgnoreCase("PIDs")) {
            sendFifthPIDsQuery();
        //}
    //}  
  }

  } else {
    //Serial.println("Received message is not an response");
  }
    //Serial.println("");


if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x80 && message.data_length_code >= 6)
  {
    //add all bit data to one number
    uint32_t PIDs81A0 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


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


    //binaryString = "";
    //for (int i = 31; i >= 0; i--) {
    // Append each bit to the binaryString
    //binaryString += String((PIDs81A0 >> i) & 1);
    //}
    //binaryString = "Binary representation: " + String(binaryString);
    //pCharacteristic->setValue(binaryString.c_str());  // Convert String to C-style string
    //pCharacteristic->notify();

    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


    // Check bits:
    // ///////////////||||||||||||||||||||///////////////
    // ///////////////VVVVVVVVVVVVVVVVVVVV///////////////
    // //////////////////////////////////////////////////
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

    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////

    PIDsHEX++;
    CountPIDs = 31;
    while (CountPIDs >= 0) {
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDs81A0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;
    }



  // Print the received message data
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }

  if (PIDs81A0 & (1 << 0)) {
  Serial.println("");  
  Serial.println("PIDs supported [A1-C0] are available, sending next query.");
  //sendSixthPIDsQuery();
    //if (Serial.available()) {
      //String command = Serial.readString();
      //command.trim();  // Remove any extra whitespaces or newlines
        //if (command.equalsIgnoreCase("PIDs")) {
            sendSixthPIDsQuery();
        //}
    //}   
  }

  } else {
    //Serial.println("Received message is not an response");
  }
    //Serial.println("");



if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0xA0 && message.data_length_code >= 6)
  {
    //add all bit data to one number
    uint32_t PIDsA1C0 = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);


    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


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


    //binaryString = "";
    //for (int i = 31; i >= 0; i--) {
    // Append each bit to the binaryString
    //binaryString += String((PIDsA1C0 >> i) & 1);
    //}
    //binaryString = "Binary representation: " + String(binaryString);
    //pCharacteristic->setValue(binaryString.c_str());  // Convert String to C-style string
    //pCharacteristic->notify();

    // //////////////////////////////////////////////////
    // /////////////// ONLY FOR DEBUGGING ///////////////
    // //////////////////////////////////////////////////


    // Check bits:
    // ///////////////||||||||||||||||||||///////////////
    // ///////////////VVVVVVVVVVVVVVVVVVVV///////////////
    // //////////////////////////////////////////////////
    Serial.println("Options :");

    Serial.print("NOx sensor corrected data: ");
    Serial.println((PIDsA1C0 & (1 << 31)) ? "yes" : "no");  // Check first bit(bit 31)
    PIDsHEX = 161;
    CountPIDs = 31;
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("Cylinder fuel rate: ");
    Serial.println((PIDsA1C0 & (1 << 30)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("Evap system vapor pressure: ");
    Serial.println((PIDsA1C0 & (1 << 29)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("Transmission actual gear: ");
    Serial.println((PIDsA1C0 & (1 << 28)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("Cmd. diesel exhaust fluid dosing: ");
    Serial.println((PIDsA1C0 & (1 << 27)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("Odometer: ");
    Serial.println((PIDsA1C0 & (1 << 26)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("NOx concentration 3, 4: ");
    Serial.println((PIDsA1C0 & (1 << 25)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("NOx corrected concentration (3, 4): ");
    Serial.println((PIDsA1C0 & (1 << 24)) ? "yes" : "no");
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0 & (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();
    PIDsHEX++;
    CountPIDs--;

    Serial.print("PIDs supported [C1 - E0]: ");
    Serial.println((PIDsA1C0 & (1 << 0)) ? "yes" : "no");
    PIDsHEX = 192;
    CountPIDs = 0;
    blePIDMessage = String(PIDsHEX, HEX) + ": " + String((PIDsA1C0& (1 << CountPIDs)) ? "yes" : "no");
    pCharacteristic->setValue(blePIDMessage.c_str());  // Convert String to C-style string
    pCharacteristic->notify();

    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////
    // //////////////////////////////////////////////////

    PIDsHEX = 0;
    CountPIDs = 0;

  // Print the received message data
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }

  if (PIDsA1C0 & (1 << 0)) {
  Serial.println("PIDs supported [C1-E0] are available, but i dont know how decode that, bmw pls send me manual for car 0_0.");
  }

  } else {
    //Serial.println("Received message is not an response");
  }



if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x01 && message.data_length_code >= 6)
  {
  int DTCs = (message.data[3]);
  Serial.printf("Monitor status since DTCs cleared: %d \n", DTCs);
  SaveDataType = message.data[2];
  SaveDataValue = DTCs;
  }

if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x02 && message.data_length_code >= 6)
  {
  int FreezeDTC = ((message.data[3] << 8) | message.data[4]);
  Serial.printf("Freeze DTC: %d \n", FreezeDTC);
  SaveDataType = message.data[2];
  SaveDataValue = FreezeDTC;
  }

if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x03 && message.data_length_code >= 6)
  {
  int FuelSystemStatus = ((message.data[3] << 8) | message.data[4]);
  Serial.printf("Fuel system status: %d \n", FuelSystemStatus);
  SaveDataType = message.data[2];
  SaveDataValue = FuelSystemStatus;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x04 && message.data_length_code >= 4)
  {
  //int CalculatedEngineLoad = (int)(message.data[3] * (1.0 / 2.55));
  //Serial.printf("Calculated engine load: %d %% \n", CalculatedEngineLoad);

  float CalculatedEngineLoad = (message.data[3] * (1.0 / 2.55));
  Serial.printf("Calculated engine load: %.2f %% \n", CalculatedEngineLoad);
  SaveDataType = message.data[2];
  SaveDataValue = CalculatedEngineLoad;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x05 && message.data_length_code >= 4)
  {
  int Coolant = (-40 + (message.data[3]));
  Serial.printf("Coolant Temp: %d degC\n", Coolant);
  SaveDataType = message.data[2];
  SaveDataValue = Coolant;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x06 && message.data_length_code >= 4)
  {
  float STFT1 = (-100 + (message.data[3] * (1.0 / 1.28)));
  Serial.printf("Short term fuel trim(bank 1): %.2f %% \n", STFT1);
  SaveDataType = message.data[2];
  SaveDataValue = STFT1;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x07 && message.data_length_code >= 4)
  {
  float LTFT1 = (-100 + (message.data[3] * (1.0 / 1.28)));
  Serial.printf("Long term fuel trim(bank 1): %.2f %% \n", LTFT1);
  SaveDataType = message.data[2];
  SaveDataValue = LTFT1;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x08 && message.data_length_code >= 4)
  {
  float STFT2 = (-100 + (message.data[3] * (1.0 / 1.28)));
  Serial.printf("Short term fuel trim(bank 2): %.2f %% \n", STFT2);
  SaveDataType = message.data[2];
  SaveDataValue = STFT2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x09 && message.data_length_code >= 4)
  {
  float LTFT2 = (-100 + (message.data[3] * (1.0 / 1.28)));
  Serial.printf("Long term fuel trim(bank 2): %.2f %% \n", LTFT2);
  SaveDataType = message.data[2];
  SaveDataValue = LTFT2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0A && message.data_length_code >= 4)
  {
  int FuelPressure = ((message.data[3]) * 3);
  Serial.printf("Fuel Pressure: %d kPa \n", FuelPressure);
  SaveDataType = message.data[2];
  SaveDataValue = FuelPressure;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0B && message.data_length_code >= 4)
  {
  int IntakeManifoldAbsolutePressure = (message.data[3]);
  Serial.printf("Intake manifold absolute pressure: %d kPa \n", IntakeManifoldAbsolutePressure);
  SaveDataType = message.data[2];
  SaveDataValue = IntakeManifoldAbsolutePressure;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x0C && message.data_length_code >= 4)
  {
  int rpm = ((message.data[3] << 8) | message.data[4]) / 4;
  Serial.printf("Engine RPM: %d\n", rpm);
  SaveDataType = message.data[2];
  SaveDataValue = rpm;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0D && message.data_length_code >= 4)
  {
  int Speed = (message.data[3]);
  Serial.printf("Vehicle speed: %d km/h \n", Speed);
  SaveDataType = message.data[2];
  SaveDataValue = Speed;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0E && message.data_length_code >= 4)
  {
  int TimingAdvance = (-64 + (message.data[3] * 0.5));
  Serial.printf("Timing advance: %d deg \n", TimingAdvance);
  SaveDataType = message.data[2];
  SaveDataValue = TimingAdvance;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x0F && message.data_length_code >= 4)
  {
  int IntakeAirTemperature = (-40 + (message.data[3]));
  Serial.printf("Intake air temperature: %d degC \n", IntakeAirTemperature);
  SaveDataType = message.data[2];
  SaveDataValue = IntakeAirTemperature;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x10 && message.data_length_code >= 6)
  {
  float MassAirFlowSensorAirFlowRate = ((message.data[3] << 8) | message.data[4]) * 0.01;
  Serial.printf("Mass air flow sensor air flow rate: %.2f grams/sec \n", MassAirFlowSensorAirFlowRate);
  SaveDataType = message.data[2];
  SaveDataValue = MassAirFlowSensorAirFlowRate;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x11 && message.data_length_code >= 4)
  {
  float ThrottlePosition = ((message.data[3]) * (1.00/2.55));
  Serial.printf("Throttle position: %.2f %% \n", ThrottlePosition);
  SaveDataType = message.data[2];
  SaveDataValue = ThrottlePosition;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x12 && message.data_length_code >= 4)
  {
  int CommandedSecondaryAirStatus = (message.data[3]);
  Serial.printf("Commanded secondary air status: %d \n", CommandedSecondaryAirStatus);
  SaveDataType = message.data[2];
  SaveDataValue = CommandedSecondaryAirStatus;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x14 && message.data_length_code >= 4)
  {
  float OxygenSensor1Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 1 voltage: %.2f V \n", OxygenSensor1Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor1Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x15 && message.data_length_code >= 4)
  {
  float OxygenSensor2Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 2 voltage: %.2f V \n", OxygenSensor2Voltage);
    SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor2Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x16 && message.data_length_code >= 4)
  {
  float OxygenSensor3Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 3 voltage: %.2f V \n", OxygenSensor3Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor3Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x17 && message.data_length_code >= 4)
  {
  float OxygenSensor4Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 4 voltage: %.2f V \n", OxygenSensor4Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor4Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x18 && message.data_length_code >= 4)
  {
  float OxygenSensor5Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 5 voltage: %.2f V \n", OxygenSensor5Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor5Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x19 && message.data_length_code >= 4)
  {
  float OxygenSensor6Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 6 voltage: %.2f V \n", OxygenSensor6Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor6Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x1A && message.data_length_code >= 4)
  {
  float OxygenSensor7Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 7 voltage: %.2f V \n", OxygenSensor7Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor7Voltage;
  }  
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x1B && message.data_length_code >= 4)
  {
  float OxygenSensor8Voltage = ((message.data[3]) * 0.005);
  Serial.printf("Oxygen sensor 8 voltage: %.2f V \n", OxygenSensor8Voltage);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor8Voltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x1C && message.data_length_code >= 4)
  {
  int OBDstanddards = (message.data[3]);
  Serial.printf("OBD standards the vehicle conforms to: %d \n", OBDstanddards);
  SaveDataType = message.data[2];
  SaveDataValue = OBDstanddards;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x1F && message.data_length_code >= 4)
  {
  int RunTime = ((message.data[3] << 8) | message.data[4]);
  Serial.printf("Run time since engine start: %d sec \n", RunTime);
  SaveDataType = message.data[2];
  SaveDataValue = RunTime;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x21 && message.data_length_code >= 6)
  {
  int DistanceTraveled = ((message.data[3] << 8) | message.data[4]);
  Serial.printf("Distance traveled with MIL on: %d km \n", DistanceTraveled);
  SaveDataType = message.data[2];
  SaveDataValue = DistanceTraveled;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x22 && message.data_length_code >= 6)
  {
  float FuelRailPres = (((message.data[3] << 8) | message.data[4]) * 0.079);
  Serial.printf("Fuel rail pres.: %.2f kPa \n", FuelRailPres);
  SaveDataType = message.data[2];
  SaveDataValue = FuelRailPres;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x23 && message.data_length_code >= 6)
  {
  int FuelRailGaugePres = (((message.data[3] << 8) | message.data[4]) * 10);
  Serial.printf("Fuel rail gauge pres: %d kPa \n", FuelRailGaugePres);
  SaveDataType = message.data[2];
  SaveDataValue = FuelRailGaugePres;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x24 && message.data_length_code >= 6)
  {
  float OxygenSensor1AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 1 air fuel: %.2f ratio \n", OxygenSensor1AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor1AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x25 && message.data_length_code >= 6)
  {
  float OxygenSensor2AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 2 air fuel: %.2f ratio \n", OxygenSensor2AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor2AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x26 && message.data_length_code >= 6)
  {
  float OxygenSensor3AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 3 air fuel: %.2f ratio \n", OxygenSensor3AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor3AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x27 && message.data_length_code >= 6)
  {
  float OxygenSensor4AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 4 air fuel: %.2f ratio \n", OxygenSensor4AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor4AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x28 && message.data_length_code >= 6)
  {
  float OxygenSensor5AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 5 air fuel: %.2f ratio \n", OxygenSensor5AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor5AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x29 && message.data_length_code >= 6)
  {
  float OxygenSensor6AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 6 air fuel: %.2f ratio \n", OxygenSensor6AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor6AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x2A && message.data_length_code >= 6)
  {
  float OxygenSensor7AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 7 air fuel: %.2f ratio \n", OxygenSensor7AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor7AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x2B && message.data_length_code >= 6)
  {
  float OxygenSensor8AirFuel = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 8 air fuel: %.2f ratio \n", OxygenSensor8AirFuel);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor8AirFuel;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2C && message.data_length_code >= 4)
  {
  float CommandedEGR = ((message.data[3]) * (1/2.55));
  Serial.printf("Commanded EGR: %.2f %% \n", CommandedEGR);
  SaveDataType = message.data[2];
  SaveDataValue = CommandedEGR;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2D && message.data_length_code >= 4)
  {
  float EGRerror = (-100 + ((message.data[3]) * (1/2.55)));
  Serial.printf("EGR Error: %.2f %% \n", EGRerror);
  SaveDataType = message.data[2];
  SaveDataValue = EGRerror;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2E && message.data_length_code >= 4)
  {
  float CommandedEvaporativePurge = ((message.data[3]) * (1/2.55));
  Serial.printf("Commanded evaporative purge: %.2f %% \n", CommandedEvaporativePurge);
  SaveDataType = message.data[2];
  SaveDataValue = CommandedEvaporativePurge;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x2F && message.data_length_code >= 4)
  {
  float FuelTankLevelInput = ((message.data[3]) * (1/2.55));
  Serial.printf("Fuel tank level input: %.2f %% \n", FuelTankLevelInput);
  SaveDataType = message.data[2];
  SaveDataValue = FuelTankLevelInput;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x30 && message.data_length_code >= 4)
  {
  int WarmupsSinceDTCsCleared = (message.data[3]);
  Serial.printf("Warmups since DTCs cleared: %d count \n", WarmupsSinceDTCsCleared);
  SaveDataType = message.data[2];
  SaveDataValue = WarmupsSinceDTCsCleared;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x31 && message.data_length_code >= 6)
  {
  int DistanceTraveledSinceDTCsCleared = ((message.data[3] << 8) | message.data[4]);
  Serial.printf("Distance traveled since DTCs cleared: %d km \n", DistanceTraveledSinceDTCsCleared);
  SaveDataType = message.data[2];
  SaveDataValue = DistanceTraveledSinceDTCsCleared;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x32 && message.data_length_code >= 6)
  {
  float ESVP = (((message.data[3] << 8) | message.data[4]) * 0.25);
  Serial.printf("Evap. system vapor pressure: %.2f Pa \n", ESVP);
  SaveDataType = message.data[2];
  SaveDataValue = ESVP;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x33 && message.data_length_code >= 4)
  {
  int AbsoluteBarometricPressure = (message.data[3]);
  Serial.printf("Absolute barometric pressure: %d kPa \n", AbsoluteBarometricPressure);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteBarometricPressure;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x34 && message.data_length_code >= 6)
  {
  float OxygenSensor1AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 1 air fuel 2: %.2f ratio \n", OxygenSensor1AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor1AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x35 && message.data_length_code >= 6)
  {
  float OxygenSensor2AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 2 air fuel 2: %.2f ratio \n", OxygenSensor2AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor2AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x36 && message.data_length_code >= 6)
  {
  float OxygenSensor3AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 3 air fuel 2: %.2f ratio \n", OxygenSensor3AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor3AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x47 && message.data_length_code >= 6)
  {
  float OxygenSensor4AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 4 air fuel 2: %.2f ratio \n", OxygenSensor4AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor4AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x38 && message.data_length_code >= 6)
  {
  float OxygenSensor5AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 5 air fuel 2: %.2f ratio \n", OxygenSensor5AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor5AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x39 && message.data_length_code >= 6)
  {
  float OxygenSensor6AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 6 air fuel 2: %.2f ratio \n", OxygenSensor6AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor6AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3A && message.data_length_code >= 6)
  {
  float OxygenSensor7AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 7 air fuel 2: %.2f ratio \n", OxygenSensor7AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor7AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3B && message.data_length_code >= 6)
  {
  float OxygenSensor8AirFuel2 = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Oxygen sensor 8 air fuel 2: %.2f ratio \n", OxygenSensor8AirFuel2);
  SaveDataType = message.data[2];
  SaveDataValue = OxygenSensor8AirFuel2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3C && message.data_length_code >= 6)
  {
  int CatalystTempBank1Sensor1 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
  Serial.printf("Catalyst temperature bank 1, sensor 1: %d degC \n", CatalystTempBank1Sensor1);
  SaveDataType = message.data[2];
  SaveDataValue = CatalystTempBank1Sensor1;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3D && message.data_length_code >= 6)
  {
  int CatalystTempBank2Sensor1 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
  Serial.printf("Catalyst temperature bank 2, sensor 1: %d degC \n", CatalystTempBank2Sensor1);
  SaveDataType = message.data[2];
  SaveDataValue = CatalystTempBank2Sensor1;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3E && message.data_length_code >= 6)
  {
  int CatalystTempBank1Sensor2 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
  Serial.printf("Catalyst temperature bank 1, sensor 1: %d degC \n", CatalystTempBank1Sensor2);
  SaveDataType = message.data[2];
  SaveDataValue = CatalystTempBank1Sensor2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x3F && message.data_length_code >= 6)
  {
  int CatalystTempBank2Sensor2 = (-40 + (((message.data[3] << 8) | message.data[4]) * 0.1));
  Serial.printf("Catalyst temperature bank 2, sensor 2: %d degC \n", CatalystTempBank2Sensor2);
  SaveDataType = message.data[2];
  SaveDataValue = CatalystTempBank2Sensor2;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x06 && message.data[1] == 0x41 && message.data[2] == 0x41 && message.data_length_code >= 6)
  {
  unsigned long MonitorStatusThisDriveCycle = (message.data[3] << 24) | (message.data[4] << 16) | (message.data[5] << 8) | (message.data[6]);
  Serial.printf("Monitor status this drive cycle: %lu encoded \n", MonitorStatusThisDriveCycle);
  SaveDataType = message.data[2];
  SaveDataValue = MonitorStatusThisDriveCycle;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x42 && message.data_length_code >= 6)
  {
  float ControlModuleVoltage = (((message.data[3] << 8) | message.data[4]) * 0.001);
  Serial.printf("Control module voltage: %.2f V \n", ControlModuleVoltage);
  SaveDataType = message.data[2];
  SaveDataValue = ControlModuleVoltage;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x43 && message.data_length_code >= 6)
  {
  float AbsoluteLoadValue = (((message.data[3] << 8) | message.data[4]) * (1/2.55));
  Serial.printf("Absolute load value: %.2f %% \n", AbsoluteLoadValue);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteLoadValue;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x04 && message.data[1] == 0x41 && message.data[2] == 0x44 && message.data_length_code >= 6)
  {
  float CommanderAirFuelEquivRatio = (((message.data[3] << 8) | message.data[4]) * (1/32768));
  Serial.printf("Commander air/fuel equiv. ratio: %.2f ratio \n", CommanderAirFuelEquivRatio);
  SaveDataType = message.data[2];
  SaveDataValue = CommanderAirFuelEquivRatio;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x45 && message.data_length_code >= 4)
  {
  float RelativeThrottlePosition = ((message.data[3]) * (1/2.55));
  Serial.printf("Relative throttle position: %.2f %% \n", RelativeThrottlePosition);
  SaveDataType = message.data[2];
  SaveDataValue = RelativeThrottlePosition;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x46 && message.data_length_code >= 4)
  {
  int AmbientAirTemperature = (-40 + (message.data[3]));
  Serial.printf("Ambient air temperature: %d degC \n", AmbientAirTemperature);
  SaveDataType = message.data[2];
  SaveDataValue = AmbientAirTemperature;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x47 && message.data_length_code >= 4)
  {
  float AbsoluteThrottlePositionB = ((message.data[3]) * (1/2.55));
  Serial.printf("Absolute throttle position B: %.2f %% \n", AbsoluteThrottlePositionB);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteThrottlePositionB;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x48 && message.data_length_code >= 4)
  {
  float AbsoluteThrottlePositionC = ((message.data[3]) * (1/2.55));
  Serial.printf("Absolute throttle position C: %.2f %% \n", AbsoluteThrottlePositionC);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteThrottlePositionC;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x49 && message.data_length_code >= 4)
  {
  float AbsoluteThrottlePositionD = ((message.data[3]) * (1/2.55));
  Serial.printf("Absolute throttle position D: %.2f %% \n", AbsoluteThrottlePositionD);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteThrottlePositionD;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4A && message.data_length_code >= 4)
  {
  float AbsoluteThrottlePositionE = ((message.data[3]) * (1/2.55));
  Serial.printf("Absolute throttle position E: %.2f %% \n", AbsoluteThrottlePositionE);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteThrottlePositionE;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4B && message.data_length_code >= 4)
  {
  float AbsoluteThrottlePositionF = ((message.data[3]) * (1/2.55));
  Serial.printf("Absolute throttle position F: %.2f %% \n", AbsoluteThrottlePositionF);
  SaveDataType = message.data[2];
  SaveDataValue = AbsoluteThrottlePositionF;
  }
if (message.identifier == 0x7E8 && message.data[0] == 0x03 && message.data[1] == 0x41 && message.data[2] == 0x4C && message.data_length_code >= 4)
  {
  float CommandedThrottleActuator = ((message.data[3]) * (1/2.55));
  Serial.printf("Commanded throttle actuator: %.2f %% \n", CommandedThrottleActuator);
  SaveDataType = message.data[2];
  SaveDataValue = CommandedThrottleActuator;
  }
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }


if (dataBuffer.length() > 0 && (millis() - lastDataTime > DATA_TIMEOUT)) {
    // Save the complete data
    receivedData = dataBuffer;
    dataBuffer = "";  // Clear the buffer

    // Debug: Print the received data to the Serial Monitor
    Serial.println("Data received via BLE: " + receivedData);

    // After processing, you can clear the receivedData variable if needed
    // receivedData = "";
}

  // Check for serial input
  if (Serial.available() > 0 || receivedData.length() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("PIDs") || receivedData == "PIDs") { // If 'PIDs' is entered
      if (!pidExecuted) { // Check if the command has already been executed
        sendPIDsQuery(); // Execute the task
        pidExecuted = true; // Set the flag indicating the task has been executed
        lastRequestTime = millis(); // Save the execution time
        isRunning = true; // Set the flag that the system is running
      } else {
        Serial.println("The PIDs command has already been executed. Wait for the next command.");
      }

  } else if (input.equalsIgnoreCase("DTCs") || receivedData == "01") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "DTCs";
      sendQuery(currentDataType); // Send the query
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }

  } else if (input.equalsIgnoreCase("Freeze DTC") || receivedData == "02") { // 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Freeze DTC";
      sendQuery(currentDataType); // Send the query
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Fuel system status") || receivedData == "03") { // 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Fuel system status";
      sendQuery(currentDataType); // Send the query
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Calculated engine load") || receivedData == "04") { // 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Calculated engine load";
      sendQuery(currentDataType); // Send the query
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Coolant") || receivedData == "05") { // If 'Coolant' is entered
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Coolant";
      sendQuery(currentDataType); // Send the query for coolant data
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("STFT1") || receivedData == "06") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "STFT1";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("LTFT1") || receivedData == "07") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "LTFT1";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("STFT2") || receivedData == "08") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "STFT2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("LTFT2") || receivedData == "09") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "LTFT2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Fuel Pressure") || receivedData == "0A") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Fuel Pressure";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("IMAP") || receivedData == "0B") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "IMAP";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("RPM") || receivedData == "0C") { // If 'RPM' is entered
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "RPM";
      sendQuery(currentDataType); // Send the query for RPM data
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Speed") || receivedData == "0D") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Speed";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Timing advance") || receivedData == "0E") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Timing advance";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("IAT") || receivedData == "0F") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "IAT";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("MAFSAFR") || receivedData == "10") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "MAFSAFR";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Throttle position") || receivedData == "11") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Throttle position";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CSAS") || receivedData == "12") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CSAS";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS1V") || receivedData == "14") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS1V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS2V") || receivedData == "15") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS2V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS3V") || receivedData == "16") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS3V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS4V") || receivedData == "17") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS4V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS5V") || receivedData == "18") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS5V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS6V") || receivedData == "19") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS6V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS7V") || receivedData == "1A") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS7V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS8V") || receivedData == "1B") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS8V";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OBD standards") || receivedData == "1C") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OBD standards";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Run time") || receivedData == "1F") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Run time";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Distance traveled") || receivedData == "21") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Distance traveled";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Fuel rail pres") || receivedData == "22") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Fuel rail pres";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Fuel rail gauge pres") || receivedData == "23") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Fuel rail gauge pres";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS1A") || receivedData == "24") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS1A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS2A") || receivedData == "25") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS2A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS3A") || receivedData == "26") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS3A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS4A") || receivedData == "27") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS4A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS5A") || receivedData == "28") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS5A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS6A") || receivedData == "29") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS6A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS7A") || receivedData == "2A") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS7A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS8A") || receivedData == "2B") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS8A";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("Commanded EGR") || receivedData == "2C") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "Commanded EGR";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("EGR Error") || receivedData == "2D") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "EGR Error";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CEP") || receivedData == "2E") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CEP";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("FTLI") || receivedData == "2F") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "FTLI";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("WSDTCsC") || receivedData == "30") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "WSDTCsC";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("DTSDTCsC") || receivedData == "31") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "DTSDTCsC";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }    
  } else if (input.equalsIgnoreCase("ESVP") || receivedData == "32") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ESVP";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }   
  } else if (input.equalsIgnoreCase("ABP") || receivedData == "33") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ABP";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }  
  } else if (input.equalsIgnoreCase("OS1A2") || receivedData == "34") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS1A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS2A2") || receivedData == "35") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS2A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS3A2") || receivedData == "36") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS3A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS4A2") || receivedData == "37") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS4A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS5A2") || receivedData == "38") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS5A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS6A2") || receivedData == "39") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS6A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS7A2") || receivedData == "3A") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS7A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("OS8A2") || receivedData == "3B") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "OS8A2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CTb1s1") || receivedData == "3C") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CTb1s1";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CTb2s1") || receivedData == "3D") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CTb2s1";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CTb1s2") || receivedData == "3E") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CTb1s2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CTb2s2") || receivedData == "3F") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CTb2s2";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("MSTDC") || receivedData == "41") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "MSTDC";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CMV") || receivedData == "42") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CMV";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("ALV") || receivedData == "43") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ALV";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("CAFER") || receivedData == "44") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "CAFER";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("RTP") || receivedData == "45") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "RTP";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("AAT") || receivedData == "46") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "AAT";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("ATPB") || receivedData == "47") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ATPB";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("ATPC") || receivedData == "48") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ATPC";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("ATPD") || receivedData == "49") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ATPD";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("ATPE") || receivedData == "4A") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ATPE";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }
  } else if (input.equalsIgnoreCase("ATPF") || receivedData == "4B") { 
    if (!pidExecuted) { // Check if the command has already been executed
      currentDataType = "ATPF";
      sendQuery(currentDataType); // Send the query 
      lastRequestTime = millis(); // Save the execution time
      isRunning = true; // Set the flag that the system is running
    }

  } else if (input.equalsIgnoreCase("STOP") || receivedData == "stop") { // If 'STOP' is entered
    isRunning = false; // Set the flag that the system has stopped
    pidExecuted = false; // Reset the command execution flag
    Serial.println("Data stream stopped. Use 'PIDs' or 'RPM' to start again.");
  } else {
    Serial.println("Unidentified command."); // Message for an unknown command
  }
}

receivedData = "";


if (isRunning && pidExecuted == false && (millis() - lastRequestTime >= POLLING_RATE_MS)) {
    sendQuery(currentDataType);  // Send the query based on the current data type
    lastRequestTime = millis();  // Update the last request time to the current time
    
    // Form the string for SaveDataType in hex format
    String SaveDataTypeHex = String(SaveDataType, HEX);  // Convert SaveDataType to a string in hex format
    
    // Convert to uppercase
    SaveDataTypeHex.toUpperCase();

    // Check the length of the string and add a leading zero if needed
    if (SaveDataTypeHex.length() < 2) {
        SaveDataTypeHex = "0" + SaveDataTypeHex;
    }

    // Add "0x" at the beginning of the string for formatting
    SaveDataTypeHex = "0x" + SaveDataTypeHex;

    // Create the string with the formatted data type and value
    String receivedData = "Type: " + String(SaveDataTypeHex) + ", Value: " + String(SaveDataValue);  
    receivedData.trim();  // Trim any unnecessary spaces or new line characters

    // Debug: Print the data that was read
    Serial.println("Data from Serial: " + receivedData);

    // Save only the latest string
    lastReceivedData = receivedData; 

    // Debug: Check the data being sent via Bluetooth
    Serial.println("Sending data via Bluetooth: " + lastReceivedData);

    // Update the value for Bluetooth transmission
    pCharacteristic->setValue(lastReceivedData.c_str());

    // Notify connected clients via Bluetooth
    pCharacteristic->notify();  // Send a notification to connected clients

    Serial.println("Notification sent via Bluetooth");
}

  // Process received alerts and messages
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    
    
    while (twai_receive(&message, 0) == ESP_OK) {
      handleReceivedMessage(message);
    }
  }
  twai_clear_receive_queue();
}