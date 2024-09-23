/*
 * LapLogic - OBD2 Decoding 
 * The purpose of this project is to decode data coming from the OBD2 port. 
*/

#include <driver/twai.h>  //this drive is included in the board package downloaded for ESP32. Required for TWAI (Two-Wire Automotive Interface).

const gpio_num_t RX_PIN = GPIO_NUM_4;
const gpio_num_t TX_PIN = GPIO_NUM_5;

void setup() {

  //Initialize Serial
  Serial.begin(9600);  //May need to change baud rate

  // Configure TWAI for normal mode
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();    // Adjust this if needed
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all messages

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI driver installation failed!");
    return;
  }

  // Start TWAI
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed!");
    return;
  }

  Serial.println("TWAI initialized successfully.");
}

void loop() {
  sendRPMQuery();  //send out query for RPM

  twai_message_t receivedMessage;
  esp_err_t result = twai_receive(&receivedMessage, pdMS_TO_TICKS(500));  //wait for a response for 500ms

  if (result == ESP_OK) {
    Serial.println("Message has been received");
    handleReceivedMessage(receivedMessage);  // Handle the received message
  } else {
    Serial.println("No message received.");
  }
  delay(500);  // Wait for .5 seconds before sending the next query
}


void sendRPMQuery() {
  twai_message_t queryMessage;        //variable to store received OBD2 message
  queryMessage.identifier = 0x7DF;    //Standard OBD-II request identifier
  queryMessage.data[0] = 0x01;        //Service ID for "Show Current Data"
  queryMessage.data[1] = 0x0C;        //PID for ENGINE RPM
  queryMessage.data_length_code = 2;  //set the legnth of the data

  esp_err_t result = twai_transmit(&queryMessage, pdMS_TO_TICKS(100));  // Send the message, Verified

  if (result == ESP_OK) {
    Serial.println("Query for RPM sent successfully.");
  } else {
    Serial.println("Failed to send RPM query.");
  }
}

void handleReceivedMessage(twai_message_t message) {
  // Check if the message identifier is 0x7E8
  if (message.identifier >= 0x7E8 && message.identifier <= 0x7EF) {
    // Check if the response length is valid (between 3 and 6 additional bytes)
    Serial.print("Identifier is within range: ");
    Serial.println(message.identifier);

    if (message.data_length_code >= 1 && message.data_length_code <= 8) {
      Serial.print("Length is within range: ");
      Serial.println(message.data_length_code);
      
 
      // Print all bytes in message.data
      for (uint8_t i = 0; i < message.data_length_code; i++) {
        Serial.print("message.data[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.println(message.data[i]); // Print in hexadecimal format
      }

      // // Check if we received the RPM data
      // if (serviceMode == 0x41 && pid == 0x0C) {
      //   //Calculate RPM from the response data
      //   //Concatenate bytes 2 and 3 (RPM values) into a single integer
      //   int concatenatedValue = (message.data[3] << 8) | message.data[4];  // Combine the two bytes
      //   int rpm = concatenatedValue / 4;                                   // Calculate RPM
      //   Serial.print("Engine value: ");
      //   Serial.println(rpm);
      // } else {
      //   Serial.println("Received data is not for RPM.");
      // }
    } else {
      Serial.println("Received response has invalid data length.");
    }
  } else {
    Serial.println("Received message is not from the expected CAN ID.");
  }
}
