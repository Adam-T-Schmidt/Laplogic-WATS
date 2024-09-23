/*
 * LapLogic - OBD2 Decoding 
 * The purpose of this project is to decode data coming from the OBD2 port. 
*/

#include <driver/twai.h>  //this drive is included in the board package downloaded for ESP32. Required for TWAI (Two-Wire Automotive Interface).

#define RX_PIN (gpio_num_t)4
#define TX_PIN (gpio_num_t)5

bool rpmPrinted = false;

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
  esp_err_t result = twai_receive(&receivedMessage, pdMS_TO_TICKS(100));  //wait for a response

  if (result == ESP_OK) {
    handleReceivedMessage(receivedMessage);  // Handle the received message
    Serial.println(receivedMessage); 
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
  queryMessage.sending = true;        //mark as sending message

  esp_err_t result = twai_transmit(&queryMessage, pdMS_TO_TICKS(100));  // Send the message
  if (result == ESP_OK) {
    Serial.println("Query for RPM sent successfully.");
  } else {
    Serial.println("Failed to send RPM query.");
  }
}
