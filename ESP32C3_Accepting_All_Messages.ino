/*
 * LapLogic - OBD2 Decoding 
 * The purpose of this project is to decode data coming from the OBD2 port. 
*/

#include <driver/twai.h>  //this drive is included in the board package downloaded for ESP32. Required for TWAI (Two-Wire Automotive Interface).

#define RX_PIN (gpio_num_t)4
#define TX_PIN (gpio_num_t)5

bool rpmPrinted = false;

  void
  setup() {

  //Initialize Serial
  Serial.begin(9600);  //May need to change baud rate

  // Configure TWAI for listen-only mode
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // Adjust this if needed
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
  twai_message_t message;                                         //variable to store received OBD2 message
  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(100));  //wait for a message up to 100ms

  if (result == ESP_OK) {

        Serial.print("Received message ID: ");
        Serial.println(message.identifier, HEX);

  delay(1000);  // Small delay to prevent flooding the serial output
}
}
