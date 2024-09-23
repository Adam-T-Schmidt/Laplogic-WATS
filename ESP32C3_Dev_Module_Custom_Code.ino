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
  //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all messages

  // Set up a filter to accept only the Engine RPM PID (0x0C)
  twai_filter_config_t f_config;
  f_config.acceptance_code = 0x0C;   // Set the acceptance code to the PID
  f_config.acceptance_mask = 0x7FF;  // Mask to match standard CAN IDs (11 bits)
  //f_config.mode = TWAI_FILTER_MODE_ACCEPTANCE;  // Set filter mode to acceptance (caused mode error)

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


    // Check if the received message is for Engine RPM (PID 0x0C)
    if (message.identifier == 0x0C) {
      // Assuming the data format is as per the OBD2 specification
      // For Engine RPM, the data is typically 2 bytes:
      // RPM = ((data[0] * 256) + data[1]) / 4
      int rpm = message.data[0];

      // Only print the RPM if we haven't printed it recently
      if (!rpmPrinted) {
        Serial.print("Received message ID: ");
        Serial.println(message.identifier, HEX);
        Serial.print("Engine RPM: ");
        Serial.println(rpm);

        rpmPrinted = true;  // Mark that we've printed the RPM
      }
    } else {
      if (rpmPrinted) {
        Serial.println("Received message not for Engine RPM.");
        rpmPrinted = false;  // Reset the flag if we received a different message
      }
    }
  } else if (result == ESP_ERR_TIMEOUT) {
    // No new messages received in specified time
    // Reset the printed state if no messages are received
    rpmPrinted = false;
  } else {
    Serial.println("Error receiving message!");
    rpmPrinted = false;
  }

  delay(100);  // Small delay to prevent flooding the serial output
}
