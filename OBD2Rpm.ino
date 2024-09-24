#include "driver/twai.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Polling interval (ms)
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
unsigned long lastRequestTime = 0;

void setup() {
  // Start Serial:
  Serial.begin(500000);

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

void sendRPMQuery() {
  // Create a CAN message for the OBD-II request
  twai_message_t queryMessage;
  queryMessage.identifier = 0x7DF;  // Standard OBD-II request identifier
  queryMessage.extd = 0;            // Standard Frame
  queryMessage.rtr = 0;             // No request for remote frame
  queryMessage.data_length_code = 8;
  queryMessage.data[0] = 0x02;      // Number of additional data bytes
  queryMessage.data[1] = 0x01;      // Service ID for "Show Current Data"
  queryMessage.data[2] = 0x0C;      // PID for Engine RPM
  for (int i = 3; i < 8; i++) {
    queryMessage.data[i] = 0x00;    // Fill the rest of the bytes with 0
  }

  // Send the RPM request
  if (twai_transmit(&queryMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("RPM query sent successfully");
  } else {
    Serial.println("Failed to send RPM query");
  }
}

void handleReceivedMessage(twai_message_t& message) {

if(message.identifier == 0x7E8)

{

  if (message.extd) {
    Serial.println("Message is in Extended Format");
  } else {
    Serial.println("Message is in Standard Format");
  }
  Serial.printf("ID: %x\nByte:", message.identifier);

  // Check if the message is a response to the RPM request (typically with ID 0x7E8)
  //if (message.identifier == 0x7E8 && message.data_length_code >= 4 && message.data[0] == 0x41 && message.data[1] == 0x0C) {
    if (message.identifier == 0x7E8) {
    // Extract RPM data
    int rpm = ((message.data[3] << 8) | message.data[4]) / 4;
    Serial.printf("Engine RPM: %d\n", rpm);
  } else {
    Serial.println("Received message is not an RPM response");
  }

  // Print the received message data
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }
  Serial.println("");

}

}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  // Send the RPM query every POLLING_RATE_MS
  if (millis() - lastRequestTime > POLLING_RATE_MS) {
    sendRPMQuery();
    lastRequestTime = millis();
  }

  // Check for received alerts and messages
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handleReceivedMessage(message);
    }
  }
}