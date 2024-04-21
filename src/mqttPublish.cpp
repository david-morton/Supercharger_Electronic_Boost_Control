#include "mqttPublish.h"
#include "Arduino.h"
#include "wifiHelpers.h"
#include <PubSubClient.h>
#include <map>

// Define toggle for connection state
bool mqttBrokerConnected = false;

IPAddress mqtt_server(192, 168, 10, 249); // Mosquitto MQTT broker address on laptop VM
const int mqtt_port = 1883;               // Mosquitto MQTT broker port on laptop VM (port forwarded)
PubSubClient mqttClient(wifiClient);      // Create MQTT client on WiFi

/* ======================================================================
   FUNCTION: Connect MQTT client to the broker
   ====================================================================== */
bool connectMqttClientToBroker() {
  if (!mqttClient.connected()) {
    Serial.println("INFO - Connecting to MQTT broker");
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setKeepAlive(5);
    if (mqttClient.connect("arduino-client")) {
      Serial.println("\tOK - MQTT Client connected");
      mqttBrokerConnected = true;
      return true;
    } else {
      Serial.println("\tFATAL - MQTT Client not connected");
      mqttBrokerConnected = false;
      return false;
    }
  }
}

/* ======================================================================
   FUNCTION: Publish multiple numeric metrics via MQTT
   ====================================================================== */
void publishMqttMetrics(String topic, std::map<String, double> metrics) {
  String payload = "{";
  for (auto const &[key, value] : metrics) {
    payload += "\"" + key + "\":" + String(value) + ",";
  }
  // Remove the trailing comma
  payload.remove(payload.length() - 1);
  payload += "}";
  mqttClient.publish(topic.c_str(), payload.c_str());
}
