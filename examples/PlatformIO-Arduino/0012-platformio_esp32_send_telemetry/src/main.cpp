#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

// Use virtual random sensor by default

// Uncomment if using DHT20
/*
#include "DHT20.h"
#include "Wire.h"
DHT20 dht20;
*/

// Uncomment if using DHT11
/*
#include <SimpleDHT.h>
#define DHT_PIN D3
SimpleDHT11 dht(DHT_PIN);
*/

// Uncomment if using DHT22
/*
#include <SimpleDHT.h>
#define DHT_PIN D3
SimpleDHT22 dht(DHT_PIN);
*/

constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

// Your device access token
constexpr char TOKEN[] = "YOUR_ACCESS_TOKEN";

// Server and port we want to establish a connection to
constexpr char COREIOT_SERVER[] = "app.coreiot.io";
constexpr uint16_t COREIOT_PORT = 1883U;

// Maximum packet size to be sent or received by the underlying MQTT client
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 128U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 128U;

// Serial debug baud rate
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// telemetry settings
constexpr int16_t TELEMETRY_SEND_INTERVAL = 5000U;
uint32_t previousTelemetrySend;

constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "humidity";

// Initalize the Mqtt client instance using WiFi
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

// Initialize used apis
const std::array<IAPI_Implementation*, 0U> apis = {};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

/// @brief Initalizes WiFi connection,
// wait until a connection established
void InitWiFi() {
  Serial.println("Connecting to WiFi AP ...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until connected
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi AP");
}

/// @brief Reconnects if WiFi disconnected
/// @return Returns true when connected again
const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If not connected, trying to connect to the given WiFi network
  InitWiFi();
  return true;
}

void setup() {

  // Uncomment if using DHT20
  //Wire.begin(SDA, SCL);
  //dht20.begin();

  // Initialize serial output for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
}

void loop() {
  delay(10);

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    // Connect to the server
    Serial.print("Connecting to: ");
    Serial.print(COREIOT_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(COREIOT_SERVER, TOKEN, COREIOT_PORT)) {
      Serial.println("Failed to connect");
      return;
    } else {
      Serial.println("Connected to server");
    }
    // Sending a MAC and IP address as an attribute
    tb.sendAttributeData("mac_address", WiFi.macAddress().c_str());
    tb.sendAttributeData("ip_address", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());    
    tb.sendAttributeData("channel", WiFi.channel());
  }

  // Sending telemetry every 5s
  if (millis() - previousTelemetrySend > TELEMETRY_SEND_INTERVAL) {

    // Use random value as virtual sensor
    float temperature = random(20, 40);
    float humidity = random(50, 100);
    
    // Uncomment if using DHT20
    /*    
    dht20.read();    
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();
    */

    // Uncomment if using DHT11/22
    /*    
    float temperature = 0;
    float humidity = 0;
    dht.read2(&temperature, &humidity, NULL);
    */

    Serial.println("Sending telemetry. Temperature: " + String(temperature, 1) + " humidity: " + String(humidity, 1));

    tb.sendTelemetryData(TEMPERATURE_KEY, temperature);
    tb.sendTelemetryData(HUMIDITY_KEY, humidity);
    tb.sendAttributeData("rssi", WiFi.RSSI()); // also update wifi signal strength
    previousTelemetrySend = millis();
  }

  tb.loop();
}
