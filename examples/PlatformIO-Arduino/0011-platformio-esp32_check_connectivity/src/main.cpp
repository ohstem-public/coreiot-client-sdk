#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

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

  tb.loop();
}
