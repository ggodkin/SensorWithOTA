#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
// Include WebServer.h before the WebServer object definition
#include <WebServer.h>
#include <ElegantOTA.h>
#include <esp_sleep.h>
#include "credentials.h" // Include the new credentials file
#include <esp_chip_info.h> // Include for chip information
#include <esp_flash.h>     // Include for flash information

// --- Includes for Sensor Task (when GPIO 23 is HIGH) ---
#include <esp_now.h>       // ESP-NOW library for ESP32
#include <OneWire.h>       // OneWire library for DS18B20
#include <DallasTemperature.h> // Dallas Temperature library for DS18B20

// --- Configuration ---
// GPIO pin to check for OTA activation.
// Connect this pin to GND at boot to activate OTA.
// Leave unconnected or connect to VCC for the Sensor Task and deep sleep.
const int OTAPin = 23;

// LED Pin (change to your board's built-in LED or desired GPIO)
// LED_BUILTIN is not always defined for all ESP32 boards in PlatformIO.
// GPIO 2 is a common pin for the built-in LED on many ESP32 development boards.
#define LED_PIN 2

// --- Pin Definitions for Sensor Task (MAP THESE TO YOUR ESP32 GPIOs) ---
// GPIO for powering the DS18B20 sensor
const int TEMP_POWER = 16; // Example: Map to an available ESP32 GPIO
// GPIO for grounding the ADC (if used for voltage measurement)
const int ADC_GND = 17;    // Example: Map to an available ESP32 GPIO
// GPIO where the DS18B20 data line is connected to
const int TEMP_DATA = 4;   // Example: Map to an available ESP32 GPIO
// Analog pin for battery voltage measurement (MAP TO ESP32 ADC GPIO)
const int ANALOG_PIN = 34; // Example: Map to an ESP32 ADC GPIO (e.g., GPIO34 is ADC1_CHANNEL6)

// Deep Sleep time in microseconds (adjust as needed)
// 120e6 = 120 * 1,000,000 = 120,000,000 microseconds = 2 minutes
const uint64_t sleepTime = 20e6; // Example: 2 minutes deep sleep


// --- Web Server and OTA Objects ---
// WebServer instance on port 80
WebServer server(80);

// --- Sensor Task Variables and Objects ---
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_DATA);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// REPLACE WITH RECEIVER MAC Address for ESP-NOW
uint8_t broadcastAddress[] = {0x2C, 0xF4, 0x32, 0x20, 0x5D, 0x1C}; // Example MAC Address
uint8_t tempDeviceAddress[8]; // For storing DS18B20 sensor address
String strMAC;

// DS18B20 count
int devCount = 0;

// Structure example to send data via ESP-NOW
// Must match the receiver structure
typedef struct struct_message {
  char a[240];
} struct_message;

// Create a struct_message called myData
struct_message myData;
String sendMsg;

// --- ESP-NOW Send Callback ---
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery success");
  } else {
    Serial.println("Delivery fail");
    digitalWrite(LED_PIN, HIGH); // Indicate failure with LED
  }
  Serial.print("On Data Sent Time (ms): ");
  Serial.println(millis());
}

// --- Helper function to convert DS18B20 address to string ---
String mac2string() {
  char str[3]; // Need 3 chars for two hex digits and null terminator
  String strMAC = "MAC";
  for (int i = 0; i < sizeof(tempDeviceAddress); i++) {
    sprintf(str, "%02X", tempDeviceAddress[i]); // Format as two uppercase hex digits
    strMAC.concat(':');
    strMAC.concat(str);
  }
  return strMAC;
}


// --- Setup Function ---
void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  Serial.println("\nBooting...");

  // Configure the OTA pin as an input with internal pull-up resistor.
  // This ensures the pin is HIGH by default if not connected to GND.
  pinMode(OTAPin, INPUT_PULLUP);

  // Give a short delay to allow the pin state to stabilize after boot
  delay(100);

  // Read the state of the OTA pin
  int pinState = digitalRead(OTAPin);

  // Get chip information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  // Get flash chip size
  uint32_t flash_size = 0;
  esp_flash_t* flash = esp_flash_default_chip;
  if (esp_flash_get_size(flash, &flash_size) != ESP_OK) {
      flash_size = 0; // Indicate unknown size on error
  }


  // Check the pin state to decide whether to activate OTA or run Sensor Task and deep sleep
  if (pinState == LOW) {
    // --- Activate OTA Mode ---
    Serial.println("GPIO 23 is LOW. Activating OTA mode.");

    // Print MCU specific information
    Serial.println("--- MCU Information ---");
    Serial.printf("Chip Model: %s\n", chip_info.model == CHIP_ESP32 ? "ESP32" : "Unknown");
    Serial.printf("Chip Revision: %u\n", chip_info.revision);
    Serial.printf("Number of Cores: %u\n", chip_info.cores);
    Serial.printf("Flash Chip Size: %uMB\n", flash_size / (1024 * 1024));
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("-----------------------");


    // Print WiFi credentials for debugging (REMOVE OR COMMENT OUT IN PRODUCTION)
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // WARNING: Printing password to serial is a security risk.
    // Only do this for debugging purposes and remove in production code.
    Serial.print("Using Password: ");
    Serial.println(password);


    // Connect to WiFi network using credentials from credentials.h
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password); // ssid and password are now defined in credentials.h
    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Setup mDNS responder. Allows accessing the device using a hostname like esp32-ota.local
    if (!MDNS.begin("esp32-ota")) { // You can change "esp32-ota" to your desired hostname
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.println("mDNS responder started");
    }

    // Start ElegantOTA server
    ElegantOTA.begin(&server); // Start ElegantOTA
    server.begin(); // Start the WebServer
    Serial.println("HTTP server started. ElegantOTA enabled.");
    Serial.println("Access OTA update page at:");
    Serial.println("http://" + WiFi.localIP().toString() + "/update");
    Serial.println("or http://esp32-ota.local/update");

  } else {
    // --- Run Sensor Task and Enter Deep Sleep ---
    Serial.println("GPIO 23 is HIGH. Running Sensor Task and preparing for deep sleep.");

    // Print MCU specific information
    Serial.println("--- MCU Information ---");
    Serial.printf("Chip Model: %s\n", chip_info.model == CHIP_ESP32 ? "ESP32" : "Unknown");
    Serial.printf("Chip Revision: %u\n", chip_info.revision);
    Serial.printf("Number of Cores: %u\n", chip_info.cores);
    Serial.printf("Flash Chip Size: %uMB\n", flash_size / (1024 * 1024));
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("-----------------------");

    // Initialize pins for sensor task
    pinMode(LED_PIN, OUTPUT);
    pinMode(TEMP_POWER, OUTPUT);
    pinMode(ADC_GND, OUTPUT);

    // Power up DS18B20
    digitalWrite(TEMP_POWER, HIGH);
    delay(100); // Give sensor time to power up

    // Start the DS18B20 sensor
    sensors.begin();
    devCount = sensors.getDeviceCount();
    Serial.print("DS18B20 Sensors found: ");
    Serial.println(devCount);
    Serial.println("setup completed for Sensor Task");

    // --- Sensor Reading and ESP-NOW Sending Logic (from original loop) ---
    float temperatureC;
    float temperatureF;
    float batVolt;
    long ellapsedTime = millis();
    digitalWrite(ADC_GND, LOW); // Set ADC_GND low for analog read

    Serial.print("Loop Enter Time (ms): ");
    Serial.println(ellapsedTime);

    sensors.setResolution(12); // Set resolution for DS18B20

    // Set values to send - Start with BOARD identifier
    strcpy(myData.a, "ESP32"); // Use "ESP32" as the board identifier
    char result[8];

    // Request and read temperatures from DS18B20 sensors
    sensors.requestTemperatures();
    for (int i = 0; i < devCount; i++) {
      temperatureC = sensors.getTempCByIndex(i);
      temperatureF = temperatureC * 1.8 + 32;
      sensors.getAddress(tempDeviceAddress, i);
      strMAC = mac2string(); // Get DS18B20 sensor address string
      Serial.print("Sensor Address: ");
      Serial.println(strMAC);

      // Append sensor data to the message string
      strcat(myData.a, "|");
      strcat(myData.a, strMAC.c_str());
      strcat(myData.a, "|TempC|");
      dtostrf(temperatureC, 6, 2, result); // Convert float to string
      strcat(myData.a, result);
      strcat(myData.a, "|TempF|");
      dtostrf(temperatureF, 6, 2, result);
      strcat(myData.a, result);

      Serial.print("DS18B20 Temp: ");
      Serial.print(temperatureC);
      Serial.print("*C\t and ");
      Serial.print(temperatureF);
      Serial.println("*F.");
    }
    ellapsedTime = millis();
    Serial.print("Temp collected Time (ms): ");
    Serial.println(ellapsedTime);

    // Power off DS18B20 after reading
    digitalWrite(TEMP_POWER, LOW);
    digitalWrite(LED_PIN, LOW); // Turn off LED

    // Read battery voltage
    batVolt = analogRead(ANALOG_PIN);
    digitalWrite(ADC_GND, HIGH); // Set ADC_GND high after analog read
    // Assuming a voltage divider if needed - adjust calculation based on your hardware
    // Example: if 10k/10k voltage divider, max 3.3V input maps to 1.65V at ADC pin
    // ESP32 ADC resolution is 12-bit (0-4095) by default
    // Adjust the calculation based on your voltage divider and ADC reference voltage
    batVolt = batVolt * (3.3 / 4095.0); // Basic conversion assuming 3.3V ADC reference
    Serial.print("Battery Voltage (ADC Raw): ");
    Serial.println(analogRead(ANALOG_PIN)); // Print raw ADC value for debugging
    Serial.print("Battery Voltage (Calculated): ");
    Serial.println(batVolt);

    // Append battery voltage data to the message string
    strcat(myData.a, "|BatV|");
    dtostrf(batVolt, 6, 2, result);
    strcat(myData.a, result);
    ellapsedTime = millis();
    Serial.print("Battery Voltage collected Time (ms): ");
    Serial.println(ellapsedTime);
    Serial.print("Full Message: ");
    Serial.println(myData.a);

    // --- ESP-NOW Initialization and Sending ---
    WiFi.mode(WIFI_STA); // Set WiFi to Station mode
    // Explicitly begin WiFi to ensure interface is ready for ESP-NOW
    // Even without connecting to an AP, this helps initialize the STA interface.
    WiFi.begin("", ""); // Use empty SSID and password to just bring up the interface
    Serial.print("WiFi.status() after WiFi.begin: ");
    Serial.println(WiFi.status());


    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) { // Check return code for ESP32
      Serial.println("Error initializing ESP-NOW");
      // Consider adding error handling or retry logic here
    } else {
      Serial.println("WiFi and ESP-NOW initialized");
      Serial.print(" Time (ms): ");
      Serial.println(millis());

      // Register the send callback function
      if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
          Serial.println("Error registering ESP-NOW send callback");
          // Consider error handling
      }

      // Register peer
      esp_now_peer_info_t peerInfo;
      memcpy(peerInfo.peer_addr, broadcastAddress, 6);
      peerInfo.channel = 0; // Use default channel 0
      peerInfo.encrypt = false; // No encryption

      // Add peer
      esp_err_t add_peer_result = esp_now_add_peer(&peerInfo);
      if (add_peer_result != ESP_OK) {
        Serial.print("Failed to add peer. Error: ");
        Serial.println(add_peer_result); // Print the specific error code
        // Consider error handling
      } else {
        Serial.println("Peer added successfully");
      }

      // Send message via ESP-NOW
      // esp_now_send returns ESP_OK on success, not a status byte like ESP8266
      esp_err_t result_send = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

      if (result_send == ESP_OK) {
          Serial.println("ESP-NOW send request successful");
      } else {
          Serial.print("Error sending ESP-NOW message: ");
          Serial.println(result_send); // Print the error code
          digitalWrite(LED_PIN, HIGH); // Indicate failure
      }

      delay(50); // Short delay after sending

      ellapsedTime = millis();
      Serial.print("Loop Exit Time (ms): ");
      Serial.println(ellapsedTime);

      // --- Enter Deep Sleep ---
      Serial.printf("Entering deep sleep for %llu microseconds...\n", sleepTime);
      esp_sleep_enable_timer_wakeup(sleepTime); // Configure timer wake-up
      esp_deep_sleep_start(); // Enter deep sleep
      // The code execution stops here until wake-up.
    }
  }
}

// --- Loop Function ---
void loop() {
  // This loop only runs if the device is in OTA mode (GPIO 23 was LOW at boot).
  // If the device ran the Sensor Task (GPIO 23 was HIGH), this loop will NOT run
  // because the device enters deep sleep at the end of setup().

  // Handle incoming client requests for the web server (including ElegantOTA)
  server.handleClient();
  // ElegantOTA loop to handle update process
  ElegantOTA.loop();

  // Add any other tasks you need to perform continuously while the device is awake
  // in OTA mode.
}
