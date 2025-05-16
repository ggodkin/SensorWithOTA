#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <esp_sleep.h>
#include "credentials.h" // Include the new credentials file
#include <esp_chip_info.h> // Include for chip information
#include <esp_flash.h>     // Include for flash information

// --- Configuration ---
// GPIO pin to check for OTA activation.
// Connect this pin to GND at boot to activate OTA.
// Leave unconnected or connect to VCC for deep sleep.
const int OTAPin = 23;

// --- Web Server and OTA Objects ---
// WebServer instance on port 80
WebServer server(80);

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


  // Check the pin state to decide whether to activate OTA or go to deep sleep
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
    // --- Enter Deep Sleep Mode ---
    Serial.println("GPIO 23 is HIGH. Entering deep sleep.");

    // Configure deep sleep.
    // In this simple example, no wake-up source is configured.
    // The device will sleep indefinitely until reset (e.g., power cycle, reset button).
    // If you need a timer wake-up (e.g., wake up after 60 seconds), uncomment and use the line below:
    // esp_sleep_enable_timer_wakeup(60 * 1000000); // Time in microseconds

    Serial.println("Going to deep sleep now...");
    // Enter deep sleep mode
    esp_deep_sleep_start();

    // The code execution stops here when deep sleep starts.
    // The lines below this point will not be reached until the next boot.
  }
}

// --- Loop Function ---
void loop() {
  // This loop only runs if the device is in OTA mode (GPIO 23 was LOW at boot).
  // If the device entered deep sleep, this function is not called.

  // Handle incoming client requests for the web server (including ElegantOTA)
  server.handleClient();
  // ElegantOTA loop to handle update process
  ElegantOTA.loop();

  // Add any other tasks you need to perform while in OTA mode here.
  // Keep this loop non-blocking if possible.
}
