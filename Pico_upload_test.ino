#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <vector>
#include <string>
#include <ArduinoJson.h>

// Configuration
#define WIFI_SSID "x"
#define WIFI_PASSWORD "y"
#define SERVER_IP "z"
#define SERVER_PORT "v"

WiFiClient client;

// Base64 characters
static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Function to generate a test image
std::vector<uint8_t> create_test_image() {
    const int width = 32, height = 24;
    std::vector<uint8_t> test_image(width * height * 3);

    for (int i = 0; i < width * height; i++) {
        int row = i / width;
        int col = i % width;
        test_image[i * 3] = col % 255;
        test_image[i * 3 + 1] = row % 255;
        test_image[i * 3 + 2] = (col + row) % 255;
    }

    return test_image;
}

// Base64 encoding implementation
std::string base64_encode(const uint8_t* data, size_t size) {
    std::string encoded;
    encoded.reserve(((size + 2) / 3) * 4);

    for (size_t i = 0; i < size; i += 3) {
        uint32_t octet_a = i < size ? data[i] : 0;
        uint32_t octet_b = i + 1 < size ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < size ? data[i + 2] : 0;

        uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

        encoded.push_back(base64_chars[(triple >> 18) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 12) & 0x3F]);
        encoded.push_back(i + 1 < size ? base64_chars[(triple >> 6) & 0x3F] : '=');
        encoded.push_back(i + 2 < size ? base64_chars[triple & 0x3F] : '=');
    }

    return encoded;
}

void setup() {
    Serial.begin(9600);

    delay(3000);
    Serial.println("Simplified Pico Image Upload Client (Arduino) using HTTP");

    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Generate and upload a single test image
    Serial.println("Generating test image...");
    std::vector<uint8_t> image_data = create_test_image();
    Serial.printf("Test image created, size: %d bytes\n", image_data.size());

    // Base64 encode the image
    std::string encoded_image = base64_encode(image_data.data(), image_data.size());
    Serial.printf("Encoded image size: %d bytes\n", encoded_image.length());

    // Create JSON payload
    JsonDocument json_doc;
    json_doc["image"] = encoded_image.c_str();
    json_doc["timestamp"] = String(millis() / 1000); // Add current time as timestamp

    String json_string;
    serializeJson(json_doc, json_string);

    Serial.printf("Sending JSON payload, size: %d bytes\n", json_string.length());

    // Send HTTP POST request
    HTTPClient http;
    http.begin(client, "http://" + String(SERVER_IP) + ":" + String(SERVER_PORT));
    http.addHeader("Content-Type", "application/json");

  int response_code = http.POST(json_string);
  if (response_code > 0) {
      String response = http.getString();
      Serial.print("Response code: ");
      Serial.println(response_code);
      Serial.print("Response: ");
      Serial.println(response);
  } else {
      Serial.print("Error code: ");
      Serial.println(response_code);
      Serial.print("Error detail: ");
      Serial.println(http.errorToString(response_code));
  }

    http.end();
}

void loop() {
    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 20) {
        delay(500);
        Serial.print(".");
        wifi_timeout++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect to WiFi");
        return;
    }
    Serial.println("\nWiFi reconnected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    delay(5000);
}
