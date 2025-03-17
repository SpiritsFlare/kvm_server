// Include necessary libraries
#include <WiFi.h>       // Library for WiFi connectivity
#include <SPI.h>        // Library for SPI communication
#include <SD.h>         // Library for SD card interfacing
#include "Arducam_Mega.h" // Library for Arducam Mega camera
#include "config.h"    // Configuration file for WiFi credentials
#include "Base64.h"    // Library for Base64 encoding

// Define Chip Select (CS) pins for camera and SD card
#define CAM_CS_PIN 8  // Chip select pin for Arducam Mega
#define SD_CS_PIN 10  // Chip select pin for SD card

// Initialize Arducam Mega camera with defined CS pin
Arducam_Mega myCamera(CAM_CS_PIN);
WiFiClient client; // WiFi client for server communication

// Server details
const char* server = "160.119.248.229"; // Server IP address
const int port = 8080;  // Server port

int imageCounter = 0; // Counter to track image files
#define MAX_IMAGES 100 // Maximum number of images to store on SD card

void deactivateCamera() {
    digitalWrite(CAM_CS_PIN, HIGH);
}

void deactivateSDCard() {
    digitalWrite(SD_CS_PIN, HIGH);
}

// Setup function (runs once at startup)
void setup() {
    Serial.begin(115200); // Start serial communication at 115200 baud rate
    while (!Serial); // Wait for Serial Monitor to open

    // Connect to WiFi network
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected!");

    // Initialize SPI and set CS pins to HIGH
    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(CAM_CS_PIN, OUTPUT);
    deactivateSDCard();
    deactivateCamera();
    SPI.begin();

    // Initialize SD card
    Serial.println("Initializing SD card...");
    deactivateCamera();
      delay(100);
    digitalWrite(SD_CS_PIN, LOW); // Activate SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("❌ SD card initialization failed!");
        deactivateSDCard();
        return;
    }
    deactivateSDCard();
    Serial.println("✅ SD Card initialized successfully.");

    // Initialize Arducam Mega Camera
    Serial.println("Initializing Arducam Mega Camera...");
    deactivateSDCard();
      delay(100);
    digitalWrite(CAM_CS_PIN, LOW); // Activate camera
    if (myCamera.begin() != CAM_ERR_SUCCESS) {
        Serial.println("❌ Failed to initialize Arducam Mega Camera!");
        deactivateCamera();
        return;
    }

    // Configure camera settings
    myCamera.setImageQuality(HIGH_QUALITY);
    myCamera.setAutoExposure(1);
    myCamera.setEV(CAM_EV_LEVEL_2);
    digitalWrite(CAM_CS_PIN, HIGH);
    Serial.println("✅ Arducam Mega Camera initialized successfully.");

    findLatestImageIndex(); // Find the latest image index stored on the SD card
}

void loop() {               // Main loop function (runs continuously)
    captureAndSendImage();  // Capture and send an image
    delay(10000);           // Wait for 10 seconds before capturing again
}

// Function to delete the oldest image when storage is full
void deleteOldestImage() {
    if (imageCounter < MAX_IMAGES) return;

    Serial.println("⚠️ Storage limit reached. Deleting the oldest image...");
    int oldestIndex = imageCounter - MAX_IMAGES;
    String oldestFilename = "/image_" + String(oldestIndex) + ".jpg";

    deactivateCamera();
    digitalWrite(SD_CS_PIN, LOW);
    if (SD.exists(oldestFilename)) {
        if (SD.remove(oldestFilename)) {
            Serial.println("✅ Deleted: " + oldestFilename);
        } else {
            Serial.println("❌ Failed to delete: " + oldestFilename);
        }
    } else {
        Serial.println("⚠️ Oldest file not found: " + oldestFilename);
    }

    // Ensure SPI is stable before reinitializing SD
    delay(200);
    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(CAM_CS_PIN, HIGH);
    delay(100);

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("❌ SD card reinitialization failed after deletion!");
    }
    deactivateSDCard();
    findLatestImageIndex(); // Find the latest image index stored on the SD card
}


// Function to capture an image and send it to the server
void captureAndSendImage() {                  // Function to capture an image and send it to the server
    Serial.println("Capturing image...");
    deactivateSDCard();
     delay(100);
    digitalWrite(CAM_CS_PIN, LOW); // Activate Camera
    int result = myCamera.takePicture(CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
     delay(100);
    if (result != CAM_ERR_SUCCESS) {
        Serial.println("❌ Failed to take picture!");
        deactivateCamera();
        return;
    }
    Serial.println("✅ Picture taken successfully!");

    uint32_t length = myCamera.getTotalLength();  // Get image size
    if (length == 0) {
        Serial.println("❌ Image length is 0. Check camera settings.");
        deactivateCamera();
        return;
    }
    Serial.print("📏 Image Size: ");
    Serial.println(length);
    Serial.print(length / 1024.0, 2);  // Convert to KB
    Serial.println(" KB");

    // Check if storage is full, delete the oldest image if necessary
    deleteOldestImage(); 

    // Create a filename and increment counter
    String filename = "/image_" + String(imageCounter) + ".jpg";
    imageCounter++;

    // Save image to SD card
    deactivateCamera(); // Deactivate camera
     delay(100);
    digitalWrite(SD_CS_PIN, LOW); // Activate SD card
    File imageFile = SD.open(filename, FILE_WRITE);
    if (!imageFile) {
        Serial.println("Failed to open file for writing");
        Serial.print("Filename: ");
        Serial.println(filename);
        Serial.println("Check if SD card is full or has too many files.");
        deactivateSDCard();
        return;
    }

    while (myCamera.getReceivedLength() > 0) {
        uint8_t buffer[384];
        int bytesRead = myCamera.readBuff(buffer, sizeof(buffer));
        imageFile.write(buffer, bytesRead);
    }
    imageFile.close();
    Serial.println("✅ Image saved to SD card: " + filename);
    deactivateSDCard();

    sendImageToServer(filename); // Send image to the server
}

// Function to find the latest image index on the SD card
void findLatestImageIndex() {   
    imageCounter = 0;
    while (true) {
        String filename = "/image_" + String(imageCounter) + ".jpg";
        deactivateCamera();
        delay(50);
        digitalWrite(SD_CS_PIN, LOW);
        if (!SD.exists(filename)) {
          deactivateSDCard();
            break;
        }
        deactivateSDCard();
        imageCounter++;
    }
    Serial.println("Starting from image index: " + String(imageCounter));
}

// Function to send the captured image to the Linux Server with Hostafrica
// Function to send image in chunks using Base64 encoding
void sendImageToServer(String filename) {
    deactivateCamera();
    digitalWrite(SD_CS_PIN, LOW);
    File imageFile = SD.open(filename, FILE_READ);
    if (!imageFile) {
        Serial.println("❌ Failed to open file for reading");
        deactivateSDCard();
        return;
    }

    Serial.println("📡 Encoding and sending image in Base64 chunks...");
    
    if (!client.connect(server, port)) {
        Serial.println("❌ Connection to server failed.");
        imageFile.close();
        deactivateSDCard();
        return;
    }

    client.println("POST /upload.php HTTP/1.1");
    client.print("Host: "); client.println(server);
    client.println("Content-Type: application/json");
    client.println("Connection: close");

    String jsonPayload = "{ \"image\": \"";
    client.print("Content-Length: "); 
    client.println(jsonPayload.length() + imageFile.size() * 1.37 + 30);
    client.println();
    client.print(jsonPayload);

    uint8_t buffer[512];  // Read in 512-byte chunks
    while (imageFile.available()) {
        int bytesRead = imageFile.read(buffer, sizeof(buffer));
        client.print(base64Encode(buffer, bytesRead));  // Send encoded chunk
        delay(5);  // Small delay to prevent overflow
    }
    imageFile.close();

    client.print("\", \"filename\": \"");
    client.print(filename);
    client.print("\" }");

    Serial.println("✅ Image sent successfully! Waiting for server response...");

    // Read and print server response
    long timeout = millis() + 5000;
    while (client.available() == 0) {
        if (millis() > timeout) {
            Serial.println("❌ Server did not respond (Timed out)");
            client.stop();
            return;
        }
    }

    Serial.println("📩 Server Response:");
    while (client.available()) {
        String response = client.readString();
        Serial.println(response);
    }

    Serial.println("✅ Server response received!");
    client.stop();
}


