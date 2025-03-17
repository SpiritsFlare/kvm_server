/*== Yeast Fluorescence IoT Biosensor Device For Estrogenicity Project ==
  ==              Updated for GW-PREV Integration                      ==
  ==              By Danelle Botha                                    
  ==              Date: February 2025                                  == */
      
      // Include necessary libraries
#include <Arduino.h>                // Include Arduino library for basic functions
//#include <WiFiS3.h>                 // Include WiFiS3 library for Arduino Uno R4 WiFi support
#include <WiFi.h>                   // Library for WiFi connectivity
#include <ArduinoHttpClient.h>      // Include HTTP client library to make HTTP requests
#include <Adafruit_SCD30.h>         // Include Adafruit library for SCD30 CO2 sensor
#include <Adafruit_VEML7700.h>      // Include Adafruit library for VEML7700 light sensor
#include <math.h>                   // Include math library for logarithmic calculations
#include <SPI.h>                    // Library for SPI communication
#include <SD.h>                     // Library for SD card interfacing
#include "Arducam_Mega.h"           // Library for Arducam Mega camera
#include "config.h"                 // Configuration file for WiFi credentials
#include "Base64.h"                 // Library for Base64 encoding




// API and platform details for MAR2PROJECT SERVER
const char* MAR2PROJECT_SERVER = "gw-prev.mar2protect.eu";     // Define the server address for API communication
const char* MAR2PROJECT_PATH = "/data/api/devices/586f95";     // Define the path for the API endpoint
const char* MAR2PROJECT_API_KEY = "yLryYNOiPN7Z6jNcQ5bVORDbckB8VE0Z"; // Define API key for authentication

// Server details for  Linux Server with Hostafrica
const char* LINUX_SERVER = "160.119.248.229"; // Server IP address
const int LINUX_PORT = 8080;  // Server port

// Define Chip Select (CS) pins for camera and SD card
#define CAM_CS_PIN 8  // Chip select pin for Arducam Mega
#define SD_CS_PIN 10  // Chip select pin for SD card

// Initialize Arducam Mega camera with defined CS pin
Arducam_Mega myCamera(CAM_CS_PIN);
WiFiClient client; // WiFi client for server communication   

// LED pins
#define LED_OnOff_PIN 2          // D6 Pin for the ON & OFF LED ()
#define LED_Lights_PIN 7         // D7 Pin for external LED (used for additional light measurements: outside absorbance sensing unit)
#define LED_590NM_PIN 9          // D9 Pin for the 590nm LED (Yellow) (used in the sensing unit)
#define LED_560NM_PIN 4          // D5 Pin for the 560nm LED (Amber)
#define LED_470NM_PIN A3          // A3 Pin for the 470nm LED (Blue)

// Define pump pins
#define PUMP1_12V_PIN 6         // D4 Pin for controlling Pump1 (12V)
#define PUMP2_12V_PIN 5         // D2 Pin for controlling Pump2 (12V)
#define PUMP3_3V_PIN  3         // D3 Pin for controlling Pump3 (3.7V)

// Analog pins for variable resistors or variable speeds for each pump
#define PUMP1_SPEED_PIN A0     // A0 Pin for controlling Pump1 (12V)
#define PUMP2_SPEED_PIN A1     // A1 Pin for controlling Pump2 (12V)
#define PUMP3_SPEED_PIN A2     // A2 Pin for controlling Pump3 (3.7V)

int pot1_value;
int pot2_value;
int pot3_value;
 
  // Global pump variables
float pump1_speed;
float Pump1RPM;
float Pump1FlowRate;

float pump2_speed;
float Pump2_RPM;
float Pump2_FlowRate;

float pump3_speed;
float Pump3_RPM;
float Pump3_FlowRate;

// Pump speed and flow rate constants
float Pump1_2MaxSpeed = 250.0;   // 12V DC maximum motor speed in RPM for pump 1 and pump 2
float Pump3MaxSpeed = 110.0;     // 3.7V DC maximum motor speed in RPM for pump 3
float Pump1_2FlowRate = 60.0;    // 12V DC maximum flow rate in mL/min for pump 1 and pump 2
float Pump3FlowRate = 4.0;       // 3.7V DC maximum flow rate in mL/min for pump 3

// Sensor objects
Adafruit_SCD30 scd30;              // Instantiate the SCD30 CO2 sensor object
Adafruit_VEML7700 veml7700;        // Instantiate the VEML7700 light sensor object

// WiFiSSLClient for HTTPS (secure WiFi connection)
WiFiSSLClient wifiSecure;          
HttpClient http(wifiSecure, MAR2PROJECT_SERVER, 443);   // Instantiate the HTTP client with secure SSL communication

// Global variables
float ReferenceLux = 0.0;   // Variable to store reference luminosity (without sample) ReferenceLux
float Absorbance = 0.0;      // Variable to store the calculated Absorbance (absorbance)

// Global variables for LED statuses
//bool LED_Lights_Status = false;  // Initially off for LED_Lights_PIN
//bool luminosity1 = false;   // Initially off for LED_590NM_PIN

int imageCounter = 0; // Counter to track image files
#define MAX_IMAGES 100 // Maximum number of images to store on SD card

// I2C multiplexer address
#define TCA9548A_ADDR 0x70 

// Function to select the I2C channel on the TCA9548A multiplexer
void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;           // Ensure the channel number is valid (0-7)
  Wire.beginTransmission(TCA9548A_ADDR);  // Begin I2C transmission to the multiplexer
  Wire.write(1 << channel);          // Select the specified channel
  Wire.endTransmission();           // End I2C transmission
}

void setup() {
  Serial.begin(115200);  // Begin serial communication with baud rate of 115200
  while (!Serial); // Wait for Serial Monitor to open

  // Configure LED pins as output
  pinMode(LED_590NM_PIN, OUTPUT);
  pinMode(LED_Lights_PIN, OUTPUT);
  pinMode(LED_470NM_PIN, OUTPUT);    // Blue LED for yEGFP fluorescence
  pinMode(LED_560NM_PIN, OUTPUT);    // Amber LED for DsRed2 fluorescence
  pinMode(LED_OnOff_PIN, OUTPUT);
   // Setup pump pins
  pinMode(PUMP1_12V_PIN, OUTPUT);
  pinMode(PUMP2_12V_PIN, OUTPUT);
  pinMode(PUMP3_3V_PIN, OUTPUT);

    // Configure potentiometer pins as input
  pinMode(PUMP1_SPEED_PIN, INPUT);
  pinMode(PUMP2_SPEED_PIN, INPUT);
  pinMode(PUMP3_SPEED_PIN, INPUT);

  // Turn off LEDs initially
  digitalWrite(LED_590NM_PIN, LOW);
  digitalWrite(LED_Lights_PIN, LOW);
  digitalWrite(LED_470NM_PIN, LOW);    // Blue LED for yEGFP fluorescence
  digitalWrite(LED_560NM_PIN, LOW);    // Amber LED for DsRed2 fluorescence
  digitalWrite(LED_OnOff_PIN, LOW);

  // Connect to WiFi network
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Begin WiFi connection with the specified SSID and password
  while (WiFi.status() != WL_CONNECTED) {  // Wait until WiFi is connected
    delay(1000);  // Wait for 1 second before checking the connection status again
    Serial.print(".");  // Print a dot while waiting
  }
  Serial.println("\nConnected to WiFi successfully!");  // Confirm successful WiFi connection

   // Initialize SCD30 sensor (CO2, temperature, humidity)
     selectI2CChannel(0);               // Select the channel for the SCD30 CO2 sensor
  if (!scd30.begin()) {
    Serial.println("Failed to initialize SCD30 sensor!");  // Display error message if initialization fails
    while (true);  // Halt the program indefinitely
  }
  Serial.println("SCD30 Sensor initialized successfully!");  // Confirm successful initialization

  // Initialize VEML7700 sensor (lux/brightness)
      selectI2CChannel(1);               // Select the channel for the VEML7700 Lux sensor
  if (!veml7700.begin()) {
    Serial.println("Failed to initialize VEML7700 sensor!");  // Display error message if initialization fails
    while (true);  // Halt the program indefinitely
  }

  veml7700.setGain(VEML7700_GAIN_1);  // Set sensor gain
  veml7700.setIntegrationTime(VEML7700_IT_100MS);  // Set integration time for sensor
  Serial.println("VEML7700 Sensor initialized successfully!");  // Confirm successful initialization

// Initialize SPI and set CS pins to HIGH
    Serial.println("Initialize SPI and set CS pins to HIGH For SD & CAM!");
    pinMode(SD_CS_PIN, OUTPUT);
    pinMode(CAM_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(CAM_CS_PIN, HIGH);
    
    SPI.begin();

    // Initialize SD card
    Serial.println("Initializing SD card...");
    digitalWrite(SD_CS_PIN, LOW);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("❌ SD card initialization failed!");
        digitalWrite(SD_CS_PIN, HIGH);
        return;
    }
    digitalWrite(SD_CS_PIN, HIGH);
    Serial.println("✅ SD Card initialized successfully.");

    // Initialize Arducam Mega Camera
    digitalWrite(CAM_CS_PIN, LOW);
    if (myCamera.begin() != CAM_ERR_SUCCESS) {
        Serial.println("❌ Failed to initialize Arducam Mega Camera!");
        digitalWrite(CAM_CS_PIN, HIGH);
        return;
    }

    // Configure camera settings
    myCamera.setImageQuality(HIGH_QUALITY);
    myCamera.setAutoExposure(1);
    myCamera.setEV(CAM_EV_LEVEL_2);
    digitalWrite(CAM_CS_PIN, HIGH);
    Serial.println("✅ Arducam Mega Camera initialized successfully.");

    findLatestImageIndex(); // Find the latest image index stored on the SD card

  
  // Take reference luminosity measurement (luminosity1) ReferenceLux
  digitalWrite(LED_590NM_PIN, HIGH);  // Turn on the 590nm LED for measurement
  digitalWrite(LED_Lights_PIN, HIGH);  // Turn on the 590nm LED for measurement
  delay(100);  // Wait for stable reading from sensor
  ReferenceLux = veml7700.readLux();  // Store the luminosity reading from the VEML7700 sensor (ReferenceLux)
  digitalWrite(LED_590NM_PIN, LOW);  // Turn off the LED after measurement
  digitalWrite(LED_Lights_PIN, LOW);
  Serial.print("Reference Luminosity (ReferenceLux): ");
  Serial.println(ReferenceLux);  // Print the reference luminosity (ReferenceLux) value to Serial Monitor
  
  digitalWrite(LED_OnOff_PIN, HIGH);  // Turn on an ON & OFF LED
}

// Function to delete the oldest image when storage is full
void deleteOldestImage() {
    if (imageCounter < MAX_IMAGES) return; // No need to delete if within limit

    Serial.println("⚠️ Storage limit reached. Deleting the oldest image...");

    // Find the oldest file
    int oldestIndex = imageCounter - MAX_IMAGES;
    String oldestFilename = "/image_" + String(oldestIndex) + ".jpg";

    if (SD.exists(oldestFilename)) {
        if (SD.remove(oldestFilename)) {
            Serial.println("✅ Deleted: " + oldestFilename);
        } else {
            Serial.println("❌ Failed to delete: " + oldestFilename);
        }
    } else {
        Serial.println("⚠️ Oldest file not found: " + oldestFilename);
    }

    // Reinitialize SD card
    SD.end();  
    delay(100);  
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("❌ SD card reinitialization failed after deletion!");
    }
}

// Function to capture an image and send it to the server
void captureAndSendImage() {                  // Function to capture an image and send it to the server
    Serial.println("Capturing image...");
    digitalWrite(CAM_CS_PIN, LOW);
    int result = myCamera.takePicture(CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
    if (result != CAM_ERR_SUCCESS) {
        Serial.println("❌ Failed to take picture!");
        digitalWrite(CAM_CS_PIN, HIGH);
        return;
    }
    Serial.println("✅ Picture taken successfully!");

    uint32_t length = myCamera.getTotalLength();  // Get image size
    if (length == 0) {
        Serial.println("❌ Image length is 0. Check camera settings.");
        digitalWrite(CAM_CS_PIN, HIGH);
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
    digitalWrite(SD_CS_PIN, LOW);
    File imageFile = SD.open(filename, FILE_WRITE);
    if (!imageFile) {
        Serial.println("Failed to open file for writing");
        Serial.print("Filename: ");
        Serial.println(filename);
        Serial.println("Check if SD card is full or has too many files.");
        digitalWrite(SD_CS_PIN, HIGH);
        return;
    }

    while (myCamera.getReceivedLength() > 0) {
        uint8_t buffer[128];
        int bytesRead = myCamera.readBuff(buffer, sizeof(buffer));
        imageFile.write(buffer, bytesRead);
    }
    imageFile.close();
    Serial.println("Image saved to SD card: " + filename);

    sendImageToServer(filename); // Send image to the server
}

// Function to find the latest image index on the SD card
void findLatestImageIndex() {   // Function to find the latest image index on the SD card
    imageCounter = 0;
    while (true) {
        String filename = "/image_" + String(imageCounter) + ".jpg";
        if (!SD.exists(filename)) {
            break;
        }
        imageCounter++;
    }
    Serial.println("Starting from image index: " + String(imageCounter));
}

// Function to send the captured image to the Linux Server with Hostafrica
void sendImageToServer(String filename) {  // Function to send the captured image to the server
    File imageFile = SD.open(filename, FILE_READ);
    if (!imageFile) {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.println("Encoding image to Base64...");
    String imageBase64 = "";
    uint8_t buffer[3];  // Process 3 bytes at a time for Base64 encoding

    while (imageFile.available()) {
        int bytesRead = imageFile.read(buffer, sizeof(buffer));
        imageBase64 += base64Encode(buffer, bytesRead);
    }
    imageFile.close();
    
    Serial.println("✅ Image encoded successfully!");

    // Prepare JSON payload
    String jsonPayload = "{ \"image\": \"" + imageBase64 + "\", \"filename\": \"" + filename + "\" }";

    Serial.println("Trying to connect to Linux Server!");
    Serial.println(" ");
    if (client.connect(LINUX_SERVER, LINUX_PORT)) {
        Serial.println("Connected to server successfully!. Sending JSON...");

        client.println("POST /upload.php HTTP/1.1");
        client.print("Host: "); client.println(LINUX_SERVER);
        client.println("Content-Type: application/json");
        client.print("Content-Length: "); client.println(jsonPayload.length());
        client.println();
        client.println(jsonPayload);

        Serial.println("✅ JSON sent successfully! Waiting for LINUX SERVER response...");

        // **Wait for response from server**
        long timeout = millis() + 5000;  // 5 seconds timeout
        while (client.available() == 0) {
            if (millis() > timeout) {
                Serial.println("❌ LINUX SERVER did not respond (Timed out)");
                client.stop();
                return;
            }
        }

        // **Read and print the response**
        Serial.println("📩 LINUX SERVER Response:");
        while (client.available()) {
            String response = client.readString();
            Serial.println(response);
        }

        Serial.println("✅ Linux Server response received!");
    } else {
        Serial.println("❌ Connection to the Linux server failed.");
    }

    client.stop();
}

void updatePumpSpeed() {

    // Read potentiometer values
  pot1_value = analogRead(PUMP1_SPEED_PIN);  // Read Potentiometer 1
   pot2_value = analogRead(PUMP2_SPEED_PIN);  // Read Potentiometer 2
   pot3_value = analogRead(PUMP3_SPEED_PIN);  // Read Potentiometer 3
    // Map potentiometer values to PWM range (0-255)
   pump1_speed = map(pot1_value, 0, 1023, 0, 255);
   pump2_speed = map(pot2_value, 0, 1023, 0, 255);
   pump3_speed = map(pot3_value, 0, 1023, 0, 255);

   // Control pumps with their respective speeds
  analogWrite(PUMP1_12V_PIN, pump1_speed);
  analogWrite(PUMP2_12V_PIN, pump2_speed);
  analogWrite(PUMP3_3V_PIN, pump3_speed);

  // Formula for converting or Pump data calculations 
  // Calculate pump data for Pump1
   Pump1RPM = Pump1_2MaxSpeed * (pump1_speed / 255.0);
   Pump1FlowRate = Pump1_2FlowRate * (pump1_speed / 255.0);

  // Calculate pump data for Pump2
   Pump2_RPM = Pump1_2MaxSpeed * (pump2_speed / 255.0);
   Pump2_FlowRate = Pump1_2FlowRate * (pump2_speed / 255.0);

  // Calculate pump data for Pump3
   Pump3_RPM = Pump3MaxSpeed * (pump3_speed / 255.0);
   Pump3_FlowRate = Pump3FlowRate * (pump3_speed / 255.0);

   Serial.println("updatePumpSpeed() running...");


}

void loop() {

    //updatePumpSpeed();
                
    static unsigned long lastPumpUpdateTime = 0;
    static unsigned long lastDataSentTime = 0;

    // Update pump speed every 1 seconds
    if (millis() - lastPumpUpdateTime >= 1000) {
        lastPumpUpdateTime = millis();
        updatePumpSpeed();
        
    }

    // Check if it's time to send data (every 9 seconds)
    if (millis() - lastDataSentTime >= 9000) {  // Wait for a minute before sending data again
        
        lastDataSentTime = millis();

        captureAndSendImage();
        findLatestImageIndex();

  // Variables for sensor readings in each cycle
  float co2 = 0.0, temperature = 0.0, humidity = 0.0;  // Variables to store SCD30 readings
  float SampleLux = 0.0;  // Variable to store second luminosity measurement with SampleLux 

  // Read SCD30 sensor data (CO2, temperature, and humidity)
  if (scd30.dataReady()) {  // Check if data is available from the SCD30 sensor
    if (scd30.read()) {  // Read the data from the SCD30 sensor
      co2 = scd30.CO2;  // Store CO2 level
      temperature = scd30.temperature;  // Store temperature value
      humidity = scd30.relative_humidity;  // Store humidity level
    }
  } else {
    Serial.println("SCD30 data not ready.");  // If data is not ready, print an error message
  }

  // Read VEML7700 sensor data for luminosity2 SampleLux (with sample)
  digitalWrite(LED_590NM_PIN, HIGH);  // Turn on the 590nm LED for measurement
  digitalWrite(LED_Lights_PIN, HIGH);
  delay(100);  // Wait for stable reading from sensor
  SampleLux = veml7700.readLux();  // Store the luminosity (SampleLux)reading from the VEML7700 sensor
  digitalWrite(LED_590NM_PIN, LOW);  // Turn off the LED after measurement
  digitalWrite(LED_Lights_PIN, LOW);

  // Calculate absorbance (Absorbance) using luminosity1 (ReferenceLux) and luminosity2 (SampleLux)
  if (ReferenceLux > 0) {  // Check if the reference luminosity is non-zero to avoid division by zero
    Absorbance = -log10(SampleLux / ReferenceLux);  // Calculate absorbance using the formula
  } else {
    Absorbance = 0;  // Set Absorbance to zero if luminosity1 (ReferenceLux) is zero
  }


  // LED Control for Fluorescence (yEGFP and DsRed2)
      digitalWrite(LED_470NM_PIN, HIGH);  // Excite yEGFP with 470nm LED
      delay(500);                          // Wait for exposure time
      digitalWrite(LED_470NM_PIN, LOW);   // Turn off the 470nm LED

      digitalWrite(LED_560NM_PIN, HIGH);  // Excite DsRed2 with 560nm LED
      delay(500);                          // Wait for exposure time
      digitalWrite(LED_560NM_PIN, LOW);   // Turn off the 560nm LED

  // Print all sensor readings to the Serial Monitor for debugging purposes
  Serial.println();
  Serial.print("CO2: ");
  Serial.print(co2);  // Print CO2 level
  Serial.print(" ppm, Temp: ");
  Serial.print(temperature);  // Print temperature
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity); Serial.println(" %"); // Print humidity
  Serial.println();

  // Print luminosity measurements
  Serial.print("Reference Luminosity (ReferenceLux): ");
  Serial.println(ReferenceLux);  // Print reference luminosity value in lux
  Serial.print("SampleLux: ");
  Serial.print(SampleLux);  // Print second sample luminosity measurement value in Lux
  Serial.print(" lux, Absorbance (Absorbance): ");
  Serial.println(Absorbance); Serial.println(); // Print calculated Absorbance value

     // Print pump data to Serial Monitor
    // Pump 1
  Serial.print("Pot1 Value: ");  Serial.print(pot1_value);
  Serial.print(", Pump1 Speed: "); Serial.print(pump1_speed);
  Serial.print(", Pump1 RPM: "); Serial.print(Pump1RPM);
  Serial.print(" rpm, Pump1 Flow Rate "); Serial.print(Pump1FlowRate); Serial.println(" mL/Min");
    // Pump 2
  Serial.print("Pot2 Value: "); Serial.print(pot2_value);
  Serial.print(", Pump2 Speed: "); Serial.print(pump2_speed);
  Serial.print(", Pump2 RPM: "); Serial.print(Pump2_RPM);
  Serial.print(" rpm, Pump2 Flow Rate: "); Serial.print(Pump2_FlowRate); Serial.println(" mL/Min");
    // Pump 3
  Serial.print("Pot3 Value: "); Serial.print(pot3_value);
  Serial.print(", Pump3 Speed: "); Serial.print(pump3_speed);
  Serial.print(", Pump3 RPM: "); Serial.print(Pump3_RPM);
  Serial.print(" rpm, Pump3 Flow Rate: "); Serial.print(Pump3_FlowRate); Serial.println(" mL/Min");
  Serial.println();

   // Update LED status
  //LED_590NM_Status = digitalRead(LED_590NM_PIN);
  //luminosity1 = digitalRead(LED_590NM_PIN);

  //LED_Lights_Status = digitalRead(LED_Lights_PIN);  // Read the status of the LED_Lights_PIN

  
  // Create JSON payload for API request to send sensor data to the MAR2PROJECT SERVER
  String payload = "{";
  payload += "\"CO2_system\": " + String(co2) + ", ";  // Placeholder for system CO2 value
  payload += "\"CO2_ambient\": " + String(co2) + ", ";  // Placeholder for biofilm CO2 value
  payload += "\"Humidity\": " + String(humidity) + ", ";  // Placeholder for ambient CO2 value
  payload += "\"Temp\": " + String(temperature) + ", ";  // Include temperature data
  
  payload += "\"ReferenceLux\": " + String(ReferenceLux) + ", ";  // Include reference luminosity
  payload += "\"SampleLux\": " + String(SampleLux ) + ", ";  // Include second luminosity
  payload += "\"Absorbance\": " + String(Absorbance) + ", ";  // Include calculated absorbance

     // For Pumps
  payload += "\"Pump1Speed\": " + String(pump1_speed) + ", ";
  payload += "\"Pump1RPM\": " + String(Pump1RPM) + ", ";
  payload += "\"Pump1FlowRate\": " + String(Pump1FlowRate) + ", ";

  payload += "\"Pump2Speed\": " + String(pump2_speed) + ", ";
  payload += "\"Pump2RPM\": " + String(Pump2_RPM) + ", "; 
  payload += "\"Pump2FlowRate\": " + String(Pump2_FlowRate) + ", "; 

  payload += "\"Pump3Speed\": " + String(pump3_speed) + ", ";
  payload += "\"Pump3RPM\": " + String(Pump3_RPM) + ", ";
  payload += "\"Pump3FlowRate\": " + String(Pump3_FlowRate)+ ", ";  // Remove trailing comma
  //payload += "\"LED_Lights_Status\": " + String(LED_Lights_Status ? "true" : "false") + ", ";
  //payload += "\"luminosity1\": " + String(luminosity1 ? "true" : "false");
  payload += "\"luminosity1\": " + String(digitalRead(LED_590NM_PIN));
  payload += "}";

  // Send data to API if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {  // Check if WiFi is connected
    http.beginRequest();  // Begin the HTTP request
    http.post(MAR2PROJECT_PATH + String("?api_key=") + MAR2PROJECT_API_KEY);  // Send a POST request with API key
    http.sendHeader("Content-Type", "application/json");  // Set content type as JSON
    http.sendHeader("Content-Length", payload.length());  // Set the length of the payload
    http.beginBody();  // Start the body of the request
    http.print(payload);  // Send the JSON payload
    http.endRequest();  // End the HTTP request

    int statusCode = http.responseStatusCode();  // Get the status code of the response
    String response = http.responseBody();  // Get the response body
    Serial.print("Status Code: ");
    Serial.println(statusCode);  // Print the response status code
    Serial.println("Response: " + response);  // Print the response body
  } else {
    Serial.println("WiFi disconnected, cannot send data");  // If WiFi is disconnected, print an error message
  }

  } }


