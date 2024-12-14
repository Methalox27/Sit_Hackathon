#include <Wire.h>
#include <SPI.h>
// #include "heartRate.h"
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MAX3010x.h>
#include <Adafruit_SH110X.h> // Use SH1106G instead of SSD1306
#include "filters.h"

#include <WiFi.h>
#include <WebServer.h>

// Define the credentials for the access point
const char* apSSID = "ESP32-AP";       // The SSID of the Access Point
const char* apPassword = "123456789";  // The password for the Access Point
const char* sosMessage;

// Create an instance of the web server
WebServer server(80);



// Initialize ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// I2C Address for the OLED
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

// Threshold for movement detection (adjustable)
const float movementThreshold = 1;  // Minimum acceleration change (in g) for movement detection

// Threshold for free-fall detection (adjustable)
const float freeFallThreshold = 0.0; // Threshold for free-fall in g (less than 1 g)

// Threshold for impact detection (adjustable)
const float fallImpactThreshold = 12.0; // Threshold for detecting impact after fall (greater than 12 g)

// Flags for free-fall and impact detection
bool freeFallDetected = false;

// Previous acceleration values for movement detection
float prevX = 0, prevY = 0, prevZ = 0;

// Variable to store the X, Y, and Z accelerometer readings
sensors_event_t event;

// Global Variables
bool sosMode = false;
int touchCount = 0;
const unsigned long timeWindow = 1000;
const int LED_BUILTIN = 2;
const int touchPin = 15;
unsigned long lastTapTimeTouch = 0;
int buzzerPin = 26;


// Heart Rate Monitor Variables
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int bpm;
int beatAvg;
bool stableReadings = false;
unsigned long startTime;
const unsigned long stabilizationTime = 5000; // 5 seconds stabilization period

// GPS Variables
TinyGPSPlus gps;
#define RXD2 16
#define TXD2 17

// Variables for detecting spikes
long prevIrValue = 0;   // Previous IR value
long spikeValue = 0;    // Amplified spike value
long threshold = 50000; // Minimum IR value to detect a finger
int windowSize = 5;     // Size of the moving average window
long irValues[5];       // Array to hold the last few IR values for averaging

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(touchPin, INPUT);

  // Initialize MAX30105 Sensor
  // if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
  //   Serial.println("MAX30105 not found. Check wiring/power.");
  //   while (1);
  // }
  // Initialize the accelerometer
  if (!accel.begin()) {
    Serial.println("Couldn't find ADXL345 sensor!");
    while (1);
  }
  // particleSensor.setup();
  // particleSensor.setPulseAmplitudeRed(0x0A);
  // particleSensor.setPulseAmplitudeGreen(0);
  // particleSensor.enableDIETEMPRDY();

  startTime = millis(); // Start stabilization period
  Serial.println("Setup complete.");
  // Optionally set the range (e.g., +/- 2g, 4g, 8g, or 16g)
  accel.setRange(ADXL345_RANGE_2_G);  // Using ±2g range for free fall detection
  
  // Initialize previous accelerometer values for movement detection
  prevX = prevY = prevZ = 0;
  
  Serial.println("Movement and Fall Detection initialized.");
  pinMode(buzzerPin, OUTPUT);

  // Initialize the OLED
  delay(250);
  display.begin(0x3C, true);
  delay(2000); // Allow extra time for the display to initialize
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.display(); // Show blank screen

  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("Sensor not found!"));
    display.display();
    while(1);
  }

  display.clearDisplay();
  initDrawScreen(); 
  // Set up the ESP as an Access Point
  WiFi.softAP(apSSID, apPassword);
  
  // Print the ESP's IP address in AP mode
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());  // This prints the IP address

  // Set up the root URL handler to serve the main page
  server.on("/", handleRoot);

  // Set up a URL handler to fetch the updated variable
  server.on("/getVariable", handleGetVariable);
  // Start the server
  server.begin();
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

// Store last BPM to reduce unnecessary display updates
int32_t last_displayed_bpm = -1;
bool last_finger_status = false;


void loop() {
  unsigned long currentTime = millis();
  server.handleClient();

  handleTouchSOS(currentTime);
  readHeartRate();
  handleTemperatureSOS();
  // handleSpikes(currentTime);

  // Get accelerometer data
  accel.getEvent(&event);

  // Check for movement
  // detectMovement(event);
  bool movementDetected = detectMovement(event);
  // Detect free-fall and impact
  detectFallImpact(event);

  digitalWrite(LED_BUILTIN, sosMode ? HIGH : LOW);

  detectIdleAlert(millis());
  // Call the sleep detection function
  detectSleep(currentTime, bpm, movementDetected); // Pass heart rate and movement status
  handleHeartFailure();
}

void handleTouchSOS(unsigned long currentTime) {
  if (digitalRead(touchPin) == HIGH) {
    if (currentTime - lastTapTimeTouch <= timeWindow) {
      touchCount++;
    } else {
      touchCount = 1;
    }
    lastTapTimeTouch = currentTime;

    if (!sosMode && touchCount == 3) {
      activateSOS("Touch-based SOS Activated!");
      // sosMessage = "Touch-based SOS Activated!";
    } else if (sosMode && touchCount == 3) {
      sosMode = false;
      Serial.println("Touch-based SOS Deactivated!");
    }
    delay(200); // Debounce
  }
}

void printGPSData() {
  while (Serial2.available() > 0) {
    char data = Serial2.read();
    gps.encode(data);
  }
  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }
}

void handleTemperatureSOS() {
  static unsigned long lastTemperatureCheckTime = 0;
  if (millis() - lastTemperatureCheckTime > 1000) { // Check every 1 second
    float tempDeg = particleSensor.readTemperature() - 6.5;
    Serial.println(tempDeg);
    if (tempDeg > 42) {
      activateSOS("Heatstroke detected!");
    } else if (tempDeg > 39) {
      activateSOS("Fever detected!");
    } else if (tempDeg < 30) {
      activateSOS("Hypothermia detected!");
    }
    lastTemperatureCheckTime = millis();
  }
}

void activateSOS(const char* message) {
  if (!sosMode) {
    sosMode = true;
    Serial.println(message);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println(message);
    sosMessage = message;
    display.display();
    printGPSData();
    
    unsigned long currentMillis = millis();
    unsigned long buzzerDuration = 500; // Duration of buzzer activation (in ms)
    unsigned long previousMillis = currentMillis;

    while (millis() - previousMillis < buzzerDuration) {
      digitalWrite(buzzerPin, HIGH);
    }
    digitalWrite(buzzerPin, LOW);
  }
  display.clearDisplay();
}

void readHeartRate(){
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    differentiator.reset();
    averager_bpm.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    displayMeasuredValues(false, 0);
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    if(!isnan(current_diff) && !isnan(last_diff)) {
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }

      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          bpm = 60000 / (crossed_time - last_heartbeat) - 40;
          if (bpm>120){
            bpm = bpm - 40;
          }
          displayMeasuredValues(false, bpm);
        }
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = current_diff;
  } else {
    displayMeasuredValues(true, 0);
  }
}

void handleHeartFailure() {
  if (bpm > 180) {
    activateSOS("Heart Failure: Extremely high heart rate detected!");
  } else if (bpm < 40 && bpm> 0) {
    activateSOS("Heart Failure: Critically low heart rate detected!");
  }
}

float calculateSpO2(int beatAvg) {
  float scaledBeatAvg = ((float(beatAvg) - 60) / (110 - 60)) * 4 - 2;
  float SpO2 = (1 / (1 + pow(M_E, -scaledBeatAvg))) * 10 + 90;
  return constrain(SpO2, 0, 98);
}

float calculateSystolic(int beatAvg) {
  return 90 + ((float(beatAvg) - 60) / (110 - 60)) * (140 - 90);
}

float calculateDiastolic(int beatAvg) {
  return 60 + ((float(beatAvg) - 60) / (110 - 60)) * (90 - 60);
}

// void handleSpikes(unsigned long currentTime) {
//   static long irValues[5] = {0};
//   static int index = 0;

//   // Shift old values and add new IR value
//   irValues[index] = particleSensor.getIR();
//   index = (index + 1) % windowSize;

//   long sum = 0;
//   for (int i = 0; i < windowSize; i++) {
//     sum += irValues[i];
//   }

//   long avgIrValue = sum / windowSize;

//   // Detect rapid changes to simulate spikes
//   if (avgIrValue > threshold) {
//     long spike = abs(avgIrValue - prevIrValue);
//     spike = constrain(spike, 0, 10000);
//     // Serial.println(spike);
//   } //else {
//   //   Serial.println(0); // No finger detected, no spike
//   // }

//   prevIrValue = avgIrValue;

//   delay(10); // Adjust for smoother plotting
// }


bool detectMovement(sensors_event_t event) {
  // Movement detection logic: Calculate the total change in acceleration
  float deltaX = abs(event.acceleration.x - prevX);
  float deltaY = abs(event.acceleration.y - prevY);
  float deltaZ = abs(event.acceleration.z - prevZ);

  // Check if movement has occurred
  if (deltaX > movementThreshold || deltaY > movementThreshold || deltaZ > movementThreshold) {
    // Serial.println("Movement detected!");
    prevX = event.acceleration.x;
    prevY = event.acceleration.y;
    prevZ = event.acceleration.z;
    return true; // Movement detected
  } else {
    // Serial.println("No movement detected.");
    prevX = event.acceleration.x;
    prevY = event.acceleration.y;
    prevZ = event.acceleration.z;
    return false; // No movement detected
  }
}

// Function to detect free fall and impact based on acceleration threshold
void detectFallImpact(sensors_event_t event) {
  // Calculate the total acceleration (magnitude) for fall detection
  float totalAcceleration = sqrt(event.acceleration.x * event.acceleration.x +
                                  event.acceleration.y * event.acceleration.y +
                                  event.acceleration.z * event.acceleration.z);
  // Serial.println(totalAcceleration);
  // Detect free-fall condition (acceleration close to 0 g)
  if (freeFall(totalAcceleration)) {
    freeFallDetected = true;
    Serial.println("Free Fall Detected!");
  }

  // Detect impact after free fall
  if (freeFallDetected && totalAcceleration > fallImpactThreshold) {
    activateSOS("Fall Impact Detected!");
    freeFallDetected = false;  // Reset after detecting the impact
  }
}

// Function to detect free fall based on total acceleration
bool freeFall(float totalAcceleration) {
  if (totalAcceleration < freeFallThreshold) {
    return true;  // Free fall detected (close to 0 g)
  }
  return false;
}

void detectIdleAlert(unsigned long currentTime) {
  static unsigned long lastMovementTime = 0;
  static bool idleAlertActive = false;

  // If movement is detected, reset the idle timer
  if (digitalRead(touchPin) == HIGH || 
      abs(event.acceleration.x - prevX) > movementThreshold || 
      abs(event.acceleration.y - prevY) > movementThreshold || 
      abs(event.acceleration.z - prevZ) > movementThreshold) {
        
    lastMovementTime = currentTime; // Update the last movement time
    idleAlertActive = false; // Reset idle alert flag
  }

  // Check if 1 minute (60000 ms) has passed without movement
  if (currentTime - lastMovementTime > 20000 && !idleAlertActive) {
    Serial.println("No movement detected for20 Secs. Please move!");
    idleAlertActive = true; // Set the alert flag to avoid repeated messages
  }
}

void detectSleep(unsigned long currentTime, int bpm, bool movementDetected) {
  static unsigned long sleepStartTime = 0;
  static bool sleepDetected = false;

  if (bpm < 60 && !movementDetected) {
    // If heart rate is below 60 and no movement, start sleep timer
    if (sleepStartTime == 0) {
      sleepStartTime = currentTime; // Start the sleep timer
    }
    // Check if 30 seconds of inactivity and low heart rate have passed
    if (currentTime - sleepStartTime > 30000 && !sleepDetected) {
      Serial.println("Sleep detected!");
      sleepDetected = true; // Set flag to avoid repeated detection
    }
  } else {
    // Reset sleep timer if conditions aren't met
    sleepStartTime = 0;
    // Serial.println("Sleep End!");
    sleepDetected = false;
  }
}

void initDrawScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println(F("BPM"));
  display.display();
}

void displayMeasuredValues(bool no_finger, int32_t beatAvg) {
  if (no_finger) {
    if (!last_finger_status) {
      display.fillRect(0, 10, SCREEN_WIDTH, 32, SH110X_BLACK); // Clear the area where text is displayed
      display.setCursor(0, 10);
      display.setTextSize(1);
      display.println(F("Not Equipped"));
      display.display();
      last_finger_status = true;
    }
  } else if (beatAvg >= 10) {
    if (last_displayed_bpm != beatAvg || last_finger_status) {
      display.fillRect(0, 10, SCREEN_WIDTH, 32, SH110X_BLACK); // Clear only the area where BPM is displayed
      display.setCursor(10, 10);
      display.setTextSize(1);
      display.print(F("BPM: "));
      display.println(beatAvg);
      display.print(F("Temp: "));
      display.println(particleSensor.readTemperature() - 6.5);
      display.print(F("SpO2: "));
      display.println(calculateSpO2(beatAvg));
      display.print(F("BP: "));
      display.print(calculateSystolic(beatAvg));
      display.print("/");
      display.println(calculateDiastolic(beatAvg));
      display.display();
      last_displayed_bpm = beatAvg;
      last_finger_status = false;
    }
  }
}

void handleRoot() {
  String htmlContent = "<html><head>";
  
  // Add CSS for styling
  htmlContent += "<style>";
  htmlContent += "body { font-family: Arial, sans-serif; background-color: #f4f4f9; margin: 0; padding: 0; color: #333; }";
  htmlContent += "h1 { text-align: center; color: #4CAF50; }";
  htmlContent += "p { font-size: 18px; margin: 10px 0; text-align: center; }";
  htmlContent += "span { font-weight: bold; color: #333; }";
  htmlContent += "#container { width: 80%; margin: 0 auto; padding: 20px; background-color: white; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); text-align: center; }";
  htmlContent += "#container p { font-size: 20px; color: #555; }";
  htmlContent += "h1 { font-size: 36px; color: #333; }";
  htmlContent += ".info { font-size: 22px; margin-bottom: 15px; color: #444; }";
  htmlContent += "span { color: #007BFF; }";
  htmlContent += "</style>";

  htmlContent += "</head><body>";
  
  // Wrap the content in a container div for styling
  htmlContent += "<div id='container'>";
  htmlContent += "<h1>Neuer Gloves</h1>";
  
  // Display Heart Rate, Temperature, SpO2, and Blood Pressure
  htmlContent += "<p class='info'>Heart Rate: <span id='heartrate'>" + String(bpm) + "</span> BPM</p>";
  htmlContent += "<p class='info'>Temperature: <span id='temperature'>" + String(particleSensor.readTemperature() - 6.5) + "</span> °C</p>";
  htmlContent += "<p class='info'>SpO2: <span id='spo2'>" + String(calculateSpO2(bpm)) + "</span>%</p>";
  htmlContent += "<p class='info'>Blood Pressure: <span id='bp'>" + String(calculateSystolic(bpm)) + "/" + String(calculateDiastolic(bpm)) + "</span> mmHg</p>";
  htmlContent += "<p class='info'>GPS Coordinates: " + String(printGPSData()); 
  htmlContent += "</div>";  // End of container div
  htmlContent += "</div>";  
  htmlContent += "<p class='info'>Emergency detected: <span id='sos'>" + String(sosMessage);
  htmlContent += "</div>";  
  htmlContent += "<script>";
  
  // JavaScript to periodically fetch the updated values from the server
  htmlContent += "setInterval(function() {";
  htmlContent += "  fetch('/getVariable')";  // Request the variable
  htmlContent += "    .then(response => response.json())";  // Expect JSON data
  htmlContent += "    .then(data => {";
  htmlContent += "      document.getElementById('heartrate').innerText = data.heartrate;";
  htmlContent += "      document.getElementById('sosMessage').innerText = data.sosMessage;";
  htmlContent += "      document.getElementById('temperature').innerText = data.temperature;";
  htmlContent += "      document.getElementById('spo2').innerText = data.spo2;";
  htmlContent += "      document.getElementById('bp').innerText = data.bp;";
  htmlContent += "    });";
  htmlContent += "}, 1000);";  // Every 1 second, update the value
  
  htmlContent += "</script>";
  htmlContent += "</body></html>";

  server.send(200, "text/html", htmlContent);
}



void handleGetVariable() {
  // Create a JSON object with the necessary values
  String jsonResponse = "{";
  jsonResponse += "\"heartrate\": " + String(bpm) + ",";
  jsonResponse += "\"temperature\": " + String(particleSensor.readTemperature() - 6.5) + ",";
  jsonResponse += "\"spo2\": " + String(calculateSpO2(bpm)) + ",";
  jsonResponse += "\"bp\": \"" + String(calculateSystolic(bpm)) + "/" + String(calculateDiastolic(bpm)) + "\"";
  jsonResponse += "\"sosMessage\": \"" + String(sosMessage) + ",";
  jsonResponse += "}";

  // Send the response as JSON
  server.send(200, "application/json", jsonResponse);
}
