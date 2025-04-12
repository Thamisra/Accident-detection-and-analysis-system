# Accident-detection-and-analysis-system
#Accident Detection &amp; Alert System using ESP32 - Sent real-time alerts to Telegram using Chatbot for emergency response
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <cstdlib>  // For generating random numbers
#include <ctime>    // For seeding the random number generator

// WiFi and Telegram Configuration
const char* ssid = "Enter your WiFi name";
const char* password = "Enter pass word";
const String BOT_TOKEN = "Enter HTTT API";
const String CHAT_ID = "Enter Chat ID";

// Hardware Configuration
#define GPS_TX 17
#define GPS_RX 16

// Threshold Constants
#define IMPACT_THRESHOLD 3.96       // 3.96G threshold for major impact
#define MINOR_COLLISION_THRESHOLD 3.2 // 3.2G for minor collision
#define HARD_BRAKING_THRESHOLD 2.5   // 2.5G for hard braking
#define POTHOLE_THRESHOLD 2.8        // 2.8G for pothole hit
#define SPEED_THRESHOLD 1.5          // Minimum speed for impact detection (km/h)
#define SPEED_LIMIT 1.5              // Speed limit threshold (km/h)
#define SPEED_CHANGE_THRESHOLD 38.0  // 38% speed change for high-speed crash
#define BRAKING_CHANGE_THRESHOLD 33.0 // 33% speed change for hard braking

// System Constants
#define BUFFER_SIZE 10               // Vibration filter buffer size
#define IMPACT_WINDOW 3000           // 3-second window for multiple impacts
#define POST_IMPACT_CHECK_TIME 5000  // 5-second monitoring after impact
#define COORD_DISPLAY_INTERVAL 2000  // Switch lat/lon every 2 seconds
#define ALERT_DISPLAY_TIME 5000      // 5 seconds for accident alerts
#define TELEGRAM_CHECK_INTERVAL 10000 // 10 seconds for Telegram checks

// Initialize components
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16x2 display
HardwareSerial gpsSerial(1);         // UART1 for GPS
TinyGPSPlus gps;                     // GPS parser
Adafruit_MPU6050 mpu;                // Accelerometer
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// Vibration filtering
float accelBuffer[BUFFER_SIZE];
int bufferIndex = 0;
float averageAccel = 0;

// Detection states
bool accidentDetected = false;
bool potentialAccident = false;
bool speedLimitExceeded = false;
bool isHardBraking = false;
bool isPothole = false;
unsigned long lastImpactTime = 0;
unsigned long impactDetectionTime = 0;
unsigned long speedExceededTime = 0;
unsigned long lastCoordSwitch = 0;
unsigned long alertStartTime = 0;
unsigned long lastTelegramCheck = 0;
bool showLat = true;
float speedAtImpact = 0.0;
bool messageSent = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize LCD
  lcd.begin();
  lcd.clear();
  lcd.backlight();
  lcd.print("Initializing...");
  
  // Initialize I2C (ESP32: SDA=21, SCL=22)
  Wire.begin(21, 22);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    lcd.clear();
    lcd.print("MPU6050 Error!");
    while (1) delay(10);
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Initialize buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    accelBuffer[i] = 0;
  }
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.print("Connecting WiFi...");
  Serial.print("Connecting to WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  lcd.clear();
  lcd.print("WiFi connected!");
  
  // Use secure connection
  client.setInsecure();  
  
  // Initialize random number generator
  srand(time(0));  // Seed the random number generator with current time
  
  delay(1000);
  lcd.clear();
  lcd.print("System Ready");
}

void loop() {
  // 1. Process GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // 2. Check speed limit
  if (gps.speed.isValid()) {
    if (gps.speed.kmph() > SPEED_LIMIT) {
      speedLimitExceeded = true;
      speedExceededTime = millis();
    } else if (millis() - speedExceededTime > IMPACT_WINDOW) {
      speedLimitExceeded = false;
    }
  }

  // 3. Read accelerometer
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate total acceleration in G units
  float currentAccel = sqrt(a.acceleration.x*a.acceleration.x + 
                        a.acceleration.y*a.acceleration.y + 
                        a.acceleration.z*a.acceleration.z) / 9.81;
  
  // Update vibration filter
  accelBuffer[bufferIndex] = currentAccel;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  // Calculate moving average
  averageAccel = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    averageAccel += accelBuffer[i];
  }
  averageAccel /= BUFFER_SIZE;

  // Reset scenario flags
  isHardBraking = false;
  isPothole = false;

  // 4. Enhanced impact detection logic
  if (currentAccel > POTHOLE_THRESHOLD) {
    if (currentAccel < MINOR_COLLISION_THRESHOLD) {
      // Pothole hit scenario (2.8G-3.2G)
      isPothole = true;
      Serial.println("Pothole detected");
    }
    else if (currentAccel >= MINOR_COLLISION_THRESHOLD && currentAccel < IMPACT_THRESHOLD) {
      // Minor collision scenario (3.2G-3.96G)
      impactDetectionTime = millis();
      speedAtImpact = gps.speed.isValid() ? gps.speed.kmph() : 0;
      potentialAccident = true;
      Serial.println("Minor collision detected");
    }
    else if (currentAccel >= IMPACT_THRESHOLD) {
      // Major impact scenario (>=3.96G)
      if (gps.speed.isValid() && gps.speed.kmph() > SPEED_THRESHOLD) {
        impactDetectionTime = millis();
        speedAtImpact = gps.speed.kmph();
        potentialAccident = true;
        Serial.println("Major impact detected");
        
        // Check for multiple impacts (Multi-Impact Crash scenario)
        if (millis() - lastImpactTime < IMPACT_WINDOW) {
          accidentDetected = true;
          potentialAccident = false;
          alertStartTime = millis();
          messageSent = false; // Reset message flag when new accident detected
        }
        lastImpactTime = millis();
      }
    }
  }

  // 5. Enhanced post-impact verification
  if (potentialAccident && (millis() - impactDetectionTime < POST_IMPACT_CHECK_TIME)) {
    if (gps.speed.isValid() && speedAtImpact > 0) {
      float speedChange = abs(gps.speed.kmph() - speedAtImpact) / speedAtImpact * 100;
      
      // High-Speed Crash scenario (38% drop)
      if (speedChange >= SPEED_CHANGE_THRESHOLD) {
        accidentDetected = true;
        potentialAccident = false;
        alertStartTime = millis();
        messageSent = false; // Reset message flag when accident confirmed
      }
      // Hard Braking scenario (33% drop)
      else if (speedChange >= BRAKING_CHANGE_THRESHOLD && currentAccel >= HARD_BRAKING_THRESHOLD) {
        isHardBraking = true;
        Serial.println("Hard braking detected");
      }
    }
  } else if (potentialAccident) {
    potentialAccident = false;
    Serial.println("Impact was temporary");
  }

  // 6. Update display according to verified response table
  lcd.clear();

  // First line: Priority alerts or coordinates
  if (accidentDetected) {
    if (speedLimitExceeded) {
      lcd.print("SPEED EXCEED +");
    } else {
      lcd.print("ACCIDENT");
    }
  } else if (speedLimitExceeded) {
    lcd.print("SPEED EXCEED");
  } else if (potentialAccident) {
    lcd.print("CHECKING IMPACT...");
  } else if (!gps.location.isValid() && currentAccel > IMPACT_THRESHOLD) {
    lcd.print("NO GPS SIGNAL");
  } else if (gps.location.isValid()) {
    // Alternate between lat/lon every 2 seconds
    if (millis() - lastCoordSwitch > COORD_DISPLAY_INTERVAL) {
      showLat = !showLat;
      lastCoordSwitch = millis();
    }
    
    lcd.print(showLat ? "LA:" : "LO:");
    lcd.print(showLat ? gps.location.lat(), 4 : gps.location.lng(), 4);
  } else {
    lcd.print("NO GPS SIGNAL");
  }

  // Second line: Status information
  lcd.setCursor(0, 1);
  if (accidentDetected) {
    lcd.print("SEND HELP!");
  } else if (speedLimitExceeded) {
    lcd.print("Slow down!");
  } else if (isPothole) {
    lcd.print("S:");
    lcd.print(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2);
    lcd.print("km/h A:");
    lcd.print(currentAccel, 1);
    lcd.print("G");
  } else if (isHardBraking) {
    lcd.print("S:");
    lcd.print(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2);
    lcd.print("km/h A:");
    lcd.print(currentAccel, 1);
    lcd.print("G");
  } else if (gps.speed.isValid()) {
    lcd.print("S:");
    lcd.print(gps.speed.kmph(), 2);
    lcd.print("km/h");
    
    if (!potentialAccident) {
      lcd.print(" A:");
      lcd.print(currentAccel, 1);
      lcd.print("G");
    }
  } else {
    lcd.print("A:");
    lcd.print(currentAccel, 1);
    lcd.print("G");
  }

  // 7. Handle alert display timing
  if (accidentDetected && (millis() - alertStartTime > ALERT_DISPLAY_TIME)) {
    accidentDetected = false;
  }

  // 8. Check for Telegram messages periodically
  if (millis() - lastTelegramCheck > TELEGRAM_CHECK_INTERVAL) {
    lastTelegramCheck = millis();
    
    // If accident is detected and message not yet sent
    if (accidentDetected && !messageSent) {
      sendAccidentAlert();
      messageSent = true;
    }
    
    // Check for incoming messages (optional)
    // int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    // while(numNewMessages) {
    //   handleNewMessages(numNewMessages);
    //   numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    // }
  }

  // 9. Serial monitor output for debugging
  Serial.print("Location: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("INVALID");
  }
  
  Serial.print(" | Speed: ");
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph(), 2);
    Serial.print("km/h");
  } else {
    Serial.print("INVALID");
  }
  
  Serial.print(" | Accel: ");
  Serial.print(currentAccel, 2);
  Serial.print("G");
  
  if (accidentDetected) {
    Serial.print(" | ACCIDENT!");
  } else if (speedLimitExceeded) {
    Serial.print(" | SPEEDING!");
  } else if (potentialAccident) {
    Serial.print(" | CHECKING...");
  } else if (isHardBraking) {
    Serial.print(" | HARD BRAKING");
  } else if (isPothole) {
    Serial.print(" | POTHOLE");
  }
  
  Serial.println();

  delay(200); // Main loop delay
}

// Function to send accident alert via Telegram
void sendAccidentAlert() {
  String message = "🚨 ACCIDENT DETECTED! 🚨\n";
  
  if (gps.location.isValid()) {
    message += "📍 Location: https://maps.google.com/?q=";
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);
    message += "\n";
  } else {
    message += "📍 Location: Unknown (No GPS signal)\n";
  }
  
  if (gps.speed.isValid()) {
    message += "🚗 Speed at impact: ";
    message += String(speedAtImpact, 2);
    message += " km/h\n";
  }
  
  message += "🕒 Time: ";
  if (gps.time.isValid()) {
    message += String(gps.time.hour());
    message += ":";
    message += String(gps.time.minute());
    message += ":";
    message += String(gps.time.second());
    message += " UTC";
  } else {
    message += "Unknown";
  }
  
  // Send the message
  bot.sendMessage(CHAT_ID, message, "");
  Serial.println("Accident alert sent via Telegram");
}

// Function to generate a random 6-digit OTP (optional)
String generateOtp() {
  String otp = "";
  for (int i = 0; i < 6; i++) {
    otp += String(rand() % 10);  // Random number between 0 and 9
  }
  return otp;
}
