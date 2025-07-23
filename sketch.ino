// ---------------------------------------------------------------- //
//                  ESP32 Multi-Sensor IoT Project                  //
// ---------------------------------------------------------------- //
// Libraries for WiFi, MQTT, Sensors, and Display
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ------------------- WiFi & MQTT Configuration ------------------ //
// Replace with your WiFi credentials if not using Wokwi's default
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT Broker settings
const char* MQTT_BROKER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "mqttx_044ec2b3"; // Make this unique

// MQTT Topics
const char* LIGHT_TOPIC = "esp32/sensors/luminance";
const char* TEMP_HUMID1_TOPIC = "esp32/sensors/humidityAndTemperatureOfAir";
const char* HUMIDITY2_TOPIC = "esp32/sensors/humidityOfEarth";
const char* POT_TOPIC = "esp32/sensors/phOfEarth";
const char* SERVO_TOPIC = "esp32/actuator/servo";

// ----------------------- Pin Definitions ------------------------ //
// Sensor Pins
const int PHOTO_PIN = 34;      // Photoresistor Analog Out
const int POT_PIN = 35;        // Potentiometer Signal
const int DHT1_PIN = 19;       // DHT22 #1 Data Pin
const int DHT2_PIN = 23;       // DHT22 #2 Data Pin

// Actuator Pins
const int RED_LED_PIN = 17;    // Light level indicator
const int GREEN_LED_PIN = 18;  // Temperature indicator
const int BLUE_LED_PIN = 27;    // Humidity #1 indicator
const int PINK_LED_PIN = 14;  // Humidity #2 indicator
const int YELLOW_LED_PIN = 26; // Potentiometer indicator
const int SERVO_PIN = 13;      // Servo motor signal
const int HORDEN_PIN = 16;
const int LIGHT_PIN = 15;

// -------------------- Threshold Definitions --------------------- //
// Adjust these values to change the trigger points
const int LIGHT_UPPER_THRESHOLD = 183;       // For photoresistor in lux (8k lux = 183)
const int LIGHT_UNDER_THRESHOLD = 156;
const float TEMP_UPPER_THRESHOLD = 26.0;      // For air temperature in Celsius
const float TEMP_UNDER_THRESHOLD = 18.0;
const float HUMIDITY1_UPPER_THRESHOLD = 85.0; // For air humidity in %
const float HUMIDITY1_UNDER_THRESHOLD = 65.0;
const float HUMIDITY2_UPPER_THRESHOLD = 80.0; // For soil moisture in %
const float HUMIDITY2_UNDER_THRESHOLD = 60.0;
const int POT_UPPER_THRESHOLD = 2047;         // For potentiometer acting as soil pH sensor (pH 7 = 2047)
const int POT_UNDER_THRESHOLD = 1755; 
const float GAMMA = 0.7;
const float RL_10 = 50;

// -------------------- Global Objects & Variables ---------------- //
// WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// OLED Display (128x64)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DHT Sensors
#define DHTTYPE DHT22
DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);

// Servo Motor
Servo myServo;

// Variables to hold sensor readings
int lightValue = 0;
float temp1 = 0;
float humidity1 = 0;
String temp_humid1 = "";
float humidity2 = 0;
int potValue = 0;
String servoState = "OFF";

// Variables for timing and display cycling
unsigned long lastReadTime = 0;
const long readInterval = 2000; // Read sensors every 2 seconds
int displayState = 0; // To cycle through display data

// -------------------------- SETUP ------------------------------- //
void setup() {
    Serial.begin(115200);

    // Initialize Pins
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(PINK_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(HORDEN_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);

    // Initialize Servo
    myServo.attach(SERVO_PIN);
    myServo.write(0); // Start at 0 degrees

    // Initialize OLED Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();

    // Initialize DHT sensors
    dht1.begin();
    dht2.begin();

    // Connect to WiFi and MQTT
    connectWiFi();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

// --------------------------- LOOP ------------------------------- //
void loop() {
    // Ensure MQTT client is connected
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();

    // Read sensors and update logic at a fixed interval
    if (millis() - lastReadTime > readInterval) {
        readSensors();
        updateLogic();
        publishData();
        updateDisplay();
        lastReadTime = millis();
    }
}

// ------------------- Core Functions ------------------- //

void readSensors() {
    lightValue = analogRead(PHOTO_PIN);
    potValue = analogRead(POT_PIN);

    // Reading from DHT sensors can take a moment
    temp1 = dht1.readTemperature();
    humidity1 = dht1.readHumidity();
    humidity2 = dht2.readHumidity();

    // Check if any reads failed and exit early (to try again).
    if (isnan(temp1) || isnan(humidity1) || isnan(humidity2)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
}

void updateLogic() {
    // 1. Photoresistor -> Red LED
    digitalWrite(RED_LED_PIN, lightValue >= LIGHT_UNDER_THRESHOLD && lightValue <= LIGHT_UPPER_THRESHOLD ? LOW : HIGH);

    // 2. Temperature (DHT1) -> Green LED
    digitalWrite(GREEN_LED_PIN, temp1 >= TEMP_UNDER_THRESHOLD && temp1 <= TEMP_UPPER_THRESHOLD ? LOW : HIGH);

    // 3. Humidity (DHT1) -> Blue LED
    digitalWrite(BLUE_LED_PIN, humidity1 >= HUMIDITY1_UNDER_THRESHOLD && humidity1 <= HUMIDITY1_UPPER_THRESHOLD ? LOW : HIGH);

    // 4. Humidity (DHT2) -> PINK LED
    digitalWrite(PINK_LED_PIN, humidity2 >= HUMIDITY2_UNDER_THRESHOLD && humidity2 <= HUMIDITY2_UPPER_THRESHOLD ? LOW : HIGH);

    // 5. Potentiometer -> Yellow LED
    digitalWrite(YELLOW_LED_PIN, potValue >= POT_UNDER_THRESHOLD && potValue <= POT_UPPER_THRESHOLD ? LOW : HIGH);

    digitalWrite(LIGHT_PIN, lightValue > LIGHT_UPPER_THRESHOLD ? HIGH : LOW);
    digitalWrite(HORDEN_PIN, lightValue < LIGHT_UNDER_THRESHOLD ? HIGH : LOW);

    // 7. Humidity (DHT2) -> Servo
    if (humidity2 <= HUMIDITY2_UNDER_THRESHOLD) {
        myServo.write(90); // Move servo to 90 degrees
        servoState = "ON";
    } else if (humidity2 >= HUMIDITY2_UPPER_THRESHOLD){
        myServo.write(0);  // Move servo back to 0
        servoState = "OFF";
    }
}

void publishData() {
    temp_humid1 = String(humidity1) + "," + String(temp1);
    float voltage = lightValue / 4095.0 * 3.3;
    float resistance = 2000.0 * voltage / (3.3 - voltage);
    float lux = pow(RL_10 * 1000.0 * pow(10, GAMMA) / resistance, (1.0 / GAMMA))/10;
    float pH = (potValue + 1) / 292.5714;
    
    // Publish all sensor data to their respective MQTT topics
    mqttClient.publish(LIGHT_TOPIC, String(lux).c_str(), true);
    mqttClient.publish(TEMP_HUMID1_TOPIC, temp_humid1.c_str(), true);
    mqttClient.publish(HUMIDITY2_TOPIC, String(humidity2).c_str(), true);
    mqttClient.publish(POT_TOPIC, String(pH).c_str(), true);
    mqttClient.publish(SERVO_TOPIC, servoState.c_str(), true);
    Serial.println("Data published to MQTT.");
}

void updateDisplay() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);

    // Cycle through which sensor data to show
    switch (displayState) {
        case 0:
            display.println(F("Light"));
            display.println(lightValue);
            break;
        case 1:
            display.println(F("Temp C"));
            display.println(temp1);
            break;
        case 2:
            display.println(F("Humid 1 %"));
            display.println(humidity1);
            break;
        case 3:
            display.println(F("Humid 2 %"));
            display.println(humidity2);
            break;
        case 4:
            display.println(F("Potentiom."));
            display.println(potValue);
            break;
        case 5:
            display.println(F("Servo"));
            display.println(servoState);
            break;
    }
    
    display.display();

    // Increment state for the next cycle
    displayState++;
    if (displayState > 5) {
        displayState = 0;
    }
}


// ------------------- Helper Functions ------------------- //

void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Connecting WiFi...");
    display.display();
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    display.println("WiFi Connected!");
    display.display();
    delay(1000);
}

void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Connecting MQTT...");
        display.display();

        if (mqttClient.connect(MQTT_CLIENT_ID)) {
            Serial.println("connected");
            display.println("MQTT Connected!");
            display.display();
            delay(1000);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            display.println("MQTT Failed!");
            display.display();
            delay(5000);
        }
    }
}
