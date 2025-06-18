#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <LiquidCrystal.h>
#include <esp_sleep.h>

// Pin definitions
#define DHTPIN 4      // Pin for DHT22 temperature and humidity sensor
#define DHTTYPE DHT22 // DHT sensor type
#define MQ135_PIN 32  // Pin for MQ135 air quality sensor
#define SOUND_PIN 34  // Pin for sound sensor
#define BUZZER_PIN 13 // Pin for buzzer
#define BUTTON_PIN 12 // Pin for button to disable buzzer and wake ESP32
#define FAN_PIN 21    // Pin for fan motor

// LCD Parallel Pin Definitions
#define RS 14 // LCD RS pin
#define E 16  // LCD Enable pin
#define D4 17 // LCD Data pin 4
#define D5 27 // LCD Data pin 5
#define D6 22 // LCD Data pin 6
#define D7 33 // LCD Data pin 7

// LEDs for each parameter
#define TEMP_LED_GREEN 2   // Green LED for temperature
#define TEMP_LED_RED 15    // Red LED for temperature
#define HUM_LED_GREEN 5    // Green LED for humidity
#define HUM_LED_RED 18     // Red LED for humidity
#define AIR_LED_GREEN 19   // Green LED for air quality
#define AIR_LED_RED 25     // Red LED for air quality
#define SOUND_LED_GREEN 26 // Green LED for sound level
#define SOUND_LED_RED 23   // Red LED for sound level

// Define MQ-135 sensor thresholds
#define MQ135_BASE 400 // Baseline sensor value (motor off)
#define MQ135_MAX 800  // Maximum sensor value (motor full speed)
#define PWM_MIN 0      // Minimum PWM (motor off)
#define PWM_MAX 255    // Maximum PWM (motor full speed)

// MQTT Configuration
const char *ssid = "WIFI_PRINTER";         // Wi-Fi SSID
const char *password = "c4nc3l477";        // Wi-Fi Password
const char *mqtt_server = "192.168.1.110"; // MQTT broker IP

WiFiClient espClient;
PubSubClient client(espClient);

// Sleep mode configuration
#define SLEEP_TIME 10e6        // Sleep time in microseconds (10 seconds)
#define INACTIVITY_THRESHOLD 5 // Number of stable readings before entering sleep

// Threshold values (can be updated remotely via MQTT)
float TEMP_THRESHOLD = 30;
float HUM_THRESHOLD = 60;
int MQ135_THRESHOLD = 550;
int SOUND_THRESHOLD = 600;

// Initialize sensors and display in 4-bit mode
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

// Global variables
float prevTemperature = -1000, prevHumidity = -1000;
int prevAirQuality = -1, prevSoundLevel = -1;
int noChangeCounter = 0;
bool buzzerDisabled = false;
unsigned long lastSwitchTime = 0; // Last time display switched
unsigned long alertStartTime = 0; // Timestamp of the alert start
bool isAlertActive = false;       // State of the alert on the LCD
bool showTempHum = true;          // Alternate display between temperature/humidity and air/sound
bool fanState = false;
bool buzzerState = false;
bool buzzerManualOverride = false; // false = auto (sensor); true = manual (MQTT)
bool fanManualOverride = false;    // false = auto; true = manual

// MQTT callback to handle threshold update messages
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    payload[length] = '\0'; // Ensure string termination
    String message = String((char *)payload);
    Serial.print("[MQTT RECEIVED] ");
    Serial.print(topic);
    Serial.print(" -> ");
    Serial.println(message);

    // Handle simple actuator commands without JSON
    if (String(topic) == "actuator/fan" || String(topic) == "actuator/buzzer")
    {
        String command = "";

        for (unsigned int i = 0; i < length; i++)
        {
            command += (char)payload[i];
        }

        command.trim(); // Remove whitespace

        bool isOn = false;

        // Check if the payload is JSON (e.g., {"state":"ON"})
        if (command.startsWith("{"))
        {
            int pos = command.indexOf(":");
            int end = command.indexOf("}");

            if (pos > 0 && end > pos)
            {
                String value = command.substring(pos + 1, end);
                value.replace("\"", ""); // Remove quotes
                value.trim();
                isOn = (value == "ON");
            }
            else
            {
                Serial.println("[ERROR] Invalid JSON for actuator command.");
                return;
            }
        }
        else
        {
            isOn = (command == "ON");
        }

        if (String(topic) == "actuator/fan")
        {
            fanManualOverride = true;
            digitalWrite(FAN_PIN, isOn ? HIGH : LOW);
            fanState = isOn;

            char fanBuffer[32];
            snprintf(fanBuffer, sizeof(fanBuffer), "{\"state\":\"%s\"}", isOn ? "ON" : "OFF");
            client.publish("status/fan", fanBuffer);
        }

        if (String(topic) == "actuator/buzzer")
        {
            buzzerManualOverride = true;
            digitalWrite(BUZZER_PIN, isOn ? HIGH : LOW);
            buzzerState = isOn;

            char buzzerBuffer[32];
            snprintf(buzzerBuffer, sizeof(buzzerBuffer), "{\"state\":\"%s\"}", isOn ? "ON" : "OFF");
            client.publish("status/buzzer", buzzerBuffer);
        }

        return;
    }

    // Extract "value" from JSON
    int valueStart = message.indexOf(":") + 1;
    int valueEnd = message.indexOf("}");
    if (valueStart > 0 && valueEnd > valueStart)
    {
        float value = message.substring(valueStart, valueEnd).toFloat();

        // Match topic and update corresponding threshold
        if (String(topic) == "config/threshold/temperature")
            TEMP_THRESHOLD = value;
        else if (String(topic) == "config/threshold/humidity")
            HUM_THRESHOLD = value;
        else if (String(topic) == "config/threshold/co2")
            MQ135_THRESHOLD = (int)value;
        else if (String(topic) == "config/threshold/noise")
            SOUND_THRESHOLD = (int)value;

        Serial.print("[THRESHOLD UPDATED] ");
        Serial.print(topic);
        Serial.print(" = ");
        Serial.println(value);
    }
    else
    {
        Serial.println("[ERROR] Invalid JSON format");
    }
}

// Function to connect to Wi-Fi
void setup_wifi()
{
    delay(10);
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

// Function to reconnect to MQTT broker
void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Connecting to MQTT broker...");
        if (client.connect("ESP32Client"))
        {
            Serial.println("Connected!");

            // 🔁 Resubscribe to topics after reconnection
            client.subscribe("config/threshold/temperature");
            client.subscribe("config/threshold/humidity");
            client.subscribe("config/threshold/co2");
            client.subscribe("config/threshold/noise");

            client.subscribe("actuator/fan");    // Subscribe to fan control topic
            client.subscribe("actuator/buzzer"); // Subscribe to buzzer control topic
        }
        else
        {
            Serial.print("Connection failed, rc=");
            Serial.print(client.state());
            Serial.println(" Trying again in 5 seconds...");
            delay(5000);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing sensors, LEDs, buzzer, button, and LCD...");

    // Config fan pin state
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW); // Ensure fan is off at startup

    // Initialize Wi-Fi and MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);

    // Set MQTT message callback function
    client.setCallback(mqttCallback);

    // Subscribe to configuration topics for threshold updates
    client.subscribe("config/threshold/temperature");
    client.subscribe("config/threshold/humidity");
    client.subscribe("config/threshold/co2");
    client.subscribe("config/threshold/noise");

    client.subscribe("actuator/fan");    // Also subscribe during setup
    client.subscribe("actuator/buzzer"); // Also subscribe during setup

    // Initialize LEDs
    pinMode(TEMP_LED_GREEN, OUTPUT);
    pinMode(TEMP_LED_RED, OUTPUT);
    pinMode(HUM_LED_GREEN, OUTPUT);
    pinMode(HUM_LED_RED, OUTPUT);
    pinMode(AIR_LED_GREEN, OUTPUT);
    pinMode(AIR_LED_RED, OUTPUT);
    pinMode(SOUND_LED_GREEN, OUTPUT);
    pinMode(SOUND_LED_RED, OUTPUT);

    // Initialize buzzer and button
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize sensors and LCD
    dht.begin();
    lcd.begin(16, 2);
    delay(1000); // Allow LCD to initialize
    lcd.print("System Ready!");
}

void loop()
{
    // Ensure MQTT connection
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // Check button to disable buzzer
    if (digitalRead(BUTTON_PIN) == LOW)
    {
        buzzerDisabled = true;
        isAlertActive = false; // Clear the alert state
        lcd.clear();           // Clear the LCD
        Serial.println("Button pressed: Buzzer disabled.");
        digitalWrite(BUZZER_PIN, LOW);
        delay(500); // Debounce delay
    }

    // Read sensor values
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int airQuality = analogRead(MQ135_PIN);
    int soundLevel = analogRead(SOUND_PIN);

    // Check for changes in sensor values
    bool hasChanged = false;
    if (abs(temperature - prevTemperature) > 0.1 ||
        abs(humidity - prevHumidity) > 0.1 ||
        abs(airQuality - prevAirQuality) > 2 ||
        abs(soundLevel - prevSoundLevel) > 2)
    {
        hasChanged = true;
    }

    // Update previous values
    prevTemperature = temperature;
    prevHumidity = humidity;
    prevAirQuality = airQuality;
    prevSoundLevel = soundLevel;

    // Enter sleep mode if no changes are detected
    if (!hasChanged)
    {
        noChangeCounter++;
        if (noChangeCounter >= INACTIVITY_THRESHOLD)
        {
            Serial.println("No variations detected. Entering sleep mode...");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Sleep Mode...");
            delay(2000);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, LOW); // Wake up on button press
            esp_sleep_enable_timer_wakeup(SLEEP_TIME);                 // Wake up after timeout
            esp_deep_sleep_start();                                    // Enter deep sleep
        }
    }
    else
    {
        noChangeCounter = 0;
    }

    // Control LEDs based on sensor values
    digitalWrite(TEMP_LED_GREEN, temperature <= TEMP_THRESHOLD);
    digitalWrite(TEMP_LED_RED, temperature > TEMP_THRESHOLD);
    digitalWrite(HUM_LED_GREEN, humidity <= HUM_THRESHOLD);
    digitalWrite(HUM_LED_RED, humidity > HUM_THRESHOLD);
    digitalWrite(AIR_LED_GREEN, airQuality <= MQ135_THRESHOLD);
    digitalWrite(AIR_LED_RED, airQuality > MQ135_THRESHOLD);
    digitalWrite(SOUND_LED_GREEN, soundLevel <= SOUND_THRESHOLD);
    digitalWrite(SOUND_LED_RED, soundLevel > SOUND_THRESHOLD);

    // Construct a JSON object with the current LED states
    String ledStates = "{";
    ledStates += "\"temperature\":\"" + String(temperature > TEMP_THRESHOLD ? "RED" : "GREEN") + "\",";
    ledStates += "\"humidity\":\"" + String(humidity > HUM_THRESHOLD ? "RED" : "GREEN") + "\",";
    ledStates += "\"co2\":\"" + String(airQuality > MQ135_THRESHOLD ? "RED" : "GREEN") + "\",";
    ledStates += "\"noise\":\"" + String(soundLevel > SOUND_THRESHOLD ? "RED" : "GREEN") + "\"";
    ledStates += "}";

    // Publish the LED states to the 'status/leds' topic as a single JSON message
    client.publish("status/leds", ledStates.c_str());

    // Determine the current state of the buzzer (HIGH = ON, LOW = OFF)
    String buzzerState = digitalRead(BUZZER_PIN) == HIGH ? "ON" : "OFF";

    // Create a JSON payload with the buzzer state
    String buzzerPayload = "{\"state\":\"" + buzzerState + "\"}";

    // Publish the buzzer state to the 'status/buzzer' topic
    client.publish("status/buzzer", buzzerPayload.c_str());

    // Control buzzer and alert for air quality
    if (!buzzerManualOverride)
    {
        if (airQuality > MQ135_THRESHOLD)
        {
            if (!buzzerDisabled)
            {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerState = true;
                Serial.println("ALERT! Poor air quality detected.");
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("ALERT!");
                lcd.setCursor(0, 1);
                lcd.print("Poor Air Quality");
                isAlertActive = true;
                alertStartTime = millis();
            }
        }
        else
        {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerState = false;
            buzzerDisabled = false;
            if (isAlertActive && millis() - alertStartTime >= 3000)
            {
                isAlertActive = false;
                lcd.clear();
            }
        }
    }

    // Control LCD display switching every 2 seconds
    if (!isAlertActive)
    {
        if (millis() - lastSwitchTime >= 2000)
        {                               // Switch display every 2 seconds
            lastSwitchTime = millis();  // Update the last switch time
            showTempHum = !showTempHum; // Toggle display state
        }

        if (showTempHum)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Temp: ");
            lcd.print(temperature);
            lcd.print("C");
            lcd.setCursor(0, 1);
            lcd.print("Humidity: ");
            lcd.print(humidity);
        }
        else
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Air: ");
            lcd.print(airQuality);
            lcd.setCursor(0, 1);
            lcd.print("Sound: ");
            lcd.print(soundLevel);
        }
    }

    // Turn the fan ON if air quality is above the threshold, otherwise turn it OFF
    if (!fanManualOverride)
    {
        fanState = airQuality > MQ135_THRESHOLD;
    }

    digitalWrite(FAN_PIN, fanState);

    // Create a JSON payload and publish fan state to MQTT
    String fanPayload = "{\"state\":\"" + String(fanState ? "ON" : "OFF") + "\"}";
    client.publish("status/fan", fanPayload.c_str());

    // Build a JSON object containing the configured thresholds for each parameter
    String thresholdsPayload = "{";
    thresholdsPayload += "\"temperature\":" + String(TEMP_THRESHOLD) + ",";
    thresholdsPayload += "\"humidity\":" + String(HUM_THRESHOLD) + ",";
    thresholdsPayload += "\"co2\":" + String(MQ135_THRESHOLD) + ",";
    thresholdsPayload += "\"noise\":" + String(SOUND_THRESHOLD);
    thresholdsPayload += "}";

    // Publish the threshold values to the 'status/thresholds' topic
    client.publish("status/thresholds", thresholdsPayload.c_str());

    // Publish sensor values to MQTT broker
    char tempPayload[32];
    snprintf(tempPayload, sizeof(tempPayload), "{\"value\":%.1f}", temperature);
    client.publish("sensors/temperature", tempPayload);

    char humPayload[32];
    snprintf(humPayload, sizeof(humPayload), "{\"value\":%.1f}", humidity);
    client.publish("sensors/humidity", humPayload);

    char airPayload[32];
    snprintf(airPayload, sizeof(airPayload), "{\"value\":%d}", airQuality);
    client.publish("sensors/co2", airPayload);

    char soundPayload[32];
    snprintf(soundPayload, sizeof(soundPayload), "{\"value\":%d}", soundLevel);
    client.publish("sensors/noise", soundPayload);

    // Debug output to Serial Monitor
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Air Quality: ");
    Serial.println(airQuality);
    Serial.print("Sound Level: ");
    Serial.println(soundLevel);

    delay(1000); // Delay between readings
}
