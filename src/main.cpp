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

// Define motor PWM parameters
#define MOTOR_PWM_PIN 21 // PWM pin for motor (ENA on L298N)
#define PWM_CHANNEL 0    // PWM channel (0-15)
#define PWM_FREQ 3000    // PWM frequency (optimized for L298N)
#define PWM_RESOLUTION 8 // PWM resolution (8 bits = 0 to 255)

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
#define MQ135_THRESHOLD 550    // Threshold for poor air quality
#define TEMP_THRESHOLD 30      // Temperature threshold for LED indication
#define HUM_THRESHOLD 60       // Humidity threshold for LED indication
#define SOUND_THRESHOLD 600    // Sound level threshold for LED indication

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

    // Configure the PWM channel
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);

    // Initialize Wi-Fi and MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);

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

    // Control buzzer and alert for air quality
    if (airQuality > MQ135_THRESHOLD)
    {
        if (!buzzerDisabled)
        {
            digitalWrite(BUZZER_PIN, HIGH); // Activate the buzzer
            Serial.println("ALERT! Poor air quality detected.");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("ALERT!");
            lcd.setCursor(0, 1);
            lcd.print("Poor Air Quality");
            isAlertActive = true;
            alertStartTime = millis(); // Record when the alert started
        }
    }
    else
    {
        digitalWrite(BUZZER_PIN, LOW); // Deactivate the buzzer
        buzzerDisabled = false;        // Reset buzzer state when air quality is safe
        if (isAlertActive && millis() - alertStartTime >= 3000)
        {                          // Alert lasts 3 seconds
            isAlertActive = false; // End the alert
            lcd.clear();           // Clear the LCD for normal display
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

    // Map the sensor reading to the PWM range
    int pwmValue = map(airQuality, MQ135_BASE, MQ135_MAX, PWM_MIN, PWM_MAX);
    pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX); // Ensure value is within 0-255

    // Apply PWM to the motor
    ledcWrite(PWM_CHANNEL, pwmValue);

    // Publish sensor values to MQTT broker
    char message[150]; // Increase buffer size to accommodate new data
    sprintf(message, "{\"temperature\":%.1f,\"humidity\":%.1f,\"airQuality\":%d,\"soundLevel\":%d,\"pwm\":%d}",
            temperature, humidity, airQuality, soundLevel, pwmValue);
    client.publish("sensor/values", message);

    // Debug output to Serial Monitor
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Air Quality: ");
    Serial.println(airQuality);
    Serial.print("Sound Level: ");
    Serial.println(soundLevel);
    Serial.print("PWM Output: ");
    Serial.println(pwmValue);

    delay(1000); // Delay between readings
}
