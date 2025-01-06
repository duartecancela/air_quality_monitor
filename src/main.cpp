#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal.h>

// Pin definitions
#define DHTPIN 4
#define DHTTYPE DHT22
#define MQ135_PIN 32
#define SOUND_PIN 34
#define BUZZER_PIN 13   // Buzzer pin
#define BUTTON_PIN 12   // Button pin

// LCD Parallel Pin Definitions
#define RS 14
#define E 16
#define D4 17
#define D5 27
#define D6 22
#define D7 33

// LEDs for each parameter
#define TEMP_LED_GREEN 2
#define TEMP_LED_RED 15
#define HUM_LED_GREEN 5
#define HUM_LED_RED 18
#define AIR_LED_GREEN 19
#define AIR_LED_RED 25
#define SOUND_LED_GREEN 26
#define SOUND_LED_RED 23

// Sleep mode configuration
#define SLEEP_TIME 10e6  // Deep sleep time in microseconds (10 seconds)
#define INACTIVITY_THRESHOLD 5  // Number of consecutive readings without variation
#define MQ135_THRESHOLD 79  // Critical threshold to trigger the buzzer
#define TEMP_THRESHOLD 22    // Temperature threshold for LED activation
#define HUM_THRESHOLD 55     // Humidity threshold for LED activation
#define SOUND_THRESHOLD 2810  // Sound level threshold for LED activation

// Initialize sensors and display in 4-bit mode
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

void setup() {

    dht.begin();
    lcd.begin(16, 2);
    lcd.print("System Ready!");

    Serial.begin(115200);
    Serial.println("Iniciando ESP32...");

    pinMode(TEMP_LED_GREEN, OUTPUT);
    pinMode(TEMP_LED_RED, OUTPUT);
    pinMode(HUM_LED_GREEN, OUTPUT);
    pinMode(HUM_LED_RED, OUTPUT);
    pinMode(AIR_LED_GREEN, OUTPUT);
    pinMode(AIR_LED_RED, OUTPUT);
    pinMode(SOUND_LED_GREEN, OUTPUT);
    pinMode(SOUND_LED_RED, OUTPUT);
}

void loop() {

  // Read sensor data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int airQuality = analogRead(MQ135_PIN);
  int soundLevel = analogRead(SOUND_PIN);

  
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);

  // Debug output to Serial Monitor
  Serial.println("=== Sensor Values ===");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Air Quality: ");
  Serial.println(airQuality);
  Serial.print("Sound Level: ");
  Serial.print(soundLevel);
  Serial.println(" (analog value)");

  // LED control for temperature
  digitalWrite(TEMP_LED_GREEN, temperature <= TEMP_THRESHOLD);
  digitalWrite(TEMP_LED_RED, temperature > TEMP_THRESHOLD);

  // LED control for humidity
  digitalWrite(HUM_LED_GREEN, humidity <= HUM_THRESHOLD);
  digitalWrite(HUM_LED_RED, humidity > HUM_THRESHOLD);

  // LED control for air quality
  digitalWrite(AIR_LED_GREEN, airQuality <= MQ135_THRESHOLD);
  digitalWrite(AIR_LED_RED, airQuality > MQ135_THRESHOLD);

  // LED control for sound level
  digitalWrite(SOUND_LED_GREEN, soundLevel <= SOUND_THRESHOLD);
  digitalWrite(SOUND_LED_RED, soundLevel > SOUND_THRESHOLD);

  delay(500);
}
