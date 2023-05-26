// include the library code:
#include <DHT.h>
#include <MHZ.h> // Co2 Sensor
#include <LiquidCrystal.h> 

#define DHT_SENSOR_PIN  21 // ESP32 pin GIOP21 connected to DHT11 sensor
#define DHT_SENSOR_TYPE DHT11
#define co2pwmPin 13 // GPIO13

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

MHZ co2(co2pwmPin, MHZ19C);

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("circuitschools.");

  Serial.begin(9600);
  dht_sensor.begin(); // initialize the DHT sensor

  pinMode(co2pwmPin, INPUT);
    delay(100);
    if (co2.isPreHeating()) {
        Serial.println("Preheating");
        while (co2.isPreHeating()) {
            Serial.print(".");
            delay(5000);
        }
        Serial.println();
    }
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 1);

  // read humidity
  float humi  = dht_sensor.readHumidity();
  // read temperature in Celsius
  float tempC = dht_sensor.readTemperature();
  // read temperature in Fahrenheit
  float tempF = dht_sensor.readTemperature(true);

  int ppm_pwm = co2.readCO2PWM();

  // check whether the reading is successful or not
  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.print("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.print("°F");

    Serial.print("  |  ");

    Serial.print("CO2: ");
    Serial.print(String(ppm_pwm));
    Serial.println("PPM");

    lcd.setCursor(0, 0);
    
    lcd.print(humi);
    lcd.print(" % ");
    lcd.print(String(ppm_pwm));
    lcd.print(" PPM");
    

    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);

    lcd.print(tempC);
    lcd.print(" C  ");
    lcd.print(tempF);
    lcd.print(" F");
  }

  // wait a 2 seconds between readings
  delay(2000);
}