# esp32TempHumCo2LCD
C code for esp32 az-delivery, temperatureHumiditySensor, and LCD Display 16x2 and Co2InfraredSensor MH-Z19C

## Installation

**ESP32**:
install ESP32: Arduino IDE 2.0,  
File > Preferences: scroll down to Additional boards manager URLs, post this URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json, click OK, und/oder? Tools > Board > Board Manager > suche esp32 > install esp32 by Espressif Systems  
Tools > Board >  esp32 > ESP32 Dev Module, Tools > Port > COM4,  

**TemperatureHumiditySensor**:  
Tools > Manage Libraries > suche DHT > install DHT sensor library by Adafruit, install all dependencies,  
Temperature Humidity Sensor wie unter ESP32 - DHT11 Module Wiring auf dieser Seite: https://esp32io.com/tutorials/esp32-temperature-humidity-sensor?utm_content=cmp-true beschrieben mit esp32 verbinden:  
Sensor VCC -> 5V esp32,  
Sensor DATA -> G21 (GIOP21) esp32,  
Sensor GND -> GND esp32,  
ESP32 Code - DHT11 kopieren und in Arduino IDE einfügen,  
```c
/*  
 * This ESP32 code is created by esp32io.com  
 *  
 * This ESP32 code is released in the public domain  
 *  
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-temperature-humidity-sensor  
 */  
  
#include <DHT.h>  
#define DHT_SENSOR_PIN  21 // ESP32 pin GIOP21 connected to DHT11 sensor  
#define DHT_SENSOR_TYPE DHT11  
  
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);  
  
void setup() {  
  Serial.begin(9600);  
  dht_sensor.begin(); // initialize the DHT sensor  
}  
  
void loop() {  
  // read humidity  
  float humi  = dht_sensor.readHumidity();  
  // read temperature in Celsius  
  float tempC = dht_sensor.readTemperature();  
  // read temperature in Fahrenheit  
  float tempF = dht_sensor.readTemperature(true);  
    
  // check whether the reading is successful or not  
  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {  
    Serial.println("Failed to read from DHT sensor!");  
  } else {  
    Serial.print("Humidity: ");  
    Serial.print(humi);  
    Serial.print("%");  
      
    Serial.print("  |  ");  
      
    Serial.print("Temperature: ");  
    Serial.print(tempC);  
    Serial.print("°C  ~  ");  
    Serial.print(tempF);  
    Serial.println("°F");  
  }  
    
  // wait a 2 seconds between readings  
  delay(2000);  
}  
```  
Upload klicken (Symbol Pfeil nach rechts, zweites von links in der zweiten Zeile von oben, alternativ unter Sketch > upload),  
Serial Monitor öffnen (Symbol Lupe ganz rechts in der zweiten Zeile von oben): hier wird nun die Luftfeuchtigkeit Humidity und die Temperatur angezeigt  

**LCD Display 16x2:**  
Protentiometer (10K) erforderlich  
File > Preferences: scroll down to Additional boards manager URLs, post this URL: https://github.com/arduino-libraries/LiquidCrystal/blob/master/src/LiquidCrystal.h, click OK, und/oder? Tools > Manage Libraries > suche LiquidCrystal > install LiquidCrystal by Arduino, install all dependencies,  
LCD 16x2 Display wie unter ESP32 - DHT11 Module Wiring auf dieser Seite: https://www.circuitschools.com/interfacing-16x2-lcd-module-with-esp32-with-and-without-i2c/ beschrieben mit Protentiometer und esp32 verbinden:  
LCD 1.Pin VSS -> GND esp32,  
LCD 2.Pin VDD -> 5V esp32,  
LCD 3.Pin V0 -> 2.Pin Potentiometer,  
LCD 4.Pin VDD -> 5V esp32,  
LCD 5.Pin R / W -> GND esp32,  
LCD 6.Pin E -> GPIO23 esp32,  
LCD 11.Pin DB4 -> GPIO18 esp32,  
LCD 12.Pin DB5 -> GPIO17 esp32,  
LCD 13.Pin DB6 -> GIOP16 esp32,  
LCD 14.Pin DB7 -> GIO15 esp32,  
LCD 15.Pin LED+ -> 5V esp32,  
LCD 16.Pin LED- -> GND esp32,  
1.Pin Popentiometer -> 5 V esp32,  
3.Pin Popentiometer -> GND esp32,  
Interfacing 16X2 LCD Module with ESP32 with and without using I2C adapter Program code kopieren und in Arduino IDE einfügen,  
```c
// include the library code:
#include <LiquidCrystal.h>
 
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
 
void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("circuitschools.");
}
 
void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
}
```
Upload klicken (Symbol Pfeil nach rechts, zweites von links in der zweiten Zeile von oben, alternativ unter Sketch > upload),  
Auf dem Display sollte nun in der ersten Zeile der Schriftzug: circuitschools. Und in der zweiten ein Sekundenzähler zu sehen sein. 

**TemperatureHumiditySensor und LCD Display 16x2 verbinden:**
```c
// include the library code:
#include <DHT.h>
#include <LiquidCrystal.h> 

#define DHT_SENSOR_PIN  21 // ESP32 pin GIOP21 connected to DHT11 sensor
#define DHT_SENSOR_TYPE DHT11

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("circuitschools.");

  Serial.begin(9600);
  dht_sensor.begin(); // initialize the DHT sensor
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 1);

  // read humidity
  float humi  = dht_sensor.readHumidity();
  // read temperature in Celsius
  float tempC = dht_sensor.readTemperature();
  // read temperature in Fahrenheit
  float tempF = dht_sensor.readTemperature(true);

  // check whether the reading is successful or not
  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.print("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");

    lcd.setCursor(0, 0);
    
    lcd.print("Humidity: ");
    lcd.print(humi);
    lcd.print("%");

    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);

    lcd.print(tempC);
    lcd.print(" C  ");
    lcd.print(tempF);
    lcd.print(" F");
  }

  // wait a 2 seconds between readings
  delay(2000);
}
```

**Infrarot CO2 Sensor MH-Z19C:**  
Tools > Manage Libraries > suche MH-Z19 > install MH-Z CO2 Sensors by Tobias Schürg  
folgendes nicht in befolgter Anleitung: https://www.souichi.club/en/m5stack/co2sensor-mhz19c/ enthalten. Doch als ich den Code (der noch folgt) uploadete kam eine Fehlermeldung, dass SoftwareSerial.h nicht gefunden werden konnte, was ich folgendermaßen behob:  
Tools > Manage Libraries > suche SoftwareSerial > install EspSoftwareSerial by Dirk Kaar, Peter Lerup  
Möglicherweise ist dies nicht in jedem Fall notwendig, sodass anzumerken ist, dass dieser Schritt zunächst übersprungen werden könnte um zunächst seine Notwendigkeit zu prüfen.  
Infrarot CO2 Sensor MH-Z19C wie unter PWN Connection, Wiring Diagram in Anleitung mit esp32 verbinden:  
MH-Z19C Vin -> 5V esp32,  
LCD 2.Pin GND -> GND esp32,  
MH-Z19C PWM → GIOP 13  
Sketch (PWM) kopieren, Teile des I2C LCD weglassen und in Arduino IDE einfügen,  
```c
/*
 * Display the measured value of Co2 Sensor (MH-Z19C)
 * Co2 Sensor:GPIO13
 */
#include <MHZ.h> // Co2 Sensor

#define co2pwmPin 13 // GPIO13
 
MHZ co2(co2pwmPin, MHZ19C);

void setup() {
    Serial.begin(9600);
    Serial.println("Program Start");
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
    int ppm_pwm = co2.readCO2PWM();
    Serial.println("PPM = " + String(ppm_pwm));
}
```
Upload klicken (Symbol Pfeil nach rechts, zweites von links in der zweiten Zeile von oben, alternativ unter Sketch > upload),  
Serial Monitor öffnen (Symbol Lupe ganz rechts in der zweiten Zeile von oben): hier wird nun nach kurzer Aufwärmzeit die CO2 Konzentration in PPM (Parts per million, Teilchen je Million) angezeigt  

**Infrarot CO2 Sensor MH-Z19C mit vorherigen Teilen verbinden:**  
```c
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
```
