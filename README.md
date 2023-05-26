# esp32TempHumCo2LCD
C code for esp32 az-delivery, temperatureHumiditySensor, Co2InfraredSensor MH-Z19C and LCD Display 16x2

## Installation

**ESP32**:
install ESP32: Arduino IDE 2.0,  
File > Preferences: scroll down to Additional boards manager URLs, post this URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json, click OK,  
und/oder?  
Tools > Board > Board Manager > suche esp32 > install esp32 by Espressif Systems  
Tools > Board >  esp32 > ESP32 Dev Module, Tools > Port > COM4,  

**TemperatureHumiditySensor**:
Tools > Manage Libraries > suche DHT > install DHT sensor library by Adafruit, install all dependencies,  
Temperature Humidity Sensor wie unter ESP32 - DHT11 Module Wiring auf dieser Seite: https://esp32io.com/tutorials/esp32-temperature-humidity-sensor?utm_content=cmp-true beschrieben mit esp32 verbinden:  
Sensor VCC -> 5V esp32,  
Sensor DATA -> G21 (GIOP21) esp32,  
Sensor GND -> GND esp32,  
ESP32 Code - DHT11 kopieren und in Arduino IDE einfügen,  
  
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
  
Upload klicken (Symbol Pfeil nach rechts, zweites von links in der zweiten Zeile von oben, alternativ unter Sketch > upload),  
Serial Monitor öffnen (Symbol Lupe ganz rechts in der zweiten Zeile von oben): hier wird nun die Luftfeuchtigkeit Humidity und die Temperatur angezeigt  
