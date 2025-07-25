/*
  - sw che carico offline per determinare l'indirizzo di un singolo sensore
  - sempre non bloccante per leggere la temperatura di un singolo sensore
*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

#define SERIAL_SPEED 9600
#define ONE_WIRE_BUS_EXTERNAL_TEMPERATURE 7
#define TEMPERATURE_PRECISION 9

#if !defined(DEVICE_DISCONNECTED)
#define DEVICE_DISCONNECTED -127
#endif

#define DEVICE_ERROR 85

/* TEMPERATURES SECTION */
OneWire oneWire(ONE_WIRE_BUS_EXTERNAL_TEMPERATURE);
DallasTemperature sensor(&oneWire);
DeviceAddress Thermometer;
unsigned long conversionTime_DS18B20_sensor; //ms
unsigned long lastTempRequest;
byte numberOfDevices;

unsigned int deviceDisconnected;
unsigned int deviceError;

float temperature; 
float marginFactor = 1.2; // fattore moltiplicativo per aspettare un po' più di delay.
bool gotTemperatures;

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
// Define a char array to store the hexadecimal representation of the address
char addressCharArray[17]; // 16 characters for the address + 1 for null terminator

unsigned long startGetTemperatures, endGetTemperatures;
/* END TEMPERATURES SECTION */

/* RELAY FRIDE SECTION */
unsigned int fridgeFanPin = 6;
unsigned int fridgeRelayPin = 12;

float setPointTemperature = 26.0;
float lowerLimit = 24.0;
float upperLimit = 28.0;

int hysteresisCycleState = 0; // 0 = fridge not active 1 = fridge active

void setup() {
  pinMode(fridgeFanPin, OUTPUT);
  pinMode(fridgeRelayPin, OUTPUT);
  digitalWrite(fridgeRelayPin, true); // deactivating fridge
  Serial.begin(SERIAL_SPEED);

  sensor.begin();
  numberOfDevices = sensor.getDeviceCount();

  sensor.setWaitForConversion(false); // quando richiedi le temperature requestTemperatures() la libreria NON aspetta il delay adeguato, quidni devi aspettarlo tu.
  sensor.requestTemperatures(); // send command to all the sensor for temperature conversion.
  lastTempRequest = millis(); 
  conversionTime_DS18B20_sensor = 750 / (1 << (12 - TEMPERATURE_PRECISION));  // res in {9,10,11,12}

  if(sensor.getAddress(tempDeviceAddress, 0)){
    addressToCharArray(tempDeviceAddress, addressCharArray); // indirizzo convertito
    Serial.print("ADDRESS: ");
    Serial.println(addressCharArray);
    sensor.getAddress(Thermometer, 0);
    deviceDisconnected = 0;
    deviceError = 0;
    delay(5);
  }
  delay(5);
}

void loop() {
  if(millis() - lastTempRequest >= (conversionTime_DS18B20_sensor * marginFactor)){
    startGetTemperatures = millis(); // per calcolare quanto tempo impiego a fetchare tutte le temperature dai sensori.
    /* Memorize all temperatures in an ordered array */
    temperature = sensor.getTempC(Thermometer);

    if(temperature <= DEVICE_DISCONNECTED){
      deviceDisconnected ++;
    }
    if(temperature >= DEVICE_ERROR){
      deviceError ++;
    }

    delay(1);       
    gotTemperatures = true;
  
    sensor.requestTemperatures();
    lastTempRequest = millis();
    endGetTemperatures = millis();
  } 

  if(gotTemperatures){
    if(hysteresisCycleState == 0 && temperature >= upperLimit){
      hysteresisCycleState = 1; // frdge active
      digitalWrite(fridgeFanPin, true);
      digitalWrite(fridgeRelayPin, false);
    }

    if(hysteresisCycleState == 1 && temperature <= lowerLimit){
      hysteresisCycleState = 0; //deactivating fridge
      digitalWrite(fridgeFanPin, false);
      digitalWrite(fridgeRelayPin, true);
    }

    gotTemperatures = false;
    Serial.print("Address: ");
    Serial.print(addressCharArray);
    Serial.print(" T:");
    Serial.print(temperature);
    Serial.print("°C");
    Serial.print(" Disc:");
    Serial.print(deviceDisconnected);
    Serial.print(" Error:");
    Serial.print(deviceError);
    Serial.print(" ReadingTime:");
    Serial.print(endGetTemperatures - startGetTemperatures);
    Serial.print("ms ");
    Serial.print("Fridge State:");
    Serial.print(hysteresisCycleState);
    Serial.println();

    





  }

}

// Function to convert a byte to its hexadecimal representation
void byteToHex(uint8_t byteValue, char *hexValue) {
  uint8_t highNibble = byteValue >> 4;
  uint8_t lowNibble = byteValue & 0x0F;
  hexValue[0] = highNibble < 10 ? '0' + highNibble : 'A' + (highNibble - 10);
  hexValue[1] = lowNibble < 10 ? '0' + lowNibble : 'A' + (lowNibble - 10);
}

// Function to print a device address
void addressToCharArray(DeviceAddress deviceAddress, char *charArray) {
  for (uint8_t i = 0; i < 8; i++) {
    byteToHex(deviceAddress[i], &charArray[i * 2]);
  }
  // Add null terminator at the end of the char array
  charArray[16] = '\0';
}
