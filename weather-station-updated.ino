#include <SPI.h>
#include <WiFi.h>
#include <avr/eeprom.h>

#include <DHT.h>
#include <Adafruit_BMP085.h>

#define BUFFER_SIZE 0x100

#define WIFI_SSID_EEPROM_ADDR (void *) 0x000
#define WIFI_PASS_EEPROM_ADDR (void *) 0x100

#define WIFI_SECURITY_WPA1 2
#define WIFI_SECURITY_WPA2 4
#define WIFI_SECURITY_NONE 7

IPAddress WSServer(217,160,149,219);
WiFiClient client = WiFiClient();

Adafruit_BMP085 bmp;

#define DHT_DATA_PIN 5

#define AL_Sensor1 9
#define AL_Sensor2 11
#define AL_Sensor3 13
#define AL_Sensor4 10
#define AL_Sensor5 12

// analog pins
#define GAS_SENSOR_ANALOG_PIN 0
#define AMBIENT_LIGHT_SENSOR_ANALOG_PIN 1

DHT dht(DHT_DATA_PIN, DHT12);

char* HTTP_REQUEST = "[{\"model\":\"WeatherStation.measurements\",\"fields\":{\"temp\":XX.X,\"humidity\":XX.X,\"wind_speed\":XX.X,\"wind_dir\":\"XXX\",\"pressure\":XX.X,\"smoke\":X,\"ambient\":XX.X}}]";

const int directions[32] = {
  -1, 0, 11, 68, 90,
  124, 135, 146, 158, 169,
  180, 191, 203, 23, 349, 214, 225,
  337, 34, 236, 248,
  326, 270, 281, 315,
  113, 101, 56, 45,
  304, 293, -1
};

const int directionThreshold = 70;

int readGrayCode() {
  int value = 0;

  value <<= 1;
  if (analogRead(AL_Sensor1) > directionThreshold) value |= 1;
  value <<= 1;
  if (analogRead(AL_Sensor2) > directionThreshold) value |= 1;
  value <<= 1;
  if (analogRead(AL_Sensor3) > directionThreshold) value |= 1;
  value <<= 1;
  if (analogRead(AL_Sensor4) > directionThreshold) value |= 1;
  value <<= 1;
  if (analogRead(AL_Sensor5) > directionThreshold) value |= 1;

  return value;
}

int grayToBinary(int gray) {
  int binary = gray;
  for (int mask = gray >> 1; mask != 0; mask >>= 1) {
    binary ^= mask;
  }
  return binary;
}

struct MeasurementData{
  float temp;
  float humidity;
  float wind_speed;
  String wind_dir;
  float pressure;
  float smoke;
  float ambient;
} data;

const int windPin = A15;
const int windThreshold = 65;

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
unsigned long lastPulseCount = 0;
bool wasBelowThreshold = false;

void setup() {

  // initialize serial:

  Serial.begin(9600);

  while(!Serial) ;

  Serial.setTimeout(60000);
  Serial.println("Checkpoint 1: Starting setup");

  Serial.println("Initializing Wifi...");

  printMacAddress();

  Serial.println("Checkpoint 2: Printed MAC");

  // initialize dht temperature and humidity sensor
  dht.begin();
  Serial.println("Checkpoint 3: DHT started");

  // initialize BMP pressure sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  Serial.println("Checkpoint 4: BMP started");


  // check wifi shield firmware version
  String firmware = WiFi.firmwareVersion();
  if(firmware != "1.1.0"){
    Serial.print("Firmware is at version ");
    Serial.print(firmware);
    Serial.print(". Please update the firmware.");
    while(true){}
  }
  else{
    Serial.print("WiFi Shield Firmware version ");
    Serial.println(firmware);
  }
  Serial.println("Checkpoint 5: Firmware verified");

  // attempt to connect to network
  while(connectToNetwork() < 0){
    Serial.println("Retrying connection...");
  }

  Serial.println("Checkpoint 6: Network connected");

  pinMode(windPin, INPUT);
  lastTime = millis();
  Serial.println("Checkpoint 7: Setup complete");
}

void loop() {
  Serial.println("Loop running...");
  struct MeasurementData data = measure();
  Serial.print("Temp: "); Serial.println(data.temp);
  Serial.print("Humidity: "); Serial.println(data.humidity);
  Serial.print("Wind Speed: "); Serial.println(data.wind_speed);
  Serial.print("Wind Dir: "); Serial.println(data.wind_dir);
  Serial.print("Pressure: "); Serial.println(data.pressure);
  Serial.print("Air quality: "); Serial.println(data.smoke);
  Serial.print("Ambient light: "); Serial.println(data.ambient);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Skipping data send.");
    return;
  }

  sendData(&data); 
  delay(5000);
}

struct MeasurementData measure(){
  struct MeasurementData result;

  // 1. Temperature and humidity
  result.temp = dht.readTemperature();
  result.humidity = dht.readHumidity();

  // 2. Ambient light
  int ambientLight = analogRead(AMBIENT_LIGHT_SENSOR_ANALOG_PIN);
  result.ambient = ((float) ambientLight);

  // 3. Smoke level
  int gasLevel = analogRead(GAS_SENSOR_ANALOG_PIN);
  result.smoke = ((float) gasLevel);

  // 4. Pressure
  int32_t pressure = bmp.readPressure()/100;
  result.pressure = (float) pressure;

  // 5. Wind speed 
    int analogValue = analogRead(windPin);

  // Detect falling edge below threshold
  if (analogValue < windThreshold) {
    if (!wasBelowThreshold) {
      pulseCount++;
      wasBelowThreshold = true;
    }
  } else {
    wasBelowThreshold = false;
  }

  unsigned long currentTime = millis();
  unsigned long interval = currentTime - lastTime;

  // Update every second or if no new pulses in over 5 seconds
  if (interval >= 1000 || (pulseCount == lastPulseCount && interval >= 5000)) {
    noInterrupts(); // Not strictly needed since no ISR, but for future safety
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    if (pulses > 0 || interval >= 5000) {
      double frequency = (1000.0 * pulses) / interval;

      // Apply calibration from original code
      double windSpeed = frequency - 0.00005 * frequency * frequency - 0.000014 * frequency * frequency * frequency;

      double wind_speed = windSpeed / 3.6;

      result.wind_speed = wind_speed;  

      lastTime = currentTime;
      lastPulseCount = pulses;
    }
  }

  // 6. Wind direction
  int gray = readGrayCode();
  int bin = grayToBinary(gray);

  if (bin >= 0 && bin < 32 && directions[bin] != -1) {
    result.wind_dir = directions[bin]; // wind_dir is now an int
  } else {
    result.wind_dir = -1; // or some sentinel for "Unknown"
  }

  return result;
}

void printMacAddress() {

  // the MAC address of your Wifi shield

  byte mac[6];

  // print your MAC address:

  WiFi.macAddress(mac);

  Serial.print("MAC: ");

  Serial.print(mac[5],HEX);

  Serial.print(":");

  Serial.print(mac[4],HEX);

  Serial.print(":");

  Serial.print(mac[3],HEX);

  Serial.print(":");

  Serial.print(mac[2],HEX);

  Serial.print(":");

  Serial.print(mac[1],HEX);

  Serial.print(":");

  Serial.println(mac[0],HEX);
}

int connectToNetwork() {
  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 10000; // 10 seconds
  int8_t numSSID; 
  long ssid_no;
  String password;
  char passphrase[BUFFER_SIZE];
  String SSID_str;
  char SSID[BUFFER_SIZE];

  Serial.println("Reading SSID and password from EEPROM...");

  // Attempt to connect to last network used
  eeprom_read_block(SSID, WIFI_SSID_EEPROM_ADDR, BUFFER_SIZE);
  Serial.print("Read SSID: '"); Serial.print(SSID); Serial.println("'");
  eeprom_read_block(passphrase, WIFI_PASS_EEPROM_ADDR, BUFFER_SIZE);
  Serial.print("Read passphrase: '"); Serial.print(passphrase); Serial.println("'");

  Serial.print("Attempting to connect to last used network\n\tSSID: ");
  Serial.println(SSID);
  Serial.print("\tPassword: ");
  Serial.println(passphrase);

  Serial.println("Calling WiFi.begin()...");
  WiFi.begin(SSID, passphrase);
  Serial.println("Called WiFi.begin()");

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi.");
    return -1;
  }

  Serial.println("Connected to WiFi.");
  return 1;

  // Make user choose network
  while(true){
    numSSID = scanNetworks();

    // Ask user to choose network
    Serial.print("Choose network #: ");
    String ssid_no_str = Serial.readStringUntil('\n');

    // Convert input to Int 
    ssid_no_str.trim();
    ssid_no = ssid_no_str.toInt();
    
    // Check for valid input and break loop if it specifies a valid network
    if(0 < ssid_no && ssid_no <= numSSID){
      ssid_no--;

      if(WiFi.encryptionType(ssid_no) == WIFI_SECURITY_WPA1 || WiFi.encryptionType(ssid_no) == WIFI_SECURITY_WPA1 || WiFi.encryptionType((ssid_no) == WIFI_SECURITY_NONE)){
        break;
      }
      else{
        Serial.println("Can only connect to networks with WPA");
      }
    }
    else{
    Serial.print("Invalid number! Choose a number from the given list. ");
    }
  }

  SSID_str = WiFi.SSID(ssid_no);
  SSID_str.toCharArray(SSID, BUFFER_SIZE);

  // Print attempt message
  Serial.print("Attempting to connect to ");
  Serial.println(SSID);
  Serial.print("Using password: ");
  Serial.println(passphrase);

  // Attempt to connect to WiFi
  if(WiFi.encryptionType(ssid_no) == WIFI_SECURITY_NONE){
    WiFi.begin(SSID);
  }
  else{
    for(int i = 0; i < 3; i++){
      Serial.print("Enter password for network: ");
      password = Serial.readStringUntil('\n');
      password.trim();

      password.toCharArray(passphrase, BUFFER_SIZE);

      Serial.println(passphrase);

      // Attempt to connect to network
      WiFi.begin(SSID, passphrase);
      
      unsigned long startAttemptTime = millis();
      while(WiFi.status() == WL_IDLE_STATUS && millis() - startAttemptTime < 10000) {
        delay(500);
        Serial.print(".");
      }
      Serial.println();

      if(WiFi.status() == WL_CONNECTED){
        break;
      }
    }

  }

  if(WiFi.status() == WL_CONNECTED){
    Serial.print("Success!!! Connected to ");
    Serial.println(WiFi.SSID(ssid_no));
  }
  else{
    Serial.println("Connection attempt failed... Trying again!");
    return -1;
  }

  // Write SSID to EEPROM
  eeprom_update_block(SSID, WIFI_SSID_EEPROM_ADDR, BUFFER_SIZE);

  // Write password to EEPROM
  eeprom_update_block(passphrase, WIFI_PASS_EEPROM_ADDR, BUFFER_SIZE);
  return 1;
}

int calculateJsonLength(struct MeasurementData* data) {
  int len = 0;
  len += 55; // constant part before any values

  len += String(data->temp, 1).length();
  len += 11; // ,"humidity":
  len += String(data->humidity, 1).length();
  len += 14; // ,"wind_speed":
  len += String(data->wind_speed, 1).length();
  len += 13;//,"wind_dir":
  len += String(data->wind_dir).length();
  len += 13; // ,"pressure":
  len += String(data->pressure, 1).length();
  len += 9;  // ,"smoke":
  len += String((int)data->smoke).length();
  len += 11; // ,"ambient":
  len += String(data->ambient, 1).length();
  len += 4; // final closing brackets }}]

  return len;
}

int8_t scanNetworks(){
  // Scan for nearby networks:
  Serial.println("** Scan Networks **");

  int8_t numSsid = WiFi.scanNetworks();

  // Print the list of networks seen:
  Serial.print("number of available networks: ");

  Serial.println(numSsid);

  // Print the network number and name for each network found:
  for (int thisNet = 0; thisNet<numSsid; thisNet++) {

    Serial.print(thisNet + 1);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));

    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");

    Serial.print("\tEncryption: ");
    Serial.println(convertEncryptionType(WiFi.encryptionType(thisNet)));
  }

  return numSsid;
}

String convertEncryptionType(uint8_t encryptionType){
  switch (encryptionType) {
    case 2: 
      return "WPA1";
    case 4:
      return "WPA2";
    case 5:
      return "WEP";
    case 7:
      return "None";
  }
  return "ERROR";
}

int sendData(struct MeasurementData* data) {
  client.stop();
  delay(100);

  Serial.println("Sending HTTP POST...");

  if (client.connect(WSServer, 80)) {
    Serial.println("Connected to server!");

    int contentLength = calculateJsonLength(data);

    // Send headers
    client.println(F("POST /WeatherStation/ HTTP/1.1"));
    client.println(F("Host: 217.160.149.219"));
    client.println(F("Content-Type: application/json"));
    client.print(F("Content-Length: "));
    client.println(contentLength);
    client.println();

    // Send JSON body directly, piece by piece
    client.print(F("[{\"model\":\"WeatherStation.measurements\",\"fields\":{"));
    client.print(F("\"temp\":"));       client.print(data->temp, 1);       client.print(F(","));
    client.print(F("\"humidity\":"));   client.print(data->humidity, 1);   client.print(F(","));
    client.print(F("\"wind_speed\":")); client.print(data->wind_speed, 1); client.print(F(","));
    client.print(F("\"wind_dir\":"));   client.print(data->wind_dir);      client.print(F(","));
    client.print(F("\"pressure\":"));   client.print(data->pressure, 1);   client.print(F(","));
    client.print(F("\"smoke\":"));      client.print((int)data->smoke);    client.print(F(","));
    client.print(F("\"ambient\":"));    client.print(data->ambient, 1);
    client.print(F("}}]"));

    client.flush();

    // Wait for response
    delay(500);

    while (client.available() == 0) {
      delay(10);
    }

    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }

    client.stop();
  } else {
    Serial.println("Connection failed.");
  }
  return 0;
}