#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// DHT sensor library for ESPx@1.19.0
#include "DHTesp.h"
DHTesp dht; //Define the DHT object

// sudo chmod a+rw /dev/ttyUSB0
IPAddress apIP(192, 168, 2, 1);
IPAddress netMsk(255, 255, 255, 0);
  

// Replace with your network credentials
char defaultSSID[32] = "";
char defaultPassword[32] = "";

// EEPROM addresses for ssid and password
int ssidAddress = 0;
int passwordAddress = 32;

// Create an instance of the ESP8266WebServer class
ESP8266WebServer server(80);

const int ledPin = 2; // GPIO 2
const int buttonPin = 0; // GPIO 0
const int fanPin = 4; // GPIO 4
const int buzzerPin = 15; // GPIO 15
const int dhtPin = 13;//Define the dht pin
const int fotoresistorPin = A0;  // Analog pin 

// Variable to store the button state
int buttonState = 0;

// Variables for LED blink
unsigned long previousMillis = 0;
const long interval = 1000; // Blink interval in milliseconds

enum FridgeDoorState
{
  FRIDGE_DOOR_UNKNOWN_STATE,
  FRIDGE_DOOR_RECENTLY_OPENED,
  FRIDGE_DOOR_LONG_AGO_OPENED,
  FRIDGE_DOOR_RECENTLY_CLOSED,
  FRIDGE_DOOR_LONG_AGO_CLOSED
};

enum WiFiSetupState
{
  INIT,
  DISCONNECT,
  CHECK_SSID,
  INVALID_CONFIG,
  CONNECT_WIFI,
  WAIT_FOR_CONNECTION,
  CONNECTED
};

WiFiSetupState wifiSetupState = INIT;
bool late_setup_done = false;

struct GlobalState {
  bool wifi_configured = false;
  int light = 0;
  float temperature = 0.0f;
  float humidity = 0.0f;
  char info[120] = "";
  FridgeDoorState fridge_door = FRIDGE_DOOR_UNKNOWN_STATE;
} global_state;

void printLEDState(bool state)
{
  if (state)
  {
    Serial.println("LED turned ON " + WiFi.softAPIP().toString() + " " + server.client().remoteIP().toString());
  }
  else
  {
    Serial.println("LED turned OFF "+ WiFi.softAPIP().toString() + " " + server.client().remoteIP().toString());
  }

  if (wifiSetupState == CONNECTED) {
    int coverage = map(WiFi.RSSI(), 32, -90, 100, 0);
    Serial.print("RSSI ");
    Serial.print(WiFi.RSSI(), DEC);
    Serial.print("coverage ");
    Serial.print(coverage, DEC);
    Serial.println(" Local IP: " + WiFi.localIP().toString());
  }
  
  String info = "";

  switch(global_state.fridge_door) {
    case FRIDGE_DOOR_UNKNOWN_STATE: info += "FRIDGE_DOOR_UNKNOWN_STATE"; break;
    case FRIDGE_DOOR_RECENTLY_OPENED: info += "FRIDGE_DOOR_RECENTLY_OPENED"; break;
    case FRIDGE_DOOR_LONG_AGO_OPENED: info += "FRIDGE_DOOR_LONG_AGO_OPENED"; break;
    case FRIDGE_DOOR_RECENTLY_CLOSED: info += "FRIDGE_DOOR_RECENTLY_CLOSED"; break;
    case FRIDGE_DOOR_LONG_AGO_CLOSED: info += "FRIDGE_DOOR_LONG_AGO_CLOSED"; break;
  }
  
  if (dht.getStatus() == 0) { //Judge if the correct value is read
    info = info + " Light:" + String(global_state.light) + " Temperature:" + String(global_state.temperature) + " C"+" Humidity:" + String(global_state.humidity)+" %";
    Serial.println(info);
    memcpy(global_state.info, info.c_str(), sizeof(global_state.info));
  }
  
}

void writeToEEPROM(int address, const char *data, int dataSize) {
  for (int i = 0; i < dataSize; ++i) {
    EEPROM.write(address + i, data[i]);
//    Serial.print("Write ");
//    Serial.print(address + i);
//    Serial.print("data ");
//    Serial.print(data[i]);
//    Serial.print(" (Decimal: ");
//    Serial.print((int)data[i]);
//    Serial.print(", Hexadecimal: 0x");
//    Serial.print((int)data[i], HEX);
//    Serial.println(")");
  }
  EEPROM.commit();
}

void readFromEEPROM(int address, char *data, int dataSize) {
  for (int i = 0; i < dataSize; ++i) {
    data[i] = EEPROM.read(address + i);
//    Serial.print("Read ");
//    Serial.print(address + i);
//    Serial.print("data ");
//    Serial.print(data[i]);
//    Serial.print(" (Decimal: ");
//    Serial.print((int)data[i]);
//    Serial.print(", Hexadecimal: 0x");
//    Serial.print((int)data[i], HEX);
//    Serial.println(")");
  }
}

bool isGarbage(const char *value, int length)
{
  for (int i = 0; i < length; ++i)
  {
    char currentChar = value[i];
    if (currentChar == '\0' && i > 0) {
      return false;
    }
    
    if (!isAlphaNumeric(currentChar) && currentChar != '_' && currentChar != '-')
    {
      Serial.print("Garbage value detected: ");
      Serial.print(currentChar);
      Serial.print(" (Decimal: ");
      Serial.print((int)currentChar);
      Serial.print(", Hexadecimal: 0x");
      Serial.print((int)currentChar, HEX);
      Serial.println("). Using default value.");
      return true; // Non-alphanumeric character found, consider it garbage
    }
  }
  return false; // No invalid characters found
}

bool stateMachineWiFi()
{
  static char storedSSID[32];
  static char storedPassword[32];
  static unsigned long startMillis;
  
  WiFiSetupState previousState = wifiSetupState;
  switch (wifiSetupState)
  {
  case INIT:
    Serial.println("Init WiFi...");
    WiFi.mode(WIFI_AP_STA);

    // Configure softAP with custom IP and DHCP settings
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.softAP("ESP8266-AP", "password");
    
    wifiSetupState = DISCONNECT;
    break;
  case DISCONNECT:
    Serial.println("Disconnect previous connections...");
    WiFi.disconnect(true);
    wifiSetupState = CHECK_SSID;
    break;
  case CHECK_SSID:
    Serial.println("Checking stored SSID...");
    readFromEEPROM(ssidAddress, storedSSID, 32);

    if (isGarbage(storedSSID, sizeof(storedSSID)))
    {
      Serial.println("Stored SSID is garbage. Using default SSID.");
      strcpy(storedSSID, defaultSSID);
    }
    readFromEEPROM(passwordAddress, storedPassword, 32);

    if (isGarbage(storedPassword, sizeof(storedPassword)))
    {
      Serial.println("Stored password is garbage. Using default password.");
      strcpy(storedPassword, defaultPassword);
      global_state.wifi_configured = false;
    }
    
    if (!isGarbage(storedSSID, sizeof(storedSSID)) && !isGarbage(storedPassword, sizeof(storedPassword)))
    {
      wifiSetupState = CONNECT_WIFI;
      global_state.wifi_configured = true;
    }
    else
    {
      wifiSetupState = INVALID_CONFIG;
    }
    break;

  case INVALID_CONFIG:
    break;

  case CONNECT_WIFI:    
    Serial.println("Connecting to WiFi...");
    Serial.print("SSID: ");
    Serial.println(storedSSID);
    Serial.print("Password: ");
    Serial.println(storedPassword);
    WiFi.begin(storedSSID, storedPassword);
    startMillis = millis();
    wifiSetupState = WAIT_FOR_CONNECTION;
    break;

  case WAIT_FOR_CONNECTION:
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: " + WiFi.localIP().toString());
      Serial.println("SSID of connected network: " + WiFi.SSID());
      wifiSetupState = CONNECTED; // Reset state machine for the next time setupWiFi is called
    }
    else if (millis() - startMillis > 40000) // Wait for up to 10 seconds
    {
      Serial.println("\nWiFi connection failed. Retrying...");
      Serial.print("SSID: ");
      Serial.println(storedSSID);
      Serial.print("Password: ");
      Serial.println(storedPassword);

      wifiSetupState = CONNECT_WIFI;
    }
    break;
  case CONNECTED:
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("\nWiFi disconnected...");
      wifiSetupState = CONNECT_WIFI;
    }
    break;
  }
  return previousState != wifiSetupState;
}

// period
// repeat_4_times
// crick_4_times
// base
void cricket_beep(unsigned long startMillis, unsigned long beep_millis) {
  unsigned long state_period = millis() - startMillis;
  unsigned long sign_period = state_period % 40000;

  unsigned long crick_4_times_period = sign_period % (700*8);
  bool crick_4_times = false;
  for (int i=0; i < 4; i++) {
    unsigned int start = 0 + i * 700;
    unsigned int end = 500 + i * 700;
    if (start < crick_4_times_period && crick_4_times_period < end)
    {
      crick_4_times = true;
      break;
    }
  }
  const bool base = ((sign_period % 30) > 15);
  if ( (sign_period < beep_millis)  && crick_4_times && base) {
    digitalWrite(buzzerPin, LOW);
    //digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

}

void transitions_door_state() {
  if (global_state.light > 50) {
    if (global_state.fridge_door != FRIDGE_DOOR_RECENTLY_OPENED && global_state.fridge_door != FRIDGE_DOOR_LONG_AGO_OPENED)
      global_state.fridge_door = FRIDGE_DOOR_RECENTLY_OPENED;
  } else {
    if (global_state.fridge_door != FRIDGE_DOOR_RECENTLY_CLOSED && global_state.fridge_door != FRIDGE_DOOR_LONG_AGO_CLOSED)
      global_state.fridge_door = FRIDGE_DOOR_RECENTLY_CLOSED;
  }
}

bool fridge_door_closed() {
  return global_state.light > 50;
}

bool stateMachineFridge() {
  static unsigned long current_state_start = 0;
  bool alert = false;

  FridgeDoorState previousState = global_state.fridge_door;
  switch (global_state.fridge_door)
  {
    case FRIDGE_DOOR_UNKNOWN_STATE:
      // If door is closed, transition directly to FRIDGE_DOOR_RECENTLY_CLOSED
      // If door is open, transition directly to FRIDGE_DOOR_RECENTLY_OPENED
      break;
    case FRIDGE_DOOR_RECENTLY_OPENED:
      alert = true;
      // Turn off fan
      // Do one beep if there were pending notifications
      // Wait for 30 seconds
      // If door is closed, transition directly to FRIDGE_DOOR_RECENTLY_CLOSED
      break;
    case FRIDGE_DOOR_LONG_AGO_OPENED:
      alert = true;
      // Fan stays off
      // Start beeping in a perdiodic manner
      // If door is closed, transition directly to FRIDGE_DOOR_RECENTLY_CLOSED
      break;
    case FRIDGE_DOOR_RECENTLY_CLOSED:
      // Immediate Fan on
      // No beepeing
      // If door is open, transition directly to FRIDGE_DOOR_RECENTLY_OPENED
      // Wait for 5 minutes before FRIDGE_DOOR_LONG_AGO_CLOSED
      break;
    case FRIDGE_DOOR_LONG_AGO_CLOSED:
      // Alternate fan
      // If door is open, transition directly to FRIDGE_DOOR_RECENTLY_OPENED
      break;
  }

  if (alert) {
    cricket_beep(current_state_start, 10000);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  transitions_door_state();
  bool change_state = previousState != global_state.fridge_door;
  if (change_state) {
    current_state_start = millis();
  }

  return change_state;
}

void handleRoot()
{

  // Get WiFi connection status and SSID
  String connectionStatus;
  String connectedSSID;
  String connectedIP;

  if (WiFi.status() == WL_CONNECTED)
  {
    connectionStatus = "Connected";
    connectedSSID = "SSID: " + WiFi.SSID();
    connectedIP = WiFi.localIP().toString();
  }
  else
  {
    connectionStatus = "Not Connected";
    connectedSSID = "SSID: N/A";
    connectedIP = "IP: N/A";
  }

  String html = "<html><body>";
  html += "<h1 id='info'>Info: " + String(global_state.light) + "</h1>";
  html += "<p>WiFi Connection Status: " + connectionStatus + "</p>";
  html += "<p>" + connectedSSID + "</p>";
  html += "<p>" + connectedIP + "</p>";
  if (!global_state.wifi_configured) {
    html += "<form action='/update' method='post'>";
    html += "New SSID: <input type='text' name='new_ssid'><br>";
    html += "New Password: <input type='password' name='new_password'><br>";
    html += "<input type='submit' value='Update WiFi'>";
    html += "</form>";
  }
  html += "<script>function updateInfo(){"
    "var xhttp=new XMLHttpRequest();"
    "xhttp.onreadystatechange=function(){"
      "if(this.readyState==4&&this.status==200){"
        "document.getElementById('info').innerHTML='Info: '+this.responseText}};"
    "xhttp.open('GET','/info',true);xhttp.send();}"
  "setInterval(updateInfo,1000);</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);

  Serial.println("Handled root request.");
}

void handleUpdate()
{
  
  static char storedSSID[32];
  static char storedPassword[32];
  
  // Handle form submission to update WiFi credentials
  String new_ssid = server.arg("new_ssid");
  String new_password = server.arg("new_password");

  Serial.println("Updating WiFi credentials...");
  Serial.print("New SSID: ");
  Serial.println(new_ssid);
  Serial.print("New Password: ");
  Serial.println(new_password);

  // Save new credentials to EEPROM
  if (!isGarbage(new_ssid.c_str(), new_ssid.length()))
  {
    writeToEEPROM(ssidAddress, new_ssid.c_str(), 32 );
  }

  if (!isGarbage(new_password.c_str(), new_password.length()))
  {
    writeToEEPROM(passwordAddress, new_password.c_str(), 32 );
  }

  Serial.println("WiFi credentials updated. Reconnecting to WiFi...");

  // Reconnect to WiFi with new credentials
  wifiSetupState = CHECK_SSID;

  server.send(200, "text/plain", "WiFi credentials updated. Please reconnect to the new WiFi.");
}

void handleInfo()
{

  // Respond with the simulated light
  server.send(200, "text/plain", global_state.info);
}

// ISR definition with ICACHE_RAM_ATTR attribute
void ICACHE_RAM_ATTR buttonInterrupt() {
  // Read the state of the button
  buttonState = digitalRead(buttonPin);

  // Command on 0 to prevent the initial high trigger
  if (buttonState == 0) {
    char zero = 0;
    char dummy = 0;
    Serial.println("Reset pw");
    delay(2000);
    writeToEEPROM(ssidAddress, &zero, 1 );
    writeToEEPROM(passwordAddress, &zero, 1 );
    readFromEEPROM(ssidAddress, &dummy, 1 );
    readFromEEPROM(passwordAddress, &dummy, 1 );
    wifiSetupState = DISCONNECT;
  }

  // Print the button state (HIGH or LOW)
  Serial.print("Button State: ");
  Serial.println(buttonState);
}

void setup()
{
  EEPROM.begin(512);
  // Set up LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  dht.setup(dhtPin, DHTesp::DHT11);//Initialize the dht pin and dht object

  // Start Serial for debugging
  Serial.begin(115200);
  delay(10);

  
  // Set the button pin as input
  pinMode(buttonPin, INPUT);


  // New line prints after Serial port setup
  Serial.println("\nSerial port setup complete.");
  Serial.println("===================================");


  // Start the server
  Serial.println("Starting server...");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/info", HTTP_GET, handleInfo); // New route for info
  server.begin();
}

// Some functions, that depend on hardware to stabilize, get setup after the first interval is ready
void late_setup() {

  // Attach an interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE);
  late_setup_done = true;
}

void updateSensorData() {
  int analogValue = analogRead(fotoresistorPin);
  global_state.light = constrain(map(analogValue, 1024, 0, 0, 100), 0, 100); // Map the analog value to a light range

  flag:TempAndHumidity newValues = dht.getTempAndHumidity(); //Get the Temperature and humidity
  global_state.temperature = newValues.temperature;
  global_state.humidity = newValues.humidity;
}

void loop()
{
  // Handle client requests
  server.handleClient();

  // Blink the LED
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval || currentMillis < previousMillis)
  {
    // Save the last time we blinked the LED
    previousMillis = currentMillis;

    if (late_setup_done == false) {
      late_setup();
    }

    updateSensorData();
    // Toggle the LED state
    if (digitalRead(ledPin) == HIGH)
    {
      digitalWrite(ledPin, LOW);
      printLEDState(false);
    }
    else
    {
      // digitalWrite(buzzerPin, HIGH);
      // delay(10);
      // digitalWrite(buzzerPin, LOW);
      digitalWrite(ledPin, HIGH);
      printLEDState(true);
    }
  }

  // Non-blocking setupWiFi
  while (stateMachineFridge());

  // Non-blocking setupWiFi
  while (stateMachineWiFi());
}
