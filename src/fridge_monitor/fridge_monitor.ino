#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

#include <ArduinoOTA.h>

// Uptime Library@1.0.0
#include "uptime_formatter.h"

// ESP8266TimerInterrupt@1.6.0
#include "ESP8266TimerInterrupt.h"

// For startup info and more
extern "C" {
#include "user_interface.h"
}

// Other
// WebSerial@2.0.6

// Select a Timer Clock
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default

// Init ESP8266 only and only Timer 1
ESP8266Timer ITimer;
#define TIMER_INTERVAL_MS        1000

// DHT sensor library for ESPx@1.19.0
#include "DHTesp.h"
DHTesp dht; //Define the DHT object

// sudo chmod a+rw /dev/ttyUSB0
IPAddress apIP(192, 168, 2, 1);
IPAddress netMsk(255, 255, 255, 0);

// Replace with your network credentials
#define SIZE_SSDID 32
#define SIZE_PASSWORD 32

#define EEPROM_ADDRESS_SSID 0
#define EEPROM_ADDRESS_PASSWORD SIZE_SSDID + EEPROM_ADDRESS_SSID
char defaultSSID[32] = "";
char defaultPassword[32] = "";

// EEPROM addresses for ssid and password
const int ssidAddress = 0;
const int passwordAddress = EEPROM_ADDRESS_PASSWORD;

// Create an instance of the ESP8266WebServer class
ESP8266WebServer server(80);
ESP8266WebServer serial_server(8080);

// 1 RST Reset Pin, Active Low
// 2 ADC AD conversion, Input voltage range 0~3.3V, the value range is 0~1024.
const int fotoresistorPin = A0;  // Analog pin. 
// 3 EN Chip Enabled Pin, Active High
// 4 IO16 Connect with RST pin to wake up Deep Sleep
// 5 IO14 GPIO14; HSPI_CLK
// 6 IO12 GPIO12; HSPI_MISO
const int lightFridgePin = 12;  // Pull up
// 7 IO13 GPIO13; HSPI_MOSI; UART0_CTS
const int dhtPin = 13;//Define the dht pin 
// 8 VCC Module power supply pin, Voltage 3.0V ~ 3.6V
// 9 GND GND
// 10 IO15 GPIO15; MTDO; HSPICS; UART0_RTS
const int buzzerPin = 15; // GPIO 15
// 11 IO2 GPIO2; UART1_TXD
const int ledPin = 2; // GPIO 2
// 12 IO0 GPIO0;HSPI_MISO;I2SI_DATA
const int buttonPin = 0; // GPIO 0
// 13 IO4 GPIO4
const int fanPin = 4; // GPIO 4. Brown
// 14 IO5 GPIO5;IR_R
const int doorFridgePin = 5;  // Pull up
// 15 RXD UART0_RXD; GPIO3
// 16 TXD UART0_TXD; GPIO1
// Variable to store the button state

int buttonState = 0;

// Variables for LED blink
unsigned long previousMillis = 0;
const long interval = 1000; // Blink interval in milliseconds

enum FridgeDoorState
{
  FRIDGE_DOOR_UNKNOWN_STATE,
  FRIDGE_DOOR_JUST_OPENED,
  FRIDGE_DOOR_RECENTLY_OPENED,
  FRIDGE_DOOR_LONG_AGO_OPENED,
  FRIDGE_DOOR_JUST_CLOSED,
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

// variable to hold the time
struct RetainedState {
  unsigned long garbage_id;
  unsigned long program_counter;
  bool enabled_ota;
};
volatile RetainedState retained_state __attribute__ ((section(".noinit")));

struct NormalState {
  int reset_reason=0;
  unsigned long program_counter_previous;
  bool wifi_configured = false;
  int analogValue = 0;
  bool light_on = false;
  bool door_contact_closed = false;
  bool initialized_records = false;
  float temperature = 0.0f;
  float humidity = 0.0f;
  float min_temperature = 0.0f;
  float min_temperature_humidity = 0.0f;
  float max_temperature = 0.0f;
  float max_temperature_humidity = 0.0f;
  float min_humidity = 0.0f;
  float min_humidity_temperature = 0.0f;
  float max_humidity = 0.0f;
  float max_humidity_temperature = 0.0f;
  FridgeDoorState fridge_door = FRIDGE_DOOR_UNKNOWN_STATE;
  bool alert = false;
  int beeps = false;
  int beep_period = 0;
  bool local_client = false;
  unsigned long current_state_start = 0;
  unsigned long current_client_time = 0;
  bool handling_ota=true;
};
volatile NormalState normal_state;

char info_[500] = "";

int get_viewer_time() {
  if (normal_state.local_client) {
    return (millis() - normal_state.current_client_time)/1000;
  }
  return 0;
}

void updateInfoString(bool state)
{
  Serial.println("LED turned OFF "+ WiFi.softAPIP().toString() + " " + server.client().remoteIP().toString());

  if (wifiSetupState == CONNECTED) {
    int coverage = map(WiFi.RSSI(), 32, -90, 100, 0);
    Serial.print("RSSI ");
    Serial.print(WiFi.RSSI(), DEC);
    Serial.print("coverage ");
    Serial.print(coverage, DEC);
    Serial.println(" Local IP: " + WiFi.localIP().toString());
  }
  
  String info = uptime_formatter::getUptime() + " Viewer " + get_viewer_time() + " <br>";

  switch(normal_state.fridge_door) {
    case FRIDGE_DOOR_UNKNOWN_STATE: info += "FRIDGE_DOOR_UNKNOWN_STATE "; break;
    case FRIDGE_DOOR_JUST_OPENED: info += "FRIDGE_DOOR_JUST_OPENED "; break;
    case FRIDGE_DOOR_RECENTLY_OPENED: info += "FRIDGE_DOOR_RECENTLY_OPENED "; break;
    case FRIDGE_DOOR_LONG_AGO_OPENED: info += "FRIDGE_DOOR_LONG_AGO_OPENED "; break;
    case FRIDGE_DOOR_RECENTLY_CLOSED: info += "FRIDGE_DOOR_RECENTLY_CLOSED "; break;
    case FRIDGE_DOOR_JUST_CLOSED: info += "FRIDGE_DOOR_JUST_CLOSED "; break;
    case FRIDGE_DOOR_LONG_AGO_CLOSED: info += "FRIDGE_DOOR_LONG_AGO_CLOSED "; break;
  }
  
  
  info = info + " ADC:" + String(normal_state.analogValue) + " Light:" + String(normal_state.light_on) + " DoorClosed:" + String(normal_state.door_contact_closed) + " <br>";
  
  info = info + "Temperature:" + String(normal_state.temperature) + " C"+" Humidity:" + String(normal_state.humidity)+" % "  + " <br>";


  info = info + "C Min/Max " + String(normal_state.min_temperature) + " (" + String(normal_state.min_temperature_humidity) + ") " +
    String(normal_state.max_temperature) + " (" + String(normal_state.max_temperature_humidity) + ")"  + " <br>";
  
  info = info + "% Min/Max " + String(normal_state.min_humidity) + " (" + String(normal_state.min_humidity_temperature) + ") " +
    String(normal_state.max_humidity) + " (" + String(normal_state.max_humidity_temperature) + ")"  + " <br>";

  uint32_t free = system_get_free_heap_size();
  info = info + "free " + String(free) + " local " + String(normal_state.local_client) + " <br>";

  retained_state.program_counter = __LINE__;
  info = info + " previous last state " + String(normal_state.program_counter_previous) + " " + String(retained_state.program_counter) + " <br>";
  switch(normal_state.reset_reason) {
    case REASON_DEFAULT_RST: info += "normal startup by power on <br>"; break;
    case REASON_WDT_RST: info += "hardware watch dog reset <br>"; break;
    case REASON_EXCEPTION_RST: info += "exception reset, GPIO status will not change <br>"; break;
    case REASON_SOFT_WDT_RST: info += "software watch dog reset, GPIO status will not change <br>"; break;
    case REASON_SOFT_RESTART: info += "software restart ,system_restart , GPIO status will not change <br>"; break;
    case REASON_DEEP_SLEEP_AWAKE: info += "wake up from deep-sleep <br>"; break;
    case REASON_EXT_SYS_RST: info += "external system reset <br>"; break;
  }
  
  info = info + "OTA Enabled: " + String(retained_state.enabled_ota) + " <br>";
  info = info + "OTA Allowed: " + String(allowed_ota()) + " <br>";

  info = info + String(info.length()+1);
  memcpy(info_, info.c_str(), min(info.length()+1, sizeof(info_)));
  info_[sizeof(info_)-1] = 0;
  Serial.println(info_);
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
    //WiFi.setMinSecurity(WIFI_AUTH_WPA2_WPA3_PSK);

    // Configure softAP with custom IP and DHCP settings
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.softAP("ESP8266-AP", "password");

    // 0    (for lowest RF power output, supply current ~ 70mA
    // 20.5 (for highest RF power output, supply current ~ 80mA
    WiFi.setOutputPower(17.0);
    
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
      normal_state.wifi_configured = false;
    }
    
    if (!isGarbage(storedSSID, sizeof(storedSSID)) && !isGarbage(storedPassword, sizeof(storedPassword)))
    {
      wifiSetupState = CONNECT_WIFI;
      normal_state.wifi_configured = true;
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
  retained_state.program_counter = __LINE__;
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
    //digitalWrite(buzzerPin, LOW);
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }
  retained_state.program_counter = __LINE__;
}

void transitions_door_state() {
  if (!normal_state.door_contact_closed) {
    if (normal_state.fridge_door != FRIDGE_DOOR_JUST_OPENED && normal_state.fridge_door != FRIDGE_DOOR_RECENTLY_OPENED && normal_state.fridge_door != FRIDGE_DOOR_LONG_AGO_OPENED)
      change_fridge_state(FRIDGE_DOOR_JUST_OPENED);
  } else {
    if (normal_state.fridge_door != FRIDGE_DOOR_JUST_CLOSED && normal_state.fridge_door != FRIDGE_DOOR_RECENTLY_CLOSED && normal_state.fridge_door != FRIDGE_DOOR_LONG_AGO_CLOSED)
      change_fridge_state(FRIDGE_DOOR_JUST_CLOSED);
  }
}

void cricket_off() {
  if (normal_state.alert) {
    digitalWrite(buzzerPin, LOW);
    normal_state.alert = false;
  }
}

void cricket_on() {
  normal_state.alert = true;
}

void change_fridge_state(FridgeDoorState new_state) {
  normal_state.fridge_door = new_state;
  normal_state.current_state_start = 0;
}

bool stateMachineFridge() {

  FridgeDoorState previousState = normal_state.fridge_door;
  switch (normal_state.fridge_door)
  {
    case FRIDGE_DOOR_UNKNOWN_STATE:
      cricket_off();
      digitalWrite(fanPin, LOW);
      // If door is closed, transition directly to FRIDGE_DOOR_RECENTLY_CLOSED
      // If door is open, transition directly to FRIDGE_DOOR_RECENTLY_OPENED
      break;
    case FRIDGE_DOOR_JUST_OPENED:
      cricket_off();
      digitalWrite(buzzerPin, HIGH);
      delay(2);
      digitalWrite(buzzerPin, LOW);
      digitalWrite(fanPin, LOW);
      change_fridge_state(FRIDGE_DOOR_RECENTLY_OPENED);
      break;
    case FRIDGE_DOOR_RECENTLY_OPENED:
      cricket_off();
      digitalWrite(fanPin, LOW);
      // Turn off fan
      // Do one beep if there were pending notifications
      // Wait for 30 seconds4
      if ((millis() - normal_state.current_state_start) > (30 * 1000)) {
        change_fridge_state(FRIDGE_DOOR_LONG_AGO_OPENED);
        cricket_on();
      }
      // If door is closed, transition directly to FRIDGE_DOOR_RECENTLY_CLOSED
      break;
    case FRIDGE_DOOR_LONG_AGO_OPENED:
      // Fan stays off
      // Start beeping in a perdiodic manner
      // If door is closed, transition directly to FRIDGE_DOOR_RECENTLY_CLOSED
      break;
    case FRIDGE_DOOR_JUST_CLOSED:
      digitalWrite(buzzerPin, HIGH);
      delay(2);
      digitalWrite(buzzerPin, LOW);
      cricket_off();
      digitalWrite(fanPin, HIGH);
      change_fridge_state(FRIDGE_DOOR_RECENTLY_CLOSED);
      break;
    case FRIDGE_DOOR_RECENTLY_CLOSED:
      // Immediate Fan on
      // No beepeing
      // If door is open, transition directly to FRIDGE_DOOR_RECENTLY_OPENED
      // Wait for 5 minutes before FRIDGE_DOOR_LONG_AGO_CLOSED
      if ((millis() - normal_state.current_state_start) > (5 * 60 * 1000)) {
        change_fridge_state(FRIDGE_DOOR_LONG_AGO_CLOSED);
      }
      break;
    case FRIDGE_DOOR_LONG_AGO_CLOSED:
      // Alternate fan
      // If door is open, transition directly to FRIDGE_DOOR_RECENTLY_OPENED
      break;
  }

  transitions_door_state();
  bool change_state = previousState != normal_state.fridge_door;
  if (change_state || normal_state.current_state_start > millis()) {  // Cover overflow
    normal_state.current_state_start = millis();
  }

  return change_state;
}

void handleRoot()
{
  retained_state.program_counter = __LINE__;

  String html = "<html><body>";
  html += __DATE__ " "  __TIME__ " version: 0.0.3";
  html += "<p id='info'>Info: " + String(info_) + "</p>";  //h1
 
  html += "<p> APIP: " + WiFi.softAPIP().toString() + "</p>";
  if (WiFi.status() == WL_CONNECTED)
  {
    html += "<p> SSID: " + WiFi.SSID() + "</p>";
    html += "<p> ESP IP: " + WiFi.localIP().toString() + "</p>";
    html += "<p> Client IP: " + server.client().remoteIP().toString() + "</p>";
  }
  // https://stackoverflow.com/questions/25983603/how-to-submit-an-html-form-without-redirection
  html += "<iframe name='dummyframe' id='dummyframe' style='display: none;'></iframe>";

  html += "<form action='/enable_ota' method='post' target='dummyframe'>";
  html += "<input type='submit' value='Enable OTA'>";
  html += "</form>";

  html += "<form action='/disable_ota' method='post' target='dummyframe'>";
  html += "<input type='submit' value='Disable OTA'>";
  html += "</form>";

  html += "<form action='/clear_records' method='post' target='dummyframe'>";
  html += "<input type='submit' value='Clear records'>";
  html += "</form>";

  if (!normal_state.wifi_configured || normal_state.local_client) {

    html += "<form action='/reset' method='post' target='dummyframe'>";
    html += "<input type='submit' value='Reset'>";
    html += "</form>";

    html += "<form action='/restart' method='post' target='dummyframe'>";
    html += "<input type='submit' value='Restart'>";
    html += "</form>";

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
  retained_state.program_counter = __LINE__;
}

void handleClearRecords() {
  retained_state.program_counter = __LINE__;
  normal_state.initialized_records = false;
  server.send(200, "text/plain", "Clear done");
  retained_state.program_counter = __LINE__;
}

void handleReset() {
  retained_state.program_counter = __LINE__;
  server.send(200, "text/plain", "Reseting...");
  ESP.reset();
}

void handleRestart() {
  retained_state.program_counter = __LINE__;
  server.send(200, "text/plain", "Restarting...");
  ESP.restart();
}

void handleDisableOTA() {
  retained_state.program_counter = __LINE__;
  server.send(200, "", "");
  retained_state.enabled_ota = false;
}

void handleEnableOTA() {
  retained_state.program_counter = __LINE__;
  server.send(200, "", "");
  retained_state.enabled_ota = true;
}

void handleUpdate()
{
  retained_state.program_counter = __LINE__;
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
  wifiSetupState = DISCONNECT;

  server.send(200, "text/plain", "WiFi credentials updated. Please reconnect to the new WiFi.");
  retained_state.program_counter = __LINE__;
}

void handleInfo()
{
  retained_state.program_counter = __LINE__;
  // Respond with the simulated light
  server.send(200, "text/plain", info_);
  retained_state.program_counter = __LINE__;
}

// ISR definition with ICACHE_RAM_ATTR attribute
void ICACHE_RAM_ATTR buttonInterrupt() {
  retained_state.program_counter = __LINE__;
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
  retained_state.program_counter = __LINE__;
}


#define HW_TIMER_INTERVAL_MS          1L

#define INTERVAL_BEEP                 200
#define TIMER_INTERVAL_5S             5000L
#define TIMER_INTERVAL_11S            11000L
#define TIMER_INTERVAL_101S           101000L

void BeepHandler() {
  if (normal_state.beeps > 0) {
    if (digitalRead(buzzerPin) == LOW){
      digitalWrite(buzzerPin, HIGH);
    } else {
      normal_state.beeps--;
      digitalWrite(buzzerPin, LOW);
    }
  }
}

void IRAM_ATTR TimerHandler()
{
  retained_state.program_counter = __LINE__;
  if (normal_state.handling_ota) {
    retained_state.program_counter = __LINE__;
    return;
  }
  if (normal_state.beep_period > 0) {
    retained_state.program_counter = __LINE__;
    normal_state.beep_period--;
  } else {
    retained_state.program_counter = __LINE__;
    BeepHandler();
    normal_state.beep_period = INTERVAL_BEEP;
  }
  // Doing something here inside ISR
    // Toggle the LED state
  // if (digitalRead(ledPin) == HIGH)
  // {
  //   digitalWrite(ledPin, LOW);
  //   // updateInfoString(false);
  // }
  // else
  // {
    // digitalWrite(buzzerPin, HIGH);
    // delay(10);
    // digitalWrite(buzzerPin, LOW);
    //digitalWrite(ledPin, HIGH);
    // updateInfoString(true);
  // }

  if (normal_state.alert) {
    retained_state.program_counter = __LINE__;
    cricket_beep(normal_state.current_state_start, 10000);
  } else if (normal_state.beeps == 0) {
    retained_state.program_counter = __LINE__;
    digitalWrite(buzzerPin, LOW);
  }
  retained_state.program_counter = __LINE__;
}


void setup()
{
  EEPROM.begin(512);
  // Set up LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(doorFridgePin, INPUT_PULLUP);
  pinMode(lightFridgePin, INPUT_PULLUP); // For high sensitivity, disable the pullup INPUT_PULLUP

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
  server.on("/clear_records", HTTP_POST, handleClearRecords);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/restart", HTTP_POST, handleRestart);
  server.on("/disable_ota", HTTP_POST, handleDisableOTA);
  server.on("/enable_ota", HTTP_POST, handleEnableOTA);
  server.on("/info", HTTP_GET, handleInfo); // New route for info
  server.begin();

  
  serial_server.on("/", HTTP_GET, handleRoot);
  serial_server.begin();
  
  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * TIMER_INTERVAL_MS, TimerHandler);



  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"password");
  
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  // tools.esptool.upload.network_pattern="{network_cmd}" -I "{runtime.platform.path}/tools/espota.py" -i "{serial.port}" -p "{network.port}" "--auth={network.password}" -f "{build.path}/{build.project_name}.bin"
  // tools.esptool.upload.network_pattern="{network_cmd}" -I "{runtime.platform.path}/tools/espota.py" -i "192.168.1.30" -p "8266" "--auth={network.password}" -f "{build.path}/{build.project_name}.bin"
  ArduinoOTA.begin(true);

  normal_state.beeps = 3;

  Serial.println("We're alive") ;
  rst_info *rinfo;
  rinfo = ESP.getResetInfoPtr();
  normal_state.reset_reason = (int)rinfo->reason;
  Serial.println(String("ResetInfo.reason = ") + (int)rinfo->reason);

  if (retained_state.garbage_id != 0xF0F0F0F0) {
    retained_state.garbage_id = 0xF0F0F0F0;
    retained_state.program_counter = 0;
    retained_state.enabled_ota = false;
  }
  normal_state.program_counter_previous = retained_state.program_counter;
}

// Some functions, that depend on hardware to stabilize, get setup after the first interval is ready
void late_setup() {

  // Attach an interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE);
  late_setup_done = true;
}

void updateSensorData() {
  normal_state.analogValue = analogRead(fotoresistorPin);
  // normal_state.light = constrain(map(analogValue, 1024, 0, 0, 100), 0, 100); // Map the analog value to a light range
  normal_state.light_on = digitalRead(lightFridgePin) == LOW;
  normal_state.door_contact_closed = digitalRead(doorFridgePin) == LOW;

  flag:TempAndHumidity newValues = dht.getTempAndHumidity(); //Get the Temperature and humidity
  if (dht.getStatus() == 0) { //Judge if the correct value is read
    normal_state.temperature = newValues.temperature;
    normal_state.humidity = newValues.humidity;
    if (!normal_state.initialized_records) {
      normal_state.initialized_records = true;
      normal_state.min_temperature = normal_state.temperature;
      normal_state.min_temperature_humidity = normal_state.humidity;
      normal_state.max_temperature = normal_state.temperature;
      normal_state.max_temperature_humidity = normal_state.humidity;
      normal_state.min_humidity = normal_state.humidity;
      normal_state.min_humidity_temperature = normal_state.temperature;
      normal_state.max_humidity = normal_state.humidity;
      normal_state.max_humidity_temperature = normal_state.temperature;
    }

    if (normal_state.temperature < normal_state.min_temperature) {
      normal_state.min_temperature = normal_state.temperature;
      normal_state.min_temperature_humidity = normal_state.humidity;
    }
    if (normal_state.temperature > normal_state.max_temperature) {
      normal_state.max_temperature = normal_state.temperature;
      normal_state.max_temperature_humidity = normal_state.humidity;
    }
    if (normal_state.humidity < normal_state.min_humidity) {
      normal_state.min_humidity = normal_state.humidity;
      normal_state.min_humidity_temperature = normal_state.temperature;
    }
    if (normal_state.humidity > normal_state.max_humidity) {
      normal_state.max_humidity = normal_state.humidity;
      normal_state.max_humidity_temperature = normal_state.temperature;
    }
  }
}

void loop()
{
  // Handle client requests
  retained_state.program_counter = __LINE__;
  server.handleClient();
  
  // Handle client requests
  retained_state.program_counter = __LINE__;
  serial_server.handleClient();

  // Blink the LED
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval || currentMillis < previousMillis)
  {
    // Save the last time we blinked the LED
    previousMillis = currentMillis;

    if (late_setup_done == false) {
      late_setup();
    }
    
    retained_state.program_counter = __LINE__;
    updateSensorData();
    retained_state.program_counter = __LINE__;
    updateInfoString(true);
    // // Toggle the LED state
    // if (digitalRead(ledPin) == HIGH)
    // {
    //   digitalWrite(ledPin, LOW);
    //   updateInfoString(false);
    // }
    // else
    // {
    //   // digitalWrite(buzzerPin, HIGH);
    //   // delay(10);
    //   // digitalWrite(buzzerPin, LOW);
    //   digitalWrite(ledPin, HIGH);
    //   updateInfoString(true);
    // }
  }

  retained_state.program_counter = __LINE__;
  for (int direct_transitions=0; stateMachineFridge() && direct_transitions < 5; direct_transitions++){
  }

  retained_state.program_counter = __LINE__;
  for (int direct_transitions=0; stateMachineWiFi() && direct_transitions < 10; direct_transitions++){
  }
  
  retained_state.program_counter = __LINE__;
  bool previous_ota = normal_state.local_client;
  IPAddress client_ip = server.client().remoteIP();

  IPAddress softAPIP = WiFi.softAPIP();

  bool client_via_AP = client_ip[0] == softAPIP[0] && client_ip[1] == softAPIP[1] && client_ip[2] == softAPIP[2];  

  normal_state.local_client = client_ip.isSet() && !client_via_AP;
  if (!previous_ota && normal_state.local_client) {
    normal_state.current_client_time = millis();
  }
  retained_state.program_counter = __LINE__;
  if ( retained_state.enabled_ota && allowed_ota() ) {
    normal_state.handling_ota = true;
    ArduinoOTA.handle();
    normal_state.handling_ota = false;
  }
  retained_state.program_counter = __LINE__;
}

bool allowed_ota(){
  return normal_state.local_client || (get_viewer_time() > 3);
}
