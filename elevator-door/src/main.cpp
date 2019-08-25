//#define FRONT_DOOR 1
#define REAR_DOOR 1

#include "Arduino.h"

#include <ESP8266WiFi.h> // WIFI support
#include <ESP8266mDNS.h> // For network discovery
#include <WiFiUdp.h> // OSC over UDP
#include <ArduinoOTA.h> // Updates over the air

// OSC
#include <OSCMessage.h> // for sending OSC messages
#include <OSCBundle.h> // for receiving OSC messages

// I2C
#include <Wire.h>

// Display (SSD1306)
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Stepper Motor Control
#include <Tic.h>

// Distance Sensors
#include <VL53L0X.h>

// Port Expander
#include <Adafruit_MCP23008.h>

/* Constants */
#ifdef FRONT_DOOR
const char* ESP_NAME = "elev-frnt-door";
const char* DOOR_NAME = "front";
#endif
#ifdef REAR_DOOR
const char* ESP_NAME = "elev-rear-door";
const char* DOOR_NAME = "rear";
#endif

const char* WIFI_SSID = "skutta-net"; // network SSID (name)
const char* WIFI_PASSWORD = "#######"; // network password
const unsigned int OSC_PORT = 53000;

const int CLOSED_POSITION = 0;
const int OPEN_POSITION = 18600; // 18650
const int MAX_CONTIGUOUS_RANGE_ERROR_COUNT = 1;

const int HIGH_ACCURACY_TIMING_BUDGET = 200000;
const int HIGH_SPEED_TIMING_BUDGET = 26000; // 20000 min

const unsigned long DOOR_DWELL_1 = 5000; // 3000
const unsigned long DOOR_DWELL_2 = 2000;

//#define XSHUT_pin3 not required for address change
const int SENSOR2_XSHUT_PIN = D3;
const int SENSOR3_XSHUT_PIN = D4;

const int SENSOR1_ADDRESS = 43;
const int SENSOR2_ADDRESS = 42;
//const int SENSOR3_ADDRESS = 41; // Default

const unsigned long OSC_MESSAGE_SEND_INTERVAL = 200; // 200 ms

/* Variables */
enum class DoorState {Unknown, Calibrating, Closed, Closing, Manual, Open, Opening, Reopening, Waiting};
static const char *DoorStateString[] = {"Unknown", "Calibrating", "Closed", "Closing", "Manual", "Open", "Opening", "Reopening", "Waiting"};
DoorState doorState = DoorState::Unknown;
DoorState lastDoorState = DoorState::Unknown;

unsigned long openTimeout = 0;
unsigned long waitTimeout = 0;

volatile int encoderCount = 0; //This variable will increase or decrease depending on the rotation of encoder
int lastEncoderPosition = 0;

bool doorOpenReceived = false;
unsigned long oscSendTime;

/* Display */
SSD1306AsciiWire oled;
uint8_t rowHeight; // pixels per row.

/* WIFI */
char hostname[21] = {0};

/* OSC */
WiFiUDP Udp;
OSCErrorCode error;

String controllerHostname;
IPAddress controllerIp;
unsigned int controllerPort;

/* TIC */
TicSerial tic(Serial);

/* VL53L0X */
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

/* Port Expander */
Adafruit_MCP23008 mcp;

int contiguousRangeErrorCount = 0;
int lastRange = 0;
int getRange() {
  int range = 0;
  int range1 = _max(sensor1.readRangeContinuousMillimeters(), 30);
  int range2 = _max(sensor2.readRangeContinuousMillimeters(), 30);
  int range3 = _max(sensor3.readRangeContinuousMillimeters(), 30);
  
  // Calculate average range.  
  int averageRange = (int) ((range1 + range2 + range3) / 3);
  
  // Determine error condition
  int minRange = averageRange - 20;
  int maxRange = averageRange + 20;
  bool error = ((range1 > 1200) || (range2 > 1200) || (range3 > 1200) || 
                (range1 < minRange) || (range1 > maxRange) || 
                (range2 < minRange) || (range2 > maxRange) || 
                (range3 < minRange) || (range3 > maxRange));
                    
  // Return error if occurred 2 times in a row
  if (error) {
    contiguousRangeErrorCount++;
    if (contiguousRangeErrorCount >= MAX_CONTIGUOUS_RANGE_ERROR_COUNT) {
      range = -1;
    } else {
      range = lastRange;
    }
  } else {
    // No error
    contiguousRangeErrorCount = 0;
    range = lastRange = averageRange;
  }
  
  // Display Results if broken
  if (averageRange > 100 && range == -1) {
    char buffer [5];
    
    sprintf (buffer, "%4d", range1);
    oled.print(buffer);
    
    sprintf (buffer, "%4d", range2);
    oled.print(buffer);
    
    sprintf (buffer, "%4d", range3);
    oled.print(buffer);
    
    sprintf (buffer, "%4d", averageRange);
    oled.print(buffer);
  
    oled.println(F("*"));
  }
  
  return range;
}

int getRangePosition(int range) {
  if (range == -1) return -1;
  return map(range, 0, 900, 0, 18650);
}

void setEncoderPosition(int position) {
  encoderCount = map(position, 0, 1600, 0, 600); // Encoder 600p/r, Stepper 200steps/rev * 8
}

int getEncoderPosition() {
  return map(encoderCount, 0, 600, 0, 1600); // Encoder 600p/r, Stepper 200steps/rev * 8
}

void waitDoor(unsigned long timeout) {
  tic.deenergize();
  doorState = DoorState::Waiting;
  waitTimeout = millis() + timeout;
}

void calibrate() {
  
  oled.println(F("Calibrating..."));
  
  doorState = DoorState::Calibrating;
  
  // High Accuracy
  sensor1.setMeasurementTimingBudget(HIGH_ACCURACY_TIMING_BUDGET);
  sensor2.setMeasurementTimingBudget(HIGH_ACCURACY_TIMING_BUDGET);
  sensor3.setMeasurementTimingBudget(HIGH_ACCURACY_TIMING_BUDGET);
  
  // Give the Tic some time to start up.
  delay(500);

  tic.energize();
  
  // Door Calibration Settings
  tic.setStepMode(TicStepMode::Microstep8); // 1/8 step: 200*8 = 1600 steps per revolution
  tic.setCurrentLimit(1500);
  tic.setMaxSpeed(30000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
  tic.setMaxAccel(200000); // 10000 steps/sec
  tic.setMaxDecel(200000); // 10000 steps/sec

  int range = getRange();
  while((range = getRange()) == -1) {
    delay(100);
  }

  // if range is less than 100mm, move into position to accurately measure
  if (range < 100) {
    tic.haltAndSetPosition(0);
    tic.setTargetPosition(OPEN_POSITION / 2);
    tic.exitSafeStart();
    
    while (tic.getCurrentPosition() != tic.getTargetPosition()) {
      // Wait for in range position read
      delay(20);
      tic.resetCommandTimeout();
    }
    delay(1000);

    while((range = getRange()) == -1) {
      delay(100);
    }
  }

  int rangePosition = getRangePosition(range);
  setEncoderPosition(rangePosition);
  tic.haltAndSetPosition(rangePosition);
  tic.setTargetPosition(OPEN_POSITION);
  tic.exitSafeStart();
  while (tic.getCurrentPosition() != tic.getTargetPosition()) {
    // Wait for in range position read
    delay(20);
    tic.resetCommandTimeout();
  }
  
  tic.deenergize();
  
  // High Speed
  sensor1.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  sensor2.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  sensor3.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  
  // Wait for door to timeout then close
  waitDoor(DOOR_DWELL_1);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(D6)==LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void closeDoor() {
  // Allow the door to settle
  if (tic.getEnergized()) {
    tic.deenergize();
    delay(250);
  }
  
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(900); // 750 is the minimum required to mostly open the door
  tic.setMaxSpeed(30000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
  tic.setMaxAccel(100000); // 10000 steps/sec
  tic.setMaxDecel(100000); // 10000 steps/sec
  tic.haltAndSetPosition(getEncoderPosition());
  tic.setTargetPosition(CLOSED_POSITION);
  tic.exitSafeStart();
  
  doorState = DoorState::Closing;
}

void openDoor(bool reopen) {
  // Allow the door to settle
  if (tic.getEnergized()) {
    tic.deenergize();
    delay(250);
  }
  
  tic.energize();
  tic.setStepMode(TicStepMode::Microstep8);
  tic.setCurrentLimit(1500);
#ifdef FRONT_DOOR
  tic.setMaxSpeed(90000000); // ~11.25 revolutions (18000 steps) to open door. 2 second open time = 9000 steps/sec
#endif
#ifdef REAR_DOOR
  if (reopen == true) {
    tic.setMaxSpeed(90000000);
  } else {
    tic.setMaxSpeed(20000000);
  }
#endif
  tic.setMaxAccel(400000); // 10000 steps/sec
  tic.setMaxDecel(700000); // 10000 steps/sec
  tic.haltAndSetPosition(getEncoderPosition());
  tic.setTargetPosition(OPEN_POSITION);
  tic.exitSafeStart();

  doorState = (reopen == true) ? DoorState::Reopening : DoorState::Opening;
}

void receiveCallUp(OSCMessage &msg, int addrOffset){
  mcp.digitalWrite(3, HIGH); // UP
  mcp.digitalWrite(1, LOW); // Down
}

void receiveCallDown(OSCMessage &msg, int addrOffset){
  mcp.digitalWrite(3, LOW); // UP
  mcp.digitalWrite(1, HIGH); // Down
}

void receiveCallNone(OSCMessage &msg, int addrOffset){
  mcp.digitalWrite(3, LOW); // UP
  mcp.digitalWrite(1, LOW); // Down
}

void receiveDoorOpen(OSCMessage &msg, int addrOffset){
  // Clear call acceptance lights
  mcp.digitalWrite(3, LOW); // UP
  mcp.digitalWrite(1, LOW); // Down
  doorOpenReceived = true;
}

void receiveOSC(){
  OSCMessage msg;
  int size;
  if((size = Udp.parsePacket())>0){
    while(size--)
      msg.fill(Udp.read());
    if(!msg.hasError()){
      char buffer [32];
      msg.getAddress(buffer);
      oled.print(F("recv: "));
      oled.println(buffer);
      
      msg.route("/call/up",receiveCallUp);
      msg.route("/call/down",receiveCallDown);
      msg.route("/call/none",receiveCallNone);
      msg.route("/door/open",receiveDoorOpen);
    } else {
      error = msg.getError();
      oled.print(F("recv error: "));
      oled.println(error);
    }
  }
}

void sendControllerOSCMessage(const char* address) {
  oled.print(F("send: "));
  oled.println(address);

  OSCMessage msg(address);
  Udp.beginPacket(controllerIp, controllerPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void sendCallUp() {
  sendControllerOSCMessage("/call/up");
}

void sendCallDown() {
  sendControllerOSCMessage("/call/down");
}

void setup() {
  
  /* Serial and I2C */
  Serial.begin(9600);
  Wire.begin(D2, D1); // join i2c bus with SDA=D1 and SCL=D2 of NodeMCU

  delay(1000);

  /* Display */
  oled.begin(&Adafruit128x64, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  oled.setFont(System5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();
  rowHeight = oled.fontRows();

  /* Function Select */
  oled.println(ESP_NAME);
  
  /* WiFi */
  sprintf(hostname, "%s-%06X", ESP_NAME, ESP.getChipId());
  oled.print(F("WiFi: "));
  oled.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    oled.println(F("Connection Failed!"));
    delay(5000);
    ESP.restart();
  }
  //while (WiFi.status() != WL_CONNECTED)
  //{
  //  delay(500);
  //  // Serial.print(".");
  //}
  // Serial.println();

  /* UDP */
  Udp.begin(OSC_PORT);

  oled.println(WiFi.macAddress());
  oled.println(hostname);
  oled.print(WiFi.localIP());
  oled.print(F(":"));
  oled.println(Udp.localPort());

  /* OTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    oled.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    oled.println(F("End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    oled.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    oled.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      oled.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      oled.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      oled.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      oled.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      oled.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  /* mDNS */
  // Initialization happens inside ArduinoOTA;
  MDNS.addService(ESP_NAME, "udp", OSC_PORT);

  // Wait to view display
  delay(2000);
  
  // Discover Elevator Controller
  while (MDNS.queryService("elev-ctrl", "udp") == 0) {
    oled.println(F("find elev-ctrl"));
    ArduinoOTA.handle();
    delay(1000);
  }
  controllerHostname = MDNS.hostname(0);
  controllerIp = MDNS.IP(0);
  controllerPort = MDNS.port(0);

  oled.println(controllerHostname);
  oled.print(controllerIp);
  oled.print(F(":"));
  oled.println(controllerPort);
  
  /* TIC */
  // Set the TIC product
  tic.setProduct(TicProduct::T500);
 
  /* VL53L0X */
  // WARNING: Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(SENSOR2_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR3_XSHUT_PIN, OUTPUT);

  // Change address of sensor and power up next one
  // Sensor 1
  sensor1.setAddress(SENSOR1_ADDRESS);
  
  // Sensor 2
  pinMode(SENSOR2_XSHUT_PIN, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  sensor2.setAddress(SENSOR2_ADDRESS);
  
  // Sensor 3
  pinMode(SENSOR3_XSHUT_PIN, INPUT);
  delay(10);
  
  sensor1.init();
  sensor2.init();
  sensor3.init();
  
  // High Speed
  sensor1.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  sensor2.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  sensor3.setMeasurementTimingBudget(HIGH_SPEED_TIMING_BUDGET);
  
  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
  sensor3.setTimeout(500);
  
  // Start continuous back-to-back mode (take readings as fast as possible).  
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();

  /* encoder */
  // Set up pins
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);
  
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(D5), ai0, RISING);

  /* Port Expander (MCP23008) */
  mcp.begin(0); // 0x20
  
  mcp.pinMode(0, INPUT); // Down Button
  mcp.pullUp(0, HIGH);  // turn on a 100K pullup internally
  
  mcp.pinMode(1, OUTPUT);  // Down Acceptance Light
  mcp.digitalWrite(1, LOW);
  
  mcp.pinMode(2, INPUT); // Up Button
  mcp.pullUp(2, HIGH);  // turn on a 100K pullup internally
  
  mcp.pinMode(3, OUTPUT);  // Up Acceptance Light
  mcp.digitalWrite(3, LOW);

#ifdef FRONT_DOOR
  mcp.pinMode(4, OUTPUT);  // Down Lanturn
  mcp.digitalWrite(4, LOW);
  
  mcp.pinMode(5, OUTPUT);  // Up Lanturn
  mcp.digitalWrite(5, LOW);
#endif
#ifdef REAR_DOOR
  mcp.pinMode(6, OUTPUT);  // Door Frame
  mcp.digitalWrite(6, HIGH);
  
  mcp.pinMode(7, OUTPUT);  // Door Frame
  mcp.digitalWrite(7, HIGH);
#endif

  // Wait to view display
  delay(2000);
  
  /* calibrate */
  calibrate();
}

void loop() {
  ArduinoOTA.handle();
  receiveOSC();

  // Read buttons
  if (mcp.digitalRead(0) == LOW) { // Down Button
    sendCallDown(); // Send immediately, do not use timer loop
  }
  if (mcp.digitalRead(2) == LOW) { // Up Button
    sendCallUp(); // Send immediately, do not use timer loop
  }
  
  // Get range and position info
  int range = getRange();
  int rangePosition = getRangePosition(range);
  int encoderPosition = getEncoderPosition();
  int currentPosition = tic.getCurrentPosition();
  int targetPosition = tic.getTargetPosition();
  
  // Indicate if range error
  if (range == -1) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Deenergize if stopped
  bool stopped = currentPosition == targetPosition;
  if (stopped && tic.getEnergized()) {
    tic.deenergize();
  }
  
  // Position Correction
  int positionCorrection = currentPosition - encoderPosition;

  // Messages
  bool openDoorRequested = doorOpenReceived;
  doorOpenReceived = false;
  
  // Handle Door States
  if (doorState == DoorState::Waiting) {
    if (millis() > waitTimeout) {
      // Close the door
      closeDoor();
    }
    else if (range == -1 || openDoorRequested || (encoderPosition != lastEncoderPosition)) {
      waitDoor(DOOR_DWELL_2);
    }
  }
  else if (doorState == DoorState::Closing) {
    if (stopped) {
      doorState = DoorState::Closed;
    }
    else if ((range == -1 && encoderPosition > 1000) || // beam break - reopen
              positionCorrection < -64 || // door is being pushed - reopen
              openDoorRequested) { // open door requested
      openDoor(true);
    }
    else if (positionCorrection > 64) { // door is being pulled - wait
      waitDoor(DOOR_DWELL_2);
    }
  }
  else if (doorState == DoorState::Opening || doorState == DoorState::Reopening) {
    if (stopped || 
      encoderPosition > OPEN_POSITION) { // Door passed jam - wait
      //abs(positionCorrection) > 128) { // Door is being pushed or pulled - wait

#ifdef REAR_DOOR
      mcp.digitalWrite(6, HIGH); // Turn off EL wire
      mcp.digitalWrite(7, HIGH); // Turn off EL wire
#endif
      
      if(doorState == DoorState::Reopening) {
        waitDoor(DOOR_DWELL_2);
      } else {
        waitDoor(DOOR_DWELL_1);
      }
    }
  }
  else if (doorState == DoorState::Closed) {
    if (openDoorRequested) {
#ifdef REAR_DOOR
      mcp.digitalWrite(6, LOW); // Turn on EL wire
      mcp.digitalWrite(7, LOW); // Turn on EL wire
#endif
      openDoor(false);
    }
  }

  // Send door state once every [interval].  This adds reliability.
  if (millis() > oscSendTime) {
    if (doorState == DoorState::Closed) {
#ifdef FRONT_DOOR
      sendControllerOSCMessage("/door/closed/front");
#endif
#ifdef REAR_DOOR
      sendControllerOSCMessage("/door/closed/rear");
#endif
    } else {
#ifdef FRONT_DOOR
      sendControllerOSCMessage("/door/open/front");
#endif
#ifdef REAR_DOOR
      sendControllerOSCMessage("/door/open/rear");
#endif
    }
    oscSendTime = millis() + OSC_MESSAGE_SEND_INTERVAL;
  }
  
  // Print the door state
  if (doorState != lastDoorState) {
    oled.print(F("door: "));
    oled.println(DoorStateString[(int)doorState]);
  }

  lastDoorState = doorState;
  lastEncoderPosition = encoderPosition;
  tic.resetCommandTimeout();
}


