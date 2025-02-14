// LINES 110 & 111 ARE UNTESTED IN INTEGRATED TEST

#include <SoftwareSerial.h>
#include <SD.h>

// GPIO definitions
#define RX_RYLR       7
#define TX_RYLR       6
#define D4184A        3
#define D4184B        5
#define LOAD          A5
#define INBUILT_LED   13
#define PRESSURE      A0

// State Machine Definition
typedef enum {
  SAFE,
  ARMED,
  LAUNCHED,
  FAILURE
} STATE;
STATE currentState = SAFE;

// Object Instantiations
SoftwareSerial RYLR(RX_RYLR, TX_RYLR);
// Variable Definitions
File logFile;
float loadCellReading, weight, calRead, calFactor = 1.29;
float startTime = 0, timeStamp;
String currState, response, logString;
int pressureAdc;
// Prototype Function Definitions
float mapVtoKG(float read) { // Not mapped properly yet
  return (calFactor*read*500.0)/1023.0;
}

// void getCalibrationFactor() {
//   calRead = 4.4;
//   Serial.print("Value according to analog input: ");
//   getData();
//   calFactor = (float)calRead/(float)weight;
//   Serial.println("Calibration factor: " + String(calFactor));
//   Serial.print("Calibrated value: ");
//   getData();
//   sendState("CALIBRATED. FACTOR: "+String(calFactor));
//   Serial.println("CALIBRATION COMPLETE\n");
// }

// void getTareValue() {
//   tareValue = analogRead(LOAD);
//   // tareValue = mapVtoKG(tareValue);
//   Serial.println("Tare value: " + String(tareValue)); 
// }

void getData() {
  loadCellReading = analogRead(LOAD);
  weight = mapVtoKG(loadCellReading);
  timeStamp = (millis() - startTime)/1000;
  // logString = String(timeStamp)+":"+String(loadCellReading);
  // Serial.println(weightString+":"+weight);
  pressureReading = analogRead(PRESSURE);
  logString = String(timeStamp)+":"+String(loadCellReading)+", pressureADC: "+String(pressureReading);
  Serial.println(logString+":"+weight);
  delay(10);
}

void logData() {
  logFile = SD.open("loadcell.txt", FILE_WRITE);
  logFile.print(logString);
  logFile.print(":");
  logFile.print(weight);
  logFile.close();
}
void sendData() {
  String transmit = "AT+SEND=0," + String(logString.length()) + "," + logString + "\r\n";
  //added a flushing buffer at 22:16 on 6Oct24
  RYLR.flush();
  //yep thassit, ek receiving side pe bhi clear buffer rakha hai 
  RYLR.print(transmit);
  delay(10);
}

String parseRYLR(String input) {
  int start = input.indexOf(',') + 1;
  start = input.indexOf(',', start) + 1;
  int end = input.indexOf(',', start);
  String parsed = input.substring(start, end);
  parsed.trim();
  return parsed;
}

void sendState(String currState) {
  currState = "TESTBED STATE: " + currState;
  currState = "AT+SEND=0,"+ String(currState.length()) + "," + currState + "\r\n";
  RYLR.print(currState);
  delay(10);
}

void checkInput(String receive) {
  if (receive == "ARM" && currentState == SAFE) {
    Serial.println("CURRENT STATE: ARMED. ");
    currentState = ARMED;
    sendState("ARMED");
    // getTareValue();
    startTime = millis();
    return;
  }
  else if (receive == "DISARM" && currentState == ARMED) {
    Serial.println("CURRENT STATE: SAFE. ");
    currentState = SAFE;
    sendState("SAFE");
    return;
  }
  else if (receive == "LAUNCH" && currentState == ARMED) {
    Serial.println("CURRENT STATE: LAUNCHED. ");
    currentState = LAUNCHED;
    sendState("LAUNCHED");
    return;
  }
  else {
    //Serial.println("INVALID INPUT PROVIDED. ");
    return;
  }
}

void performOperations() {
  switch(currentState) {
    case SAFE:
      break;
    case ARMED:
      getData();
      logData();
      // digitalWrite(INBUILT_LED, ~digitalRead(INBUILT_LED));
      // delay(100);
      break;
    case LAUNCHED:
      // digitalWrite(INBUILT_LED, HIGH);
      Serial.println("D4184s LATCHED");
      digitalWrite(D4184A, HIGH);
      digitalWrite(D4184B, HIGH);
      unsigned long int start = millis();
      bool flag = false;
      while(1) {
        getData();
        logData();
        sendData();
        if (!flag && millis() - start > 10000) {
          digitalWrite(D4184A, LOW);
          digitalWrite(D4184B, LOW);
          delay(10);
          sendState("UNLATCHED");
          Serial.println("D4184s UNLATCHED");
          flag = true;
        }
      }
      break;
    case FAILURE:
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(D4184A, OUTPUT);
  pinMode(D4184B, OUTPUT);
  pinMode(LOAD, INPUT);
  digitalWrite(D4184A, LOW);
  digitalWrite(D4184B, LOW);

  pinMode(INBUILT_LED, OUTPUT);
  digitalWrite(INBUILT_LED, LOW);

  //RYLR setup
  RYLR.begin(57600);

  //Load Cell setup
  Serial.println("\nLoad Cell Comm. Initialised.");

  //SD Card setup
  Serial.println("Serial Comm. Initialised.");
  if (!SD.begin()) {
    Serial.println("SD card initialisation failed.");
    return;
  }
  logFile = SD.open("datatest.txt", FILE_WRITE);
  if (!logFile) {
    Serial.println("Couldn't open log file");
  } 
  else {
    Serial.println("Logging to SD card...\n");
  }

  // getCalibrationFactor();
  // Tare the Loadcell
  // getTareValue();
  sendState("SAFE");
  Serial.println("TESTBED SETUP COMPLETE.");
}

void loop() {
  if (RYLR.available()) {
    response = RYLR.readStringUntil('\n');
    response = parseRYLR(response);
    // if (response.length() > 3) {
      Serial.println("RESPONSE: " + response);
      checkInput(response);
    // }
  }
  performOperations();
}
