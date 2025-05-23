// Arduino Uno Code
// Uses Hardware Serial (Pins 0-RX, 1-TX) for communication with ESP32-CAM at 115200 baud.
// Handles servo control (Pins 12, 13), buzzer (Pin 9), and DS1302 RTC (Pins 5,6,7).
// Logs command reception times and inactivity.
// Sends "LOG_START:<timestamp>" and "LOG_STOP:<timestamp>" messages to ESP32-CAM.
// Internal debug messages from Arduino are minimized to avoid spamming ESP32-CAM.
// Servos auto-center on boot.
// PAN SERVO MOVEMENT IS INVERTED.

#include <Servo.h>
#include <ThreeWire.h>      // Required by RtcDS1302 library
#include <RtcDS1302.h>      // Library for DS1302 RTC

// --- Pin Definitions ---
const int PAN_SERVO_PIN = 12;   
const int TILT_SERVO_PIN = 13;  
const int BUZZER_PIN = 9;       
const int DS1302_RST_PIN = 5;   
const int DS1302_DAT_PIN = 6;   
const int DS1302_CLK_PIN = 7;   

// --- Servo Objects ---
Servo panServo;
Servo tiltServo;

// --- RTC Object Setup ---
ThreeWire myWire(DS1302_DAT_PIN, DS1302_CLK_PIN, DS1302_RST_PIN); 
RtcDS1302<ThreeWire> Rtc(myWire); 

// --- Variables ---
String inputString = "";         
boolean stringComplete = false;  
int panAngle = 90; // Stores the *commanded* pan angle             
int tiltAngle = 90; // Stores the *commanded* tilt angle             

// --- Inactivity Timeout Logic ---
unsigned long lastCommandTime = 0;       
const unsigned long inactivityTimeout = 10000; 
boolean systemIsActive = false;          

// --- Setup Function: Runs once when the Arduino starts ---
void setup() {
  Serial.begin(115200); 
  inputString.reserve(30); 

  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  
  // Initialize servos to the center position (panAngle is 90, so inverted is also 90)
  panServo.write(180 - panAngle); // Write the inverted angle for pan
  tiltServo.write(tiltAngle);     // Tilt remains as is
  // Serial.println("Arduino (USB Debug): Servos centered (pan inverted)."); 

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); 
  
  // Serial.println("Arduino (USB Debug): Initializing DS1302 RTC..."); 
  Rtc.Begin(); 

  if (Rtc.GetIsWriteProtected()) {
    // Serial.println("Arduino (USB Debug): RTC was write protected, enabling writing."); 
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning()) {
    // Serial.println("Arduino (USB Debug): RTC not running, setting time to compile time."); 
    char currDate[] = __DATE__;
    char currTime[] = __TIME__;
    Rtc.SetDateTime(RtcDateTime(currDate, currTime));
    Rtc.SetIsRunning(true); 
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now.Year() < 2024) { 
      // Serial.println("Arduino (USB Debug): RTC time seems invalid."); 
  }

  // Serial.print("Arduino (USB Debug): Current RTC Time: "); 
  // printAndFormatDateTime(now, NULL); 
  // Serial.println();
  
  Serial.println("Arduino: Setup Complete. Ready."); 
  lastCommandTime = millis(); 
  briefBeep(); 
}

// --- Loop Function: Runs repeatedly ---
void loop() {
  if (stringComplete) {
    RtcDateTime commandTime = Rtc.GetDateTime(); 

    if (!systemIsActive) { 
        // Serial.print("Arduino (USB Debug): Activity RESUMED at "); 
        // printAndFormatDateTime(commandTime, NULL);
        // Serial.println();
        
        char formattedTime[25];
        printAndFormatDateTime(commandTime, formattedTime); 
        Serial.print("LOG_START:"); 
        Serial.println(formattedTime); 
        
        systemIsActive = true;
    }
    lastCommandTime = millis(); 
    
    inputString.trim(); 
    // Serial.print("Arduino (Serial RX): Received '");
    // Serial.print(inputString); 
    // Serial.print("' at ");
    // printAndFormatDateTime(commandTime, NULL);
    // Serial.println();
    
    processCommand(inputString); 
    inputString = "";            
    stringComplete = false;      
  }

  if (systemIsActive && (millis() - lastCommandTime > inactivityTimeout)) {
    RtcDateTime stopTime = Rtc.GetDateTime();
    // Serial.print("Arduino (USB Debug): Activity STOPPED at "); 
    // printAndFormatDateTime(stopTime, NULL);
    // Serial.println();

    char formattedTime[25];
    printAndFormatDateTime(stopTime, formattedTime); 
    Serial.print("LOG_STOP:"); 
    Serial.println(formattedTime); 
        
    systemIsActive = false; 
  }
}

// --- Serial Event: Called automatically when data is available on Hardware Serial port ---
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    inputString += inChar;             
    if (inChar == '\n') {
      stringComplete = true; 
    }
  }
}

// --- Process Incoming Command from ESP32-CAM ---
void processCommand(String cmd) {
  // cmd is already trimmed
  if (cmd.startsWith("S")) { 
    int colonIndex = cmd.indexOf(':'); 
    if (colonIndex > 0) {
      String panStr = cmd.substring(1, colonIndex);
      String tiltStr = cmd.substring(colonIndex + 1);
      int newPanCmd = panStr.toInt(); // Commanded pan angle
      int newTiltCmd = tiltStr.toInt(); // Commanded tilt angle

      newPanCmd = constrain(newPanCmd, 0, 180);
      newTiltCmd = constrain(newTiltCmd, 0, 180); 

      // Invert the pan angle
      int actualPanToWrite = 180 - newPanCmd; 
      // Ensure the inverted angle is also within servo limits (it should be if original is)
      actualPanToWrite = constrain(actualPanToWrite, 0, 180);

      // Store the original commanded angles for state tracking if needed,
      // but write the possibly inverted angle to the servo.
      panAngle = newPanCmd; // Store the logical angle
      tiltAngle = newTiltCmd;

      panServo.write(actualPanToWrite); // Write the INVERTED pan angle
      tiltServo.write(newTiltCmd);      // Write the tilt angle as is

      // Serial.print("Arduino: Commanded Pan: "); Serial.print(newPanCmd); // Debug original command
      // Serial.print(" -> Writing to Servo: "); Serial.print(actualPanToWrite); // Debug inverted value
      // Serial.print(", Tilt: "); Serial.println(newTiltCmd);            
    } else {
      // Serial.println("Arduino: Invalid servo command format received."); 
    }
  } else {
    // Serial.print("Arduino (Serial RX): Received non-servo command or unhandled: "); Serial.println(cmd); 
  }
}

// --- Buzzer Control ---
void briefBeep() {
  digitalWrite(BUZZER_PIN, HIGH); delay(50); digitalWrite(BUZZER_PIN, LOW);  
}

// --- Helper function to print and optionally format date/time ---
void printAndFormatDateTime(const RtcDateTime& dt, char* buffer) {
    char tempBuffer[25]; 
    snprintf_P(tempBuffer, 
            countof(tempBuffer),
            PSTR("%02u/%02u/%04u,%02u:%02u:%02u"), 
            dt.Day(), dt.Month(), dt.Year(),
            dt.Hour(), dt.Minute(), dt.Second() );
    
    if (buffer != NULL) {
        strcpy(buffer, tempBuffer); 
    } else {
        // Serial.print(tempBuffer); 
    }
}
