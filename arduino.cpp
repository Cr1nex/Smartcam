#include <Servo.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>  

//Pin Definitions
//Servo Pins
const int PAN_SERVO_PIN = 12;   //PWM pin for Pan Servo
const int TILT_SERVO_PIN = 13;  //PWM pin for Tilt Servo
// Buzzer Pin
const int BUZZER_PIN = 9;       //Digital pin for Buzzer
//DS1302 RTC Pins
const int DS1302_RST_PIN = 5;   //CE/RST pin
const int DS1302_DAT_PIN = 6;   //IO/DAT pin
const int DS1302_CLK_PIN = 7;   //SCLK/CLK pin

//Servo Objects
Servo panServo;
Servo tiltServo;

//RTC Object Setup
//Initialize a ThreeWire interface for the RtcDS1302 library
ThreeWire myWire(DS1302_DAT_PIN, DS1302_CLK_PIN, DS1302_RST_PIN); 
RtcDS1302<ThreeWire> Rtc(myWire); //Create an RtcDS1302 object

//Variables
String inputString = "";         //A String to hold incoming data from ESP32-CAM
boolean stringComplete = false;  //Whether the string is complete
int panAngle = 90;               //Initial pan angle (center)
int tiltAngle = 90;              //Initial tilt angle (center)

//Inactivity Timeout Logic
unsigned long lastCommandTime = 0;       //Stores the millis() timestamp of the last command
const unsigned long inactivityTimeout = 10000; //10 seconds in milliseconds
boolean systemIsActive = false;          //Tracks if the system is currently considered active

//Setup Function: Runs once when the Arduino starts ---
void setup() {
  Serial.begin(115200); //For communication with ESP32-CAM AND for Arduino's debug prints
  inputString.reserve(30); 

  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);

  // Initialize servos to the center position (as per your sketch)
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); 

  Serial.println("Arduino: Initializing DS1302 RTC..."); //Debug for Arduino's monitor
  Rtc.Begin(); //Initialize the RTC

  //Check if the RTC is write-protected (optional, but good to know for DS1302)
  if (Rtc.GetIsWriteProtected()) {
    Serial.println("Arduino: RTC was write protected, enabling writing now.");
    Rtc.SetIsWriteProtected(false);
  }

  //Check if the RTC is running. If not, set the time.
  if (!Rtc.GetIsRunning()) {
    Serial.println("Arduino: RTC was not actively running, starting now and setting time to compile time.");
    //Use char arrays for __DATE__ and __TIME__ for RtcDateTime constructor
    char currDate[] = __DATE__;
    char currTime[] = __TIME__;
    Rtc.SetDateTime(RtcDateTime(currDate, currTime));
    Rtc.SetIsRunning(true); // Ensure the clock is running
  }

  // Optional: Force set time to compile time every boot (uncomment if needed)
  // Serial.println("Arduino: Setting RTC to compile time...");
  // char currDateForce[] = __DATE__;
  // char currTimeForce[] = __TIME__;
  // Rtc.SetDateTime(RtcDateTime(currDateForce, currTimeForce));

  RtcDateTime now = Rtc.GetDateTime();
  if (now.Year() < 2024) { // Or some other reasonable year like 2023
      Serial.println("Arduino: RTC time seems invalid (e.g., year < 2024). Consider re-setting time or checking battery.");
      // You might want to force-set it again here if the time is clearly wrong
      // char currDateInvalid[] = __DATE__;
      // char currTimeInvalid[] = __TIME__;
      // Rtc.SetDateTime(RtcDateTime(currDateInvalid, currTimeInvalid));
  }

  Serial.print("Arduino: Current RTC Time: ");
  printAndFormatDateTime(now, NULL); //Just print to Arduino Serial for local debug
  Serial.println();
  
  Serial.println("Arduino: Ready with DS1302. Waiting for commands from ESP32-CAM...");
  lastCommandTime = millis(); 
  briefBeep(); 
}

//Loop Function
void loop() {
  if (stringComplete) {
    RtcDateTime commandTime = Rtc.GetDateTime(); 

    if (!systemIsActive) { 
        Serial.print("Arduino: Activity RESUMED at "); 
        printAndFormatDateTime(commandTime, NULL);
        Serial.println();
        
        char formattedTime[25]; //Buffer for formatted time string
        printAndFormatDateTime(commandTime, formattedTime);
        Serial.print("LOG_START:"); //This goes to ESP32-CAM
        Serial.println(formattedTime); //This goes to ESP32-CAM
        
        systemIsActive = true;
    }
    lastCommandTime = millis(); 
    
    //Local debug of received command
    Serial.print("Arduino: Command '");
    inputString.trim(); 
    Serial.print(inputString); 
    Serial.print("' received at ");
    printAndFormatDateTime(commandTime, NULL); //Print time for this specific command
    Serial.println();
    
    processCommand(inputString); 
    inputString = "";            
    stringComplete = false;      
  }

  if (systemIsActive && (millis() - lastCommandTime > inactivityTimeout)) {
    RtcDateTime stopTime = Rtc.GetDateTime();
    Serial.print("Arduino: Activity STOPPED at "); 
    printAndFormatDateTime(stopTime, NULL);
    Serial.println();

    char formattedTime[25];
    printAndFormatDateTime(stopTime, formattedTime);
    Serial.print("LOG_STOP:"); //This goes to ESP32-CAM
    Serial.println(formattedTime); //This goes to ESP32-CAM
        
    systemIsActive = false; 
  }
}

//Serial Event
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    inputString += inChar;             
    if (inChar == '\n') {
      stringComplete = true; 
    }
  }
}

//Process Incoming Command
void processCommand(String cmd) {
  // cmd is already trimmed by the caller (in loop)
  if (cmd.startsWith("S")) { 
    int colonIndex = cmd.indexOf(':'); 
    if (colonIndex > 0) {
      String panStr = cmd.substring(1, colonIndex);
      String tiltStr = cmd.substring(colonIndex + 1);
      int newPan = panStr.toInt();
      int newTilt = tiltStr.toInt();
      newPan = constrain(newPan, 0, 180);
      newTilt = constrain(newTilt, 0, 180); 

      panAngle = newPan;
      tiltAngle = newTilt;
      panServo.write(panAngle);   
      tiltServo.write(tiltAngle); 

      Serial.print("Arduino: Moved to Pan: "); Serial.print(panAngle); 
      Serial.print(", Tilt: "); Serial.println(tiltAngle);            
    } else {
      Serial.println("Arduino: Invalid servo command format received."); 
    }
  } else {
    Serial.print("Arduino: Received non-servo command or echo: "); Serial.println(cmd); 
  }
}

//Buzzer Control
void briefBeep() {
  digitalWrite(BUZZER_PIN, HIGH); delay(50); digitalWrite(BUZZER_PIN, LOW);  
}

//Helper function to print and optionally format date/time
void printAndFormatDateTime(const RtcDateTime& dt, char* buffer) {
    char tempBuffer[25]; 
    snprintf_P(tempBuffer, 
            countof(tempBuffer),
            PSTR("%02u/%02u/%04u,%02u:%02u:%02u"), //CSV friendly: Date,Time
            dt.Day(), dt.Month(), dt.Year(),
            dt.Hour(), dt.Minute(), dt.Second() );
    
    if (buffer != NULL) {
        strcpy(buffer, tempBuffer);
    } else {
        Serial.print(tempBuffer); //For Arduino's local debugging
    }
}
