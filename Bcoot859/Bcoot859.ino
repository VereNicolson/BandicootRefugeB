/*
  Version 859. (Bcoot858 is backup not to modify (not even tidy up))  28/11/18  working OK  as Bcoot857 @ 28//11/18 so copied to be Bcoot858
  Platformio throws  warnings about overflow from chip numbers.
  Platformio suggests another bracket at (end of input" at end of loop, but calls it an error if you put it in??
  Has functioned perfectly for a few hours, so any intermittent problems are attributable to one individual sensor.
  DON"T CHANGE THE CODE TO CHASE MINOR PROBLEMS.


  Modifications to this version keeping "858" as backup



*/
/* Pin use by other hardware
  0 is hardware serial Rx keep free for serial monitor/USB
  1 is hardware serial Tx keep free for serial monitor/USB
  2 available  const byte IR_inPowerPin     = 2;
  3 available const byte IR_outPowerPin     = 3;
  4 available const byte doorSafetyPin  = 4;                    // the door safety switch pin senses the door is unobstructed and has fully closed
  5  available const byte SERVO_PIN      = 5;                    //Servo, attaches to    5;    //to avoid timer related resource clashes
  6 available const byte x     = 6;                     //Digital sensor input
  7 available const byte x    = 7;                    //Digital sensor input
  8  const byte IC Rx     =  8;                AltSoftSerial uses pin 8
  9 const byte IC Tx     =  9;                AltSoftSerial uses pin 9
  10 const byte  chipSelect    = 10;             // for the data logging shield,  SD card requires pin #10 as Hardware pin = CS pin
  //new SD library prefers pin4 for CS
  11 const int  MOSI       = 11;             These 3 are declared within the SD library so don't use these pins for anything else
  12const int  MISO       = 12;             These 3 are declared within the SD library so don't use these pins for anything else
  13 const int  SCK        = 13;             These 3 are declared within the SD library so don't use these pins for anything else
  A0
  A1
  A5
*/



//Include libraries
#include <Arduino.h>

#include <AltSoftSerial.h>     //Faster/newer than SoftwareSerial
#include <ServoTimer2.h>      //Chosen to avoid timer conflict in earlier version, works OK
#include <SD.h>              // Library looks after SD card
#include <Wire.h>           // needed for RTC communication
#include "RTClib.h"        // Looks after clock, the Bcoot RTC is probably a DS3231 definitely not DS1307

// Settings for this box.
//#define DEBUG_SERIAL 1 // Enable serial output
const byte boxID = 1; // Unique box ID

// Our global states the app can be in.
#define STATE_IDLE_LISTENING_FOR_IR 0
#define STATE_IR_IN_IS_TRIGGERED 1
#define STATE_IR_OUT_IS_TRIGGERED 2
#define STATE_RFID_READING 3
#define STATE_RFID_SUCCESS 4
#define STATE_LOG_EXIT 5
#define STATE_DOOR_CYCLING 6
int STATE_current = STATE_IDLE_LISTENING_FOR_IR;

// Settings for RFID reader:
//Declare the "non-hardware" serial
AltSoftSerial RFID_serial;
//Short Term Memory, expect OK so care if using any other comand that could give OK response
char RFID_configureTagType[5] = {'S', 'T', '2', 13};
// Read animal chip,   expect "String data response"
char RFID_readTag[5] = {'R', 'A', 'T', 13,};
// Deactivates reader circuit at setup,and end of function expect "OK" response
char RFID_deactivateCircuit[5] = {'S', 'R', 'D', 13,};
//Activates reader circuit, expect "OK" response
char RFID_activateCircuit[5] = {'S', 'R', 'A', 13,};
char RFID_currentTag[12] = {000000000000};                       //Holds the PIT ID as a char, as it comes over serial
int RFID_currentTagPos = 0;

#define RFID_STATE_OFF 1
#define RFID_STATE_INIT 2
#define RFID_STATE_READING 3
#define RFID_STATE_GOT_VALUE 4

int RFID_state = RFID_STATE_OFF;
unsigned long RFID_state_time = millis();

// Settings for door (servo and safety switch):
ServoTimer2 DOOR_servo1;                             //declare the servo that will open the door
const byte DOOR_safetyPin = 4;                    // the door safety switch pin senses the door is unobstructed and has fully closed
const byte DOOR_servoPin = 5;                    //Servo, attaches to    5;    //to avoid timer related resource clashes
int DOOR_isClosed = 1;                     // HIGH if door tightly shut
int DOOR_servoAngleOpen = 1600;                        //global to share with door cycle and training
int DOOR_servoAngleShut = 950;                       //global to share with door cycle and training  **COMMENT OUT IN FIELD sketch**


#define DOOR_STATE_INITIALISE 0
#define DOOR_STATE_OPENING 1
#define DOOR_STATE_CLOSING 2
#define DOOR_STATE_CHECKING 3
#define DOOR_STATE_WAITING 4
#define DOOR_STATE_CLOSED 5

int DOOR_state = DOOR_STATE_INITIALISE;
unsigned long DOOR_state_time = millis();
int doorTight = HIGH;
int countDoorTries = 0;                             //

// Settings for RTC:
RTC_DS1307 RTC;                                // define the Real Time Clock object

// Settings for SD card.
File SDCARD_logfile;                                   // this wont be the actual name of the file, a placeholder
const byte SDCARD_chipSelectPin = 10;             // for the data logging shield, we use digital pin 10 for the SD cs line. SD card requires pin #10 as Hardware pin

#define SDCARD_STATE_OFF 0
#define SDCARD_STATE_WORKING 1
#define SDCARD_STATE_ERROR 2

int SDCARD_state = SDCARD_STATE_OFF;
unsigned long SDCARD_state_time = millis();

#define LOG_EXIT_TIME 0
#define FLUSH_TIME_EXIT 1
#define OPEN_DOOR_FOR_EXIT 2

int EXIT_Log_Door = LOG_EXIT_TIME;

// Settings for IR readers:
const byte IR_inReaderPin = 6;                   //  for inbound SharpIR
const byte IR_outReaderPin = 7;                  // for outbound SharpIR
const byte IR_inPowerPin = 2;                    //Switch IRs to save power, and allow settle times
const byte IR_outPowerPin = 3;                   // Switch IRs to save power
int proximityInbound1 = HIGH;    // If there's anything within 10cm of sensor, this goes low.                  // made these 4 global, works
int proximityInbound2 = HIGH;    // Second reading of the same sensor above (50ms later)
int proximityOutbound1 = HIGH;
int proximityOutbound2 = HIGH;                              //

//sub switch Inbound_Sensor_Sequence                        //sub switches need distinct labels
#define POWER_SAVE 10
#define ENABLE_INBOUND_SENSOR 11
#define FIRST_INBOUND_READ_DECISION 12
int Inbound_Sensor_Sequence = 10;                       // add ten to label #, cant use same number for nested switch as in the one it is nested in!!!


//sub switch Outbound_Sensor_Sequence         //sub switches need distinct labels, cant use same number for nested switch as in the one it is nested in!!!
#define POWER_SAVE_OUT 30              // add 20 to label # to differentiate from other switches,
#define ENABLE_OUTBOUND_SENSOR 31
#define FIRST_OUTBOUND_READ_DECISION 32
int Outbound_Sensor_Sequence = 30;


// cases for IR_cycle
#define IR_INITIATE 0
#define IR_STATE_READ_IN1 1
#define IR_STATE_READ_IN2 2
#define IR_STATE_READ_OUT1 3
#define IR_STATE_READ_OUT2 4


int IR_state = IR_INITIATE;
unsigned long IR_state_time = millis();

bool IR_checkStatusIn = false ;                                     // bool gets value set in
bool IR_checkStatusOut = false ;

//Debugging serial
int debugByte = 1;                              // read from serial
int deBugByte1 = HIGH;                             //debugging int variables generated in switch debugByte
int deBugByte2 = HIGH;
int deBugByte3 = HIGH;
int deBugByte4 = HIGH;
int deBugByte5 = HIGH;

//END DEFINES ***************           END DEFINES ***************           END DEFINES ***************           END DEFINES ***************

void setup() {
  Serial.println(F("setup() - booting."));
  Serial.begin(9600);
  Serial.println(F("setup() - starting RFID."));
  RFID_serial.begin(9600);
  Serial.println(F("setup() - setting pin status."));

  pinMode(IR_inPowerPin, OUTPUT);         //2
  pinMode(IR_outPowerPin, OUTPUT);        //3
  pinMode(DOOR_safetyPin, INPUT_PULLUP);  //4
  pinMode(IR_inReaderPin, INPUT_PULLUP);         //6
  pinMode(IR_outReaderPin, INPUT_PULLUP);        //7
  pinMode(SDCARD_chipSelectPin, OUTPUT);  //10


  Serial.println(F("setup() - starting SD card."));
  if (!SD.begin(SDCARD_chipSelectPin)) {
    Serial.println(F("setup() - warning! Couldn't connect to SD card."));
    SDCARD_state = SDCARD_STATE_ERROR;
  } else {
    Serial.println(F("setup() - SD card connected.."));
    SDCARD_state = SDCARD_STATE_WORKING;

    char SDCARD_filename[] = "0BOXFL00.CSV";
    //Adds box number to 1st place in "SDCARD_filename" will only work for 9 boxes
    SDCARD_filename[0] = boxID + '0';
    for (uint8_t i = 0; i < 100; i++) {
      SDCARD_filename[6] = i / 10 + '0';
      SDCARD_filename[7] = i % 10 + '0';
      if (!SD.exists(SDCARD_filename)) {
        SDCARD_logfile = SD.open(SDCARD_filename, FILE_WRITE);
        break;
      }
    }
    Serial.println(F("setup() - writing to file: "));
    Serial.println(SDCARD_filename);

    SDCARD_logfile.print("Date ");
    SDCARD_logfile.print(",");
    SDCARD_logfile.print("Date ");
    SDCARD_logfile.print(",");
    SDCARD_logfile.print("Time ");
    SDCARD_logfile.print(",");
    SDCARD_logfile.print("Time ");
    SDCARD_logfile.print(",");
    SDCARD_logfile.print("Tag number");
    SDCARD_logfile.print(",");
    SDCARD_logfile.print("Entry or Exit ");
    SDCARD_logfile.print(",");
    SDCARD_logfile.println(SDCARD_filename);
    delay(80);
    SDCARD_logfile.flush();                                                     // this saves headings to SD card file
  }

  Serial.println(F("setup() - starting RTC"));
  Wire.begin();
  if (!RTC.begin()) {
    Serial.println(F("setup() - Failed to start RTC"));
  }

  Serial.println(F("setup() - Starting RFID, configuring tag, expecting OK response."));
  RFID_serial.write(RFID_configureTagType);
  unsigned long RFID_boot_delay = millis() + 2000;
  while (millis() < RFID_boot_delay) {
    for (int readByte = 0; readByte < RFID_serial.available(); readByte++) {
      Serial.println(F("setup() - init RFID and received byte: "));
      char bytefound = RFID_serial.read();
      Serial.println(bytefound);
    }
  }
  // RFID_serial.write (SRD);          // send code for Deactivate RF for power saving  ************



  Serial.println(F("setup() - Starting IR power."));
  digitalWrite(IR_inPowerPin, LOW);
  digitalWrite(IR_outPowerPin, LOW);

  Serial.println(F("setup() - Starting DOOR servo."));
  DOOR_servo1.attach(DOOR_servoPin);
  DOOR_servo1.write(DOOR_servoAngleOpen);
  delay(500);
  DOOR_servo1.write(DOOR_servoAngleShut);
  delay(500);
  DOOR_servo1.detach();

  Serial.println(F("setup() - complete"));
}                                             // END SETUP   END SETUP  END SETUP  END SETUP  END SETUP  END SETUP  END SETUP

//________________________Doors_______________________________Doors__________________________Doors__________________________Doors__________________________
//Doors__________________________Doors__________________________Doors__________________________Doors__________________________Doors_________________________
void DOOR_trigger() {                                 //this function debugged and works including safety switch
  switch (DOOR_state) {

    case DOOR_STATE_INITIALISE:     //label 0
      Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_INITIALISE"));                       //debug
      DOOR_state = DOOR_STATE_OPENING;          //so next loop state will be DOOR_state-DOOR_STATE_OPENING
      DOOR_servo1.attach(DOOR_servoPin);                              // door servo has been attached,
      DOOR_state_time = millis();               // reinitialises timer after commited to case DOOR_state-DOOR_STATE_OPENING

      break;

    case DOOR_STATE_OPENING:                                      //label 1
      Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_OPENING"));                       //debug
      if ( millis() - DOOR_state_time > 500) {                                // allow 2sec for servo sweep
        DOOR_servo1.write(DOOR_servoAngleOpen);
        DOOR_state = DOOR_STATE_CLOSING;
        DOOR_state_time = millis();               // reinitialises timer after commited to case transition
      }

      break;

    case DOOR_STATE_CLOSING:                                     //label 2
      Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_CLOSING"));                       //debug
      if ( millis() - DOOR_state_time > 5000) {             // allow 2 sec for servo sweep and animal to pass through
        DOOR_state = DOOR_STATE_CHECKING;
        DOOR_servo1.write(DOOR_servoAngleShut);
        DOOR_state_time = millis();               // reinitialises timer after commited to case transition
      }

      break;

    case DOOR_STATE_CHECKING:                                     //label 3
      Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_CHECKING"));                       //debug
      if (countDoorTries >= 3 ) {
        DOOR_state = DOOR_STATE_WAITING;
        Serial.println(F("door has cycled 3 times to allow escape, anything animate should be gone, close to save power"));                       //debug
        DOOR_state_time = millis();               // reinitialises timer after commited to case transition
      }

      if (millis() - DOOR_state_time > 2000) {             //servo and door bounce so need this time before check
        doorTight = digitalRead (DOOR_safetyPin);          // read door switch should be HIGH once door closed
      }

      //****** debug
      if (deBugByte5 == LOW) {
        doorTight = deBugByte5;                   //debug *****  type 'g' to make doortight low till type 'h'
      }
      // debug


      if (millis() - DOOR_state_time > 2050) {    // another 50ms to read  nest ifs so all follows this
        if  (doorTight == LOW) {                         // i.e. door can't shut properly in 1 seconds
          DOOR_state = DOOR_STATE_INITIALISE ;                            // cycle door again to free whatever is blocking door.
          Serial.println(F("Door is obtructed, cycle door to allow escape"));                       //debug
          countDoorTries ++;                     // increment global variable to hold number of tries, must be global or will clear when case changes
          Serial.print(F("DoorTight is;   "));
          Serial.println(doorTight);
          Serial.print(F("opening door again for this many times out of 3;    "));
          Serial.println(countDoorTries);
        }


        else {                  //i.e. door is shut tight, DOOR_state is HIGH
          DOOR_state = DOOR_STATE_WAITING;
          DOOR_state_time = millis();               // reinitialises timer after commited to case transition
        }



      }


      break;

    case DOOR_STATE_WAITING:                                        // label 4
      Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_WAITING"));                       //debug
      //a delay to allow animal moving inside the box to be initially ignored for first 15 secs rather than triggering a series of door cycles , scans & time stamps
      DOOR_servo1.detach();
      if (millis() - DOOR_state_time > 5000) {                            //make this 15 sec when finished testing
        DOOR_state = DOOR_STATE_CLOSED;                         // transition to case 5 DOOR_STATE_CLOSED

      }

      break;

    case DOOR_STATE_CLOSED:                                   //label 5   this case finalises, resets and changes global back to monitoring IRs
      Serial.println(F("Switch DOOR_state has reached case DOOR_STATE_CLOSED"));                       //debug
      Serial.println(F(" STATE_CYCLING_DOORS now transitioning to STATE_IDLE_LISTENING_FOR_IR   "));
      STATE_current = STATE_IDLE_LISTENING_FOR_IR;                    //next global state, doing IRs not Doors
      countDoorTries = 0;                                             //reset  variable that counts how many door try attempts
      break;


  }                                             //end switch
}                                             //end function


void LOG_date_time () {                   //Logs date,time to SD card
  Serial.println(F("LOG_date_time - Logging date to SD card"));
  DateTime now = RTC.now();                               // fetch the time, time won't increment without this  //temp swapout  DateTime now = RTC.now();
  SDCARD_logfile.print("Date");
  SDCARD_logfile.print("");
  SDCARD_logfile.print(",");
  SDCARD_logfile.print("");
  SDCARD_logfile.print('"');
  SDCARD_logfile.print(now.day(), DEC);
  SDCARD_logfile.print("/");
  SDCARD_logfile.print(now.month(), DEC);
  SDCARD_logfile.print("/");
  SDCARD_logfile.print(now.year(), DEC);
  SDCARD_logfile.print('"');
  SDCARD_logfile.print(",");

  SDCARD_logfile.print("Time");
  SDCARD_logfile.print("");
  SDCARD_logfile.print(",");
  SDCARD_logfile.print("");
  SDCARD_logfile.print(now.hour(), DEC);
  SDCARD_logfile.print(":");
  SDCARD_logfile.print(now.minute(), DEC);
  SDCARD_logfile.print(":");
  SDCARD_logfile.print(now.second(), DEC);         //Excell is chopping off the seconds in cells but notepad and the excel formula bar show correctly.Excel's problem not Arduino
  SDCARD_logfile.print('"');
  SDCARD_logfile.print(", ");
}



void LOG_time_and_open_door_for_exit() {             //*****
  Serial.println(F("fnxn LOG_time_open_door_for_exit()-has been reached"));
  switch (EXIT_Log_Door) {                                                  //
    case LOG_EXIT_TIME:                                                     //label 0
      LOG_date_time();
      SDCARD_logfile.print("");
      SDCARD_logfile.print(",");
      SDCARD_logfile.println("This was an exit. ");
      Serial.println(F("fnxn LOG_time_and_open_door_for_exit()....logging time for an exit to SD card"));
      EXIT_Log_Door = FLUSH_TIME_EXIT;

      break;

    case FLUSH_TIME_EXIT:                                                       // label1
      Serial.println(F("flush file to SD card"));
      SDCARD_logfile.flush();                //Saves line to current file
      EXIT_Log_Door = OPEN_DOOR_FOR_EXIT;
      break;

    case OPEN_DOOR_FOR_EXIT:
      Serial.println(F("Cycle door for an exit"));
      STATE_current = STATE_DOOR_CYCLING;                                      //commit to state transition
      DOOR_state = DOOR_STATE_INITIALISE;                                     //gets read once after commit



  }
}

//______________________IR_________________________IR________________________IR_________________________IR_________________________IR_________________________IR___________________
//______________________IR_________________________IR________________________IR_________________________IR_________________________IR_________________________IR___________________

//27/11/18 version of Bcoot857
  void cycle_IRs() {                              //still good to save power even with digital sensors

  switch (IR_state) {                           //times work cycle of IRs and reads appropriate times,

    case IR_INITIATE:                                                       //label 0
      Serial.println (F("Cycle_IRs reached case IR_INITIATE    "));                          //Debug
      IR_state = IR_STATE_READ_IN1;
      Inbound_Sensor_Sequence = POWER_SAVE;
      IR_state_time = millis();                                                        //global time variable, 1st time set , subsequent ones in case

      break;

    case IR_STATE_READ_IN1:                                            //label 1
      Serial.print (F("Cycle_IRs reached case IR_STATE_READ_IN1    "));                          //Debug
      // IR_state_time = millis();                                           // can't have this here, resets every time thru switch and noting ever acheives an "if" time threshold


      switch (Inbound_Sensor_Sequence ) {                                     //SubSwitch nested || label 10

        case POWER_SAVE:                                                      //SubSwitch case
        Serial.println (F(" POWER_SAVE:    "));                          //Debug
          digitalWrite(IR_inPowerPin, LOW);                                      // Don't power IRs yet, power saving 200ms off
          digitalWrite(IR_outPowerPin, LOW);
          if   (millis() - IR_state_time > 200) {
            Inbound_Sensor_Sequence = ENABLE_INBOUND_SENSOR ;
            IR_state_time = millis();
          }

          break;

        case ENABLE_INBOUND_SENSOR:                                                     //SubSwitch case | label 11
        Serial.println (F(" ENABLE_INBOUND_SENSOR:    "));                          //Debug
          digitalWrite(IR_inPowerPin, HIGH);                                      //GPIO pin  still enables inbound IR
          digitalWrite(IR_outPowerPin, LOW);                                      //   outbound IR disabled
          if (millis() - IR_state_time > 50) {
            Inbound_Sensor_Sequence = FIRST_INBOUND_READ_DECISION;
            IR_state_time = millis();
          }

          break;

        case FIRST_INBOUND_READ_DECISION :                                          //SubSwitch case  | label 12
        Serial.println (F("  FIRST_INBOUND_READ_DECISION :  "));                          //Debug
          digitalWrite(IR_inPowerPin, HIGH);                                      //GPIO pin  still enables inbound IR  ?????
          digitalWrite(IR_outPowerPin, LOW);                                      //   outbound IR disabled             ?????
          proximityInbound1 = digitalRead(IR_inReaderPin);                // Read the inbound IR every loop, default is HIGH, LOW if proximity <10cm,
          Serial.print (F("proximityInbound1 ==  "));                          //Debug
          Serial.println (proximityInbound1);                          //Debug

          if (proximityInbound1 == LOW) {                                   //
            Serial.println (F("something is close to Inbound sensorsomething is close to Inbound sensorsomething is close to Inbound sensorsomething is close to Inbound sensor    "));        //Debug
            IR_state = IR_STATE_READ_IN2;                                   // detected proximity once so go to next case to confirm
            IR_state_time = millis();                                       //reset time after comit to next case
          }

      else if  (millis() - IR_state_time > 150) {                                     //NOT ow and times out
          Outbound_Sensor_Sequence = POWER_SAVE_OUT;

            IR_state = IR_STATE_READ_OUT1;                                               // commit to case change
            Serial.println (F(" > 150ms, leaving FIRST_INBOUND_READ_DECISION Now going to IR_STATE_READ_OUT1  "));                          //Debug
            IR_state_time = millis();                                             // after commit this resets time for next case
      }
/*
          else if  (millis() - IR_state_time > 150) {                                     //NOT low and times out
             Outbound_Sensor_Sequence = POWER_SAVE_OUT;                                   // start in this sub-case when transitioned
            IR_state = IR_STATE_READ_OUT1;                                               // Nothing comming inbound so go check for outbound, commit to case transition
             IR_state_time = millis();                                             // after commit this resets time for next case
            Serial.println (F("case FIRST_INBOUND_READ_DECISION : has timed out now without a LOW  go to IR_STATE_READ_OUT1: subcase POwER_SAVE_OUT "));                          //Debug
            proximityInbound1 = HIGH;                                       //reset now because not used in next case
          }
*/
          break;                                    // last break of sub switch
      }                                             // close Inbound_Sensor_Sequence subcase
          break;                                    // break for case IR_STATE_READ_IN1




        case IR_STATE_READ_IN2:                                                     //label 2
          // this case is to confirm contact, proximityInbound1 will already  be LOW
          Serial.println (F("Cycle_IRs reached case IR_STATE_READ_IN2    "));                          //Debug
          if (millis() - IR_state_time > 50) {                                  //50ms time to enable and settle sensor, then start read
            proximityInbound2 = digitalRead(IR_inReaderPin);                // read sensor,default is HIGH, LOW if proximity <10cm,
          }



          if (proximityInbound2 == LOW) {                                   // second read 50ms later confirms proximity and
            STATE_current = STATE_IR_IN_IS_TRIGGERED;           //this transitions without condition to State RFID reading which calls RFID_checkForTag();
            Serial.println(F("State transition statement, IDLE_LISTENING_FOR_IR -> IR_IN_IS_TRIGGERED"));                          //Debug*****
            digitalWrite(IR_inPowerPin, LOW);                                      // Power down IRs
            digitalWrite(IR_outPowerPin, LOW);
            proximityInbound1 = HIGH;                                               //reset both inbound proximity variable,
            proximityInbound2 = HIGH;
          }

          else if ( millis() - IR_state_time > 150) {       // no confirmation in another 100ms consider false and go to outbound
            IR_state = IR_STATE_READ_OUT1;
            IR_state_time = millis();
            Outbound_Sensor_Sequence = POWER_SAVE_OUT;
            Serial.println(F(" IR_STATE_READ_IN2:  has timed out now without 2nd LOW"));                          //Debug
            digitalWrite(IR_inPowerPin, LOW);                                      //Power down IRs
            digitalWrite(IR_outPowerPin, LOW);
            proximityInbound1 = HIGH;                                                                   //reset both inbound proximity variable
            proximityInbound2 = HIGH;
            //reset IR_state_time for next case  ????
          }

          break;
        //OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_OUT_

        case IR_STATE_READ_OUT1:                                            //label 3
          Serial.println (F("Cycle_IRs reached case IR_STATE_READ_OUT1    "));                          //Debug


          switch (Outbound_Sensor_Sequence ) {                                     //SubSwitch nested  | label 30

            case POWER_SAVE_OUT:                                                      //SubSwitch case
            Serial.println (F("  POWER_SAVE_OUT:  "));                          //Debug
              digitalWrite(IR_inPowerPin, LOW);                                      // Don't power IRs yet, power saving 200ms off
              digitalWrite(IR_outPowerPin, LOW);
              if   (millis() - IR_state_time > 200) {                                 // 200ms with IRs disabled to save power
                Outbound_Sensor_Sequence = ENABLE_OUTBOUND_SENSOR ;
                IR_state_time = millis();
              }

              break;

            case ENABLE_OUTBOUND_SENSOR:                                                     //SubSwitch case   |label 31
            Serial.println (F("  ENABLE_OUTBOUND_SENSOR:  "));                          //Debug
              digitalWrite(IR_inPowerPin, LOW);                                      // inbound IR disabled still
              digitalWrite(IR_outPowerPin, HIGH);                                      //  Now GPIO pin   enables outbound IR enabled
              if (millis() - IR_state_time > 50) {                                                                         //another 50ms to enable and settle
                Outbound_Sensor_Sequence = FIRST_OUTBOUND_READ_DECISION;
                IR_state_time = millis();
              }

              break;

            case FIRST_OUTBOUND_READ_DECISION :                                          //SubSwitch case     | label 32
            Serial.println (F("  FIRST_OUTBOUND_READ_DECISION :   "));                          //Debug
                digitalWrite(IR_inPowerPin, LOW);                                      //no change
                digitalWrite(IR_outPowerPin, HIGH);                                      // no change
              proximityOutbound1 = digitalRead(IR_outReaderPin);                // Read the outbound IR, default is HIGH, LOW if proximity <10cm,
              Serial.print (F ("proximityOutbound1 == "));
              Serial.println (proximityOutbound1);

              if ( proximityOutbound1 ==  LOW  )   {                                         //
                Serial.print (F(" something is close to  outbound sensor something is close to  outbound sensor something is close to  outbound sensor transition to IR_STATE_READ_OUT2  "));    //Debug
                IR_state = IR_STATE_READ_OUT2;                                   // has already detected proximity once so go to next case to confirm
                IR_state_time = millis();                                                     //reset time
              }


              else if  (millis() - IR_state_time > 150) {                                     //NOT low and times out
                IR_state = IR_INITIATE;                                               // commit to case change
                Serial.println (F(" case FIRST_OUTBOUND_READ_DECISION : has timed out now without a LOW  "));                          //Debug
                // IR_INITIATE resets millis()
                //deBugByte2 = HIGH;
                //no need to do  IR_state_time = millis();   because  IR_INITIATE does that
              }
          }
              break;                                    // last break of sub switch
                                                       //
          break;                                    // break for case Outbound_Sensor_Sequence






        case IR_STATE_READ_OUT2:                                                   // label 4
          // this case is to confirm contact, proximityOutbound1 will already  be LOW  to have reached this point
          digitalWrite(IR_inPowerPin, LOW);                                      //inbound IR disabled
          digitalWrite(IR_outPowerPin, HIGH);                                      // GPIO pin  still enables  outbound IR
          Serial.println (F("Cycle_IRs reached case IR_STATE_READ_OUT2    "));                          //Debug

          if (millis() - IR_state_time > 50) {                                  //50ms time to enable and settle sensor, then start read
            proximityOutbound2 = digitalRead(IR_outReaderPin);                // Read the sensor, default is HIGH, LOW if proximity <10cm,
          }


          if (proximityOutbound2 == LOW) {
            STATE_current = STATE_IR_OUT_IS_TRIGGERED;                           //State transition because of confirmed proximity, cuts off this switch
            Serial.println(F("State transition statement, IDLE_LISTENING_FOR_IR -> IR_OUT_IS_TRIGGERED"));
            digitalWrite(IR_inPowerPin, LOW);                                      // power down IRs before leaving case
            digitalWrite(IR_outPowerPin, LOW);
            proximityOutbound1 = HIGH;                                              // reset variables
            proximityOutbound2 = HIGH;

          }

          else if ( millis() - IR_state_time > 150) {
            Serial.println (F(" case IR_STATE_READ_OUT2:   has timed out now without a 2nd LOW  "));                  //Debug   has timed out now without 2nd LOW
            IR_state = IR_INITIATE;                                                   //  no confirmation in 50ms consider false and go back to cycle start
            digitalWrite(IR_inPowerPin, LOW);                                         // power down IRs before leaving case
            digitalWrite(IR_outPowerPin, LOW);                                        //INITIATE will reset millis()
            proximityOutbound1 = HIGH;                                              // reset variables
            proximityOutbound2 = HIGH;
          }

          break;                                 // last break of  switch
  }                                           // close switch IR_state
  }                                // Close function
//_________END_IR____________END_IR____________END_IR____________END_IR____________END_IR____________END_IR____________END_IR____________END_IR___



//_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________
//_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________RFID_____________

void LOG_rfidCodeFound() {
  Serial.println(F("fnxn LOG_codeFound() - running"));
  LOG_date_time();                                 //  makes time stamp into line of data for SD

  SDCARD_logfile.print(RFID_currentTag);
  SDCARD_logfile.print (",");
  SDCARD_logfile.println("Authorised entry");
  SDCARD_logfile.flush();                //Saves line to current file

}
void RFID_checkForTag() {                                                               //dont't change case names till everything works

  switch (RFID_state) {
    case RFID_STATE_OFF:
      Serial.println(F("RFID_checkForTag() - Starting RFID"));
      RFID_serial.write(RFID_activateCircuit);
      RFID_state = RFID_STATE_INIT;
      RFID_state_time = millis();
      break;
    case RFID_STATE_INIT:
      // we're initializing the card, wait here for at least 200 milliseconds while it starts.
      if ( millis() > RFID_state_time + 200 ) {
        Serial.println(F("RFID_checkForTag() - RFID has waited 200, changing into read state"));
        RFID_state = RFID_STATE_READING;
        RFID_state_time = millis();
      }
      break;
    case RFID_STATE_READING:
      // Ready to read a tag.
      if ( millis() - RFID_state_time < 3000 ) {                                      //reads for 3s then times out*****           could maybe reduce, see time on output
        for (int readByte = 0; readByte < RFID_serial.available(); readByte++) {
          Serial.print(F("RFID_checkForTag() - read a byte, it is    "));
          char bytefound = RFID_serial.read();
          Serial.println(bytefound);
          if (bytefound == '_') {
            RFID_currentTagPos = 0;
          }
          else if (RFID_currentTagPos < 12) {
            RFID_currentTag[RFID_currentTagPos++] = bytefound;
          }
          if (RFID_currentTagPos == 12) {
            RFID_state = RFID_STATE_GOT_VALUE;                              //State change not time dependant if successful
            RFID_state_time = millis();
          }
        }
      }
      else {                                                   //***** add in a timeout if tag not read in 1 sec
        STATE_current = STATE_IDLE_LISTENING_FOR_IR;                                                    //transition state commit
        Serial.println(F("RFID_checkForTag() - Timed out, Stopping RFID reader"));          // debug
        RFID_serial.write(RFID_deactivateCircuit);                                            //deactivate Priority1 reader after commit

      }


      break;
    case RFID_STATE_GOT_VALUE:
      // we have a tag value in RFID_currentTag
      Serial.println(F("RFID_checkForTag() - found a code:"));
      Serial.println(RFID_currentTag);

      long RFID_currentTagNumber = atol(RFID_currentTag);               // atol == convert a char string to a long integer. This is causing and overflow warning in platformio
      switch (RFID_currentTagNumber) {
        case 141000981540:                              // all these Tag# /cases throw a warning "overflow in implicit constant conversion [-Woverflow]"
        case 141000981549:
        case 141000981545:
        case 141000981542:
          Serial.println(F("RFID_checkForTag() - access granted."));
          LOG_rfidCodeFound();
          STATE_current = STATE_RFID_SUCCESS;
          break;                                                          //break for switch (RFID_currentTagNumber)
        default:
          Serial.println(F("RFID_checkForTag() - access DENIED."));
      }


      Serial.println(F("RFID_checkForTag() - Stopping RFID reader"));         //reset and close down reader
      RFID_serial.write(RFID_deactivateCircuit);
      RFID_state = RFID_STATE_OFF;
      RFID_state_time = millis();
      RFID_currentTag[12] = 0;
      RFID_currentTagPos = 0;

      break;                                                              //break for case RFID_STATE_GOT_VALUE:
  }

}                        // When this function is a bool, throws a warning in platformio "no return statement in function returning non-void [-Wreturn-type]"
                        // no problem when a void function. 


void loop() {
  // Not sure this is needed in every loop: ?         ****When everything else works, comment it out****
  DateTime now;

  if (Serial.available() > 0) {
    int debugByte = Serial.read();    //what you type is the case
    //use serial for debugging inputs.
    // do something different depending on the character received.
    // The switch statement expects single number values for each case; in this
    // example, though, you're using single quotes to tell the controller to get
    // the ASCII value for the character. For example 'a' = 97, 'b' = 98,
    // and so forth:

    switch (debugByte) {                            //go to 855 for examples of the in-switch code
      case 'a':                                 //97
        deBugByte1 = LOW;                   //mimic inputs from sensors, 1st read
        break;
      case 'b':                             //98
        deBugByte1 = HIGH;                   //restores
        break;
      case 'c':
        deBugByte2 = LOW ;                  //
        break;
      case 'd':                               //
        deBugByte2 = HIGH;                    //restores inbound
        break;
      case 'e':
        deBugByte3 = LOW;                       //mimic 2 inbound reads

        break;
      case 'f':
        deBugByte4 = LOW;                         //mimic 2 outbound reads
        break;

      case 'g':
        deBugByte5 = LOW;                             // use to mimic jammed door
        break;

      case 'h':
        deBugByte5 = HIGH;                             // restores door to high
        break;


      case 'z':
        Serial.println(F("I just typed 'z' to stop the serial"));
        Serial.end();
        break;
    }
  }



  switch (STATE_current) {
    case STATE_IDLE_LISTENING_FOR_IR:                                           // label 0
     // Serial.println(F("STATE_now_IDLE_LISTENING_FOR_IR"));
      cycle_IRs();                      //function that times work cycle of IRs and reads appropriate times called every loop if this state current



      //  if (IR_checkStatusOut == true) {                                              //these 3 lines handled in fnxn cycle IRs case read out 2
      //  STATE_current = STATE_IR_OUT_IS_TRIGGERED;                          // jump to case  "STATE_IR_OUT_IS_TRIGGERED: "
      //  Serial.println(F("State transition, IDLE_LISTENING_FOR_IR -> IR_IN_IS_TRIGGERED"));

      break;

    case STATE_IR_IN_IS_TRIGGERED:                                          //label 1
      Serial.println(F("STATE_current is StateIRinIsTriggered"));
      Serial.println(F("loop() - IR IN has triggered, turning on RFID"));               // in is triggered, we want to turn the rfid on.
      STATE_current = STATE_RFID_READING;                                               // which in turn calls fnxn RFID_checkForTag()

      break;

    case STATE_IR_OUT_IS_TRIGGERED:                                         //label 2
      Serial.println(F("loop() - IR OUT has triggered"));
      Serial.println(F("STATE_current is StateIR_OutIsTriggered"));
      STATE_current = STATE_LOG_EXIT;

      // add opportuniist scan on exit  feature once everything works if still feel the need
      //scan after interval for door to open then log time, direction and chip# regardless if on list.

      break;

    case STATE_RFID_READING:                                                 //label 3
      //  Serial.println(F("loop() - RFID is reading"));
      Serial.println(F("STATE_current is RFID_Is_Reading"));

      RFID_checkForTag();

      break;


    case STATE_RFID_SUCCESS:                                                //label 4
      Serial.println(F("loop() - RFID received successfully"));
      Serial.println(F("STATE_current is State_RFID_Success"));
      STATE_current = STATE_DOOR_CYCLING;                                     //commit to state transition
      DOOR_state = DOOR_STATE_INITIALISE;                                     //gets read once after commit

      break;

    case STATE_LOG_EXIT:                                                      // label 5
      Serial.println(F("STATE_current is STATE_LOG_EXIT"));                     // which will initiate functions to log timestamp and transition to STATE_DOOR CYCLING.
      LOG_time_and_open_door_for_exit();                                                        //3 cases in this fnxn log; time, flush file to SD, and call door cycle


      break;

    case STATE_DOOR_CYCLING:                                                  // label 6
      Serial.println(F("STATE_current is STATE_DOOR_CYCLING"));                     //transitions out to with statement in fnxn DOOR-trigger case DOOR_STATE_CLOSED

      DOOR_trigger();



      break;
  }                 // end switch
}                   // end loop
