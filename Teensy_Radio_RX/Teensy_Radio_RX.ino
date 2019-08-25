// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     1

/*
#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif
*/
//Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     2    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ)

const int LEFTMOTORA = 23;
const int LEFTMOTORB = 22;
const int RIGHTMOTORA = 21;
const int RIGHTMOTORB = 20;

#define RH_ENCODER_A 14
#define RH_ENCODER_B 15

#define LH_ENCODER_A 16
#define LH_ENCODER_B 17

/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_IRQN);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission


/** GPS Setup **/
static const int RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);

// For stats that happen every 5 seconds
unsigned long last = 0UL;

volatile long countL = 0;
volatile long countR = 0;


void setup() 
{
  Serial.begin(38400);
  //GPS Setup
  Serial3.begin(GPSBaud);
  
  //Radio Setup
  
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  pinMode(RFM69_CS, OUTPUT);
  //pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 RX Test!");
  Serial.println();
  
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  //pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  
  //Motor Setup
  pinMode(RIGHTMOTORA, OUTPUT);
  pinMode(RIGHTMOTORB, OUTPUT);
  pinMode(LEFTMOTORA, OUTPUT);
  pinMode(LEFTMOTORB, OUTPUT);
  
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
}

//Internal variables for state machine
enum UGVControlState{CONTROL_START, CONTROL_AUTO, CONTROL_MANUAL, CONTROL_STOP};
//Radio Communicate: 5, 6, 7
enum UGVCommandState{COMMAND_START, COMMAND_STOP, COMMAND_FORWARD, COMMAND_BACK, COMMAND_TURNL, COMMAND_TURNR};
//Radio Communicate: 0, 1, 2, 3, 4

unsigned char controlState = CONTROL_START;
unsigned char commandState = COMMAND_START;

long countLSaved = 0;
long countRSaved = 0;
unsigned long commandDistance = 0;

const unsigned char DISTANCE_DIGIT_LIMIT = 5;

const int MOTORSPEED = 140;
uint8_t commandSaved = 0;


bool commandFinished = false;

// Dont put this on the stack:

uint8_t dataOutMaintain[RH_RF69_MAX_MESSAGE_LEN] = "Maintain";
uint8_t dataOutRecieved[RH_RF69_MAX_MESSAGE_LEN] = "Recieved";
uint8_t *dataOut = dataOutMaintain;
uint8_t dataRecieved[RH_RF69_MAX_MESSAGE_LEN];
String radiopacketInput;

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

//Logic Variables
unsigned long countGPS = 0;
unsigned const long PERIODGPS = 2000;


void loop() {
  //Check for inputs
  
  //MicroUsb Serial
  while(Serial.available()) {
    radiopacketInput = Serial.readString();
    Serial.println(radiopacketInput);
    //Set first char to command
    commandSaved = 0x0F & radiopacketInput[0];
    countLSaved = countL;
    countRSaved = countR;
    if(radiopacketInput.length() >= 3) {
      unsigned char i = 0;
      for(i = 0; i < DISTANCE_DIGIT_LIMIT && i + 2 < radiopacketInput.length(); ++i) {
        //Checks how long is parameter
      }
      --i;
      unsigned char k = 0;
      commandDistance = 0;
      for(unsigned char j = i; j >= 0 && j + 2 < radiopacketInput.length(); --j) {
        commandDistance += (radiopacketInput[j + 2] & 0x0F) * pow(10, k);
        ++k;
      }
    }
  }

  //Radio Management
  if (rf69_manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      
      Serial.print("Got packet from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      dataRecieved[0] = buf[0];
      
      unsigned long tempRadioValue = 0;
      tempRadioValue = dataRecieved[0];
      Serial.println(tempRadioValue);
      tempRadioValue = dataRecieved[1];
      Serial.println(tempRadioValue);
      tempRadioValue = dataRecieved[2];
      Serial.println(tempRadioValue);
      if(dataRecieved[0] >= 0 && dataRecieved[0] < 10) { 
        dataRecieved[0] = buf[0];
        commandSaved = 0x0F & dataRecieved[0];
        dataRecieved[1] = buf[1];
        dataRecieved[2] = buf[2];
        commandDistance = (dataRecieved[1] << 8) | dataRecieved[2];
        dataOut = dataOutRecieved;
        countLSaved = countL;
        countRSaved = countR;
        
      }
      else {
        dataOut = dataOutMaintain;
      }
      Serial.println((char*)buf);

      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(dataOut, sizeof(dataOut), from))
        Serial.println("Sending failed (no ack)");
    }
  }
  
  // This sketch displays information every time a new sentence is correctly encoded.
    while (Serial3.available() > 0){
      if (gps.encode(Serial3.read())){
        //displayInfo();
      }
        /*Serial.print("CountL: ");
        Serial.println(countL);
        Serial.print("CountR: ");
        Serial.println(countR);*/
    }

  
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  //Output
  //UGV State Machine Execute
  commandState = CommandStateMachine(commandState);
  

}


unsigned int CommandStateMachine(unsigned int state) {
  //Command State Machine Transitions
  switch(state) {
    case COMMAND_START:
    state = COMMAND_STOP;
    break;
    
    case COMMAND_STOP:
    if(commandSaved == 1) {
        state = COMMAND_FORWARD;
        Serial.println("Popper");
    }
    else if(commandSaved == 2) {
        state = COMMAND_BACK;
    }
    else if(commandSaved == 3) {
        state = COMMAND_TURNL;
    }
    else if(commandSaved == 4) {
        state = COMMAND_TURNR;
    }
    break;

    case COMMAND_FORWARD:
    if(commandFinished){
        state = COMMAND_STOP;
        commandSaved = 0;
        commandFinished = false;
    }
    else if(commandSaved == 0){
        state = COMMAND_STOP;
    }
    else if(commandSaved == 2) {
        state = COMMAND_BACK;
    }
    else if(commandSaved == 3) {
        state = COMMAND_TURNL;
    }
    else if(commandSaved == 4) {
        state = COMMAND_TURNR;
    }
    break;

    case COMMAND_BACK:
    if(commandFinished){
        state = COMMAND_STOP;
        commandSaved = 0;
        commandFinished = false;
    }
    else if(commandSaved == 0){
        state = COMMAND_STOP;
    }
    else if(commandSaved == 1) {
        state = COMMAND_FORWARD;
    }
    else if(commandSaved == 3) {
        state = COMMAND_TURNL;
    }
    else if(commandSaved == 4) {
        state = COMMAND_TURNR;
    }
    break;

    case COMMAND_TURNL:
    if(commandFinished){
        state = COMMAND_STOP;
        commandSaved = 0;
        commandFinished = false;
    }
    else if(commandSaved == 0){
        state = COMMAND_STOP;
    }
    else if(commandSaved == 1) {
        state = COMMAND_FORWARD;
    }
    else if(commandSaved == 2) {
        state = COMMAND_BACK;
    }
    else if(commandSaved == 4) {
        state = COMMAND_TURNR;
    }
    break;

    case COMMAND_TURNR:
    if(commandFinished){
        state = COMMAND_STOP;
        commandSaved = 0;
        commandFinished = false;
    }
    else if(commandSaved == 0){
        state = COMMAND_STOP;
    }
    else if(commandSaved == 1) {
        state = COMMAND_FORWARD;
    }
    else if(commandSaved == 2) {
        state = COMMAND_BACK;
    }
    else if(commandSaved == 3) {
        state = COMMAND_TURNL;
    }
    break;

    default:
    state = COMMAND_START;
    break;
  }

  //Command State Machine Actions
  switch(state) {
    case COMMAND_STOP:
    UGV_Stop();
    //Serial.println("help");
    break;

    case COMMAND_FORWARD:
    commandFinished = UGV_Forward(countLSaved, countLSaved, commandDistance);
    //Serial.println("popping");
    break;

    case COMMAND_BACK:
    commandFinished = UGV_Back(countLSaved, countLSaved, commandDistance);
    break;

    case COMMAND_TURNL:
    commandFinished = UGV_TurnL(countLSaved, countLSaved, commandDistance);
    break;

    case COMMAND_TURNR:
    commandFinished = UGV_TurnR(countLSaved, countRSaved, commandDistance);
    break;
    
    default:
    break;
  }
  return state;
}

void UGV_Stop() {
    analogWrite(LEFTMOTORA, 0);
    analogWrite(LEFTMOTORB, LOW);
    analogWrite(RIGHTMOTORA, 0);
    analogWrite(RIGHTMOTORB, LOW);
}

bool UGV_Forward(long recordedEncoderL, long recordedEncoderR, unsigned long distance) {
    analogWrite(LEFTMOTORA, MOTORSPEED);
    analogWrite(LEFTMOTORB, LOW);
    analogWrite(RIGHTMOTORA, MOTORSPEED);
    analogWrite(RIGHTMOTORB, LOW);
    if(abs(countL - recordedEncoderL) >= distance) {
        return true;
    }
    return false;
}

bool UGV_Back(long recordedEncoderL, long recordedEncoderR, unsigned long distance) {
    analogWrite(LEFTMOTORA, LOW);
    analogWrite(LEFTMOTORB, MOTORSPEED);
    analogWrite(RIGHTMOTORA, LOW);
    analogWrite(RIGHTMOTORB, MOTORSPEED);
    if(abs(countL - recordedEncoderL) >= distance) {
        return true;
    }
    return false;
}

bool UGV_TurnL(long recordedEncoderL, long recordedEncoderR, unsigned long distance) {
    analogWrite(LEFTMOTORA, MOTORSPEED);
    analogWrite(LEFTMOTORB, LOW);
    analogWrite(RIGHTMOTORA, LOW);
    analogWrite(RIGHTMOTORB, MOTORSPEED);
    
    if(abs(countL - recordedEncoderL) >= distance) {
        return true;
    }
    return false;
}

bool UGV_TurnR(long recordedEncoderL, long recordedEncoderR, long distance) {
    analogWrite(LEFTMOTORA, LOW);
    analogWrite(LEFTMOTORB, MOTORSPEED);
    analogWrite(RIGHTMOTORA, MOTORSPEED);
    analogWrite(RIGHTMOTORB, LOW);
    if(abs(countL - recordedEncoderL) >= distance) {
        return true;
    }
    return false;
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      countL++;
    } else {
      countL--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      countL--;
    } else {
      countL++;
    }
  }
}
 
// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      countR++;
    } else {
      countR--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      countR--;
    } else {
      countR++;
    }
  }
}
