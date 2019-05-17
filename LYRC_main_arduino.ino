// disclaimer:
// code pas parfait

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMegaPi.h>
#include <SPI.h>

// include async rfid lib
#include "Lel.h"

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (52)
#define PN532_MOSI (51)
#define PN532_SS   (20)  // connected to the SDA pin of the MegaPi ( digital pin 20 )
#define PN532_MISO (50)
#define PN532_IRQ  (21) // connected to the SCL pin of the MegaPi ( digital pin 21 )

Adafruit_PN532 nfc(PN532_SS);
MeLineFollower detectLigne(PORT_8);


uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;              // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

bool interruptTriggered = false;
bool readerDisabled = false;

String command = "";
String commandName = "";
int commandParam1 = 0;
int commandParam2 = 0;
int commandParam3 = 0;
int commandParam4 = 0;

const uint32_t timeoutBetweenCards = 3000;
uint32_t timeoutNfc = 0;

void handleInterrupt() {
  detachInterrupt(PN532_IRQ);
  interruptTriggered = true;
  readerDisabled = true;
  //Serial3.println("READERENABLED");
}

//Encoder Motor
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);
MeEncoderOnBoard Encoder_4(SLOT4);

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

void isr_process_encoder3(void)
{
  if (digitalRead(Encoder_3.getPortB()) == 0) {
    Encoder_3.pulsePosMinus();
  } else {
    Encoder_3.pulsePosPlus();
  }
}

void isr_process_encoder4(void)
{
  if (digitalRead(Encoder_4.getPortB()) == 0) {
    Encoder_4.pulsePosMinus();
  } else {
    Encoder_4.pulsePosPlus();
  }
}

void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == 1) {
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if (direction == 2) {
    leftSpeed = speed;
    rightSpeed = -speed;
  } else if (direction == 3) {
    leftSpeed = speed;
    rightSpeed = speed;
  } else if (direction == 4) {
    leftSpeed = -speed;
    rightSpeed = -speed;
  }
  Encoder_1.setTarPWM(rightSpeed);
  Encoder_2.setTarPWM(leftSpeed);
}
void moveDegrees(int direction, long degrees, int speed_temp)
{
  speed_temp = abs(speed_temp);
  if (direction == 1)
  {
    Encoder_1.move(degrees, (float)speed_temp);
    Encoder_2.move(-degrees, (float)speed_temp);
  }
  else if (direction == 2)
  {
    Encoder_1.move(-degrees, (float)speed_temp);
    Encoder_2.move(degrees, (float)speed_temp);
  }
  else if (direction == 3)
  {
    Encoder_1.move(degrees, (float)speed_temp);
    Encoder_2.move(degrees, (float)speed_temp);
  }
  else if (direction == 4)
  {
    Encoder_1.move(-degrees, (float)speed_temp);
    Encoder_2.move(-degrees, (float)speed_temp);
  }
}



void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime)_loop();
}


double angle_rad = PI / 180.0;
double angle_deg = 180.0 / PI;

MeMegaPiDCMotor motor_sec(12);

void handleRFID()
{

  if (interruptTriggered == true) {
    success = nfc.readDetectedPassiveTargetID(uid, &uidLength);
    Serial.println("read detected passive target id triggered");
  
    interruptTriggered = false;
  }
  if (success) {
    //le tag rfid a été détecté
        Serial.println("-- start reading --");

    success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);
    if (success) { //L'authetification a réussi
            Serial.println("  Auth success");
      uint8_t data[16];
      success = nfc.mifareclassic_ReadDataBlock(4, data);
      if (success) { // le block 4 a été lu
        if (data[0] == 0x77) { //il contient 0x77  au premier emplacement
          Serial3.println("C: VALID");
          Serial.println("VALID - This is a team card");
        }
        else {
          Serial3.println("C: BAD");
          Serial.println("INVALID - Not a team card");
        }
      }
    }
    success = 0;
        Serial.println("-- end reading --");
  }

  // c'est surtout le truc du timeout qu'il faut regarder, en gros ya un delai entre plusieurs lectures j'ai pas trop bien compris et ça qui permet d'éviter de lire 3000 cartes de suite je pense
  // faut regarder
  if (readerDisabled == true) {
    if (abs(millis() - timeoutNfc) > timeoutBetweenCards) {
      readerDisabled = false;
      nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
      attachInterrupt(digitalPinToInterrupt(PN532_IRQ), handleInterrupt, FALLING);
      timeoutNfc = millis();
    }
  }
}

boolean rfidConnected = false;
boolean rfidEnabled = true;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello");
  Serial3.begin(115200);

  if (rfidEnabled) {
    delay(200);

    nfc.begin();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
      Serial.println("E: RFID CARD NOT CONNECTED");
      rfidConnected = false;
    } else {
      Serial.println("Setup rfid...");
      rfidConnected = true;
//      Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
//      Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
//      Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

      // j'ai commenté car il n'y avait pas cette instruction dans le code du gars
      //nfc.setPassiveActivationRetries(0xFF);
      nfc.SAMConfig();

      // register IRQ
      pinMode(PN532_IRQ, INPUT_PULLUP);
      nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
      delay(500);
      attachInterrupt(digitalPinToInterrupt(PN532_IRQ), handleInterrupt, FALLING);


    }
  }

  //Set Pwm 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);
  //  Encoder_3.setPulse(8);
  //  Encoder_3.setRatio(46.67);
  //  Encoder_3.setPosPid(1.8,0,1.2);
  //  Encoder_3.setSpeedPid(0.18,0,0);

  Serial.println("L: Ready");
  Serial3.println("L: Ready");
//
//  int state = detectLigne.readSensors();
//  Serial.print("retour : ");
//  Serial.println(state);
//
//  Serial.print("S1_IN_S2_IN : ");
//  Serial.println(S1_IN_S2_IN);
//
//  Serial.print("S1_IN_S2_OUT : ");
//  Serial.println(S1_IN_S2_OUT);
//
//  if (state == S1_IN_S2_IN) {
//    Serial.println("HOME");
//    Serial3.println("HOME");
//  }
//  else {
//    Serial.println("FIELD");
//    Serial3.println("FIELD");
//  }



}

void loop() {
  _loop();
  if (Serial3.available()) {
    command = Serial3.readStringUntil('\n');
    if (command != "") {
      // if we have a tab char at the end of the command
      //       command = command.substring(0, command.length() - 1);
      commandName = command.substring(0, command.indexOf("#"));

      // Parse command arguments (1, 2, 3)
      String raw1 = command.substring(command.indexOf("#") + 1, command.length());
      if (raw1 != command) {
        commandParam1 = raw1.substring(0, raw1.indexOf("#")).toInt();

        String raw2 = raw1.substring(raw1.indexOf("#") + 1, raw1.length());
        //        Serial.println(raw2);

        if (raw2 != raw1) {
          if (raw2.indexOf("#") != -1) {
            commandParam2 = raw2.substring(0, raw2.indexOf("#")).toInt();

            String raw3 = raw2.substring(raw2.indexOf("#") + 1, raw2.length());
            if (raw3 != raw2) {
              if (raw3.indexOf("#") != -1) {
                commandParam3 = raw3.substring(0, raw3.indexOf("#")).toInt();
                commandParam4 = raw3.substring(raw3.indexOf("#") + 1, raw3.length()).toInt();
              } else {
                commandParam3 = raw3.substring(0, raw3.length()).toInt();
              }
            }
          } else {
            commandParam2 = raw2.substring(0, raw2.length()).toInt();
          }
        }
      }

      Serial.println(commandName);
      if (commandName == "RESET") {
        Serial3.println("L: Reset!");
      } else if (commandName == "HELLO") {
        Serial3.println("L: World!");
      } else if (commandName == "PING") {
        Serial3.println("L: Pong!");
      } else if (commandName == "OMA") {
        // open main arm
        Serial.println("Open main arm");
        Encoder_3.setTarPWM(-175);
        _delay(0.8);
        Encoder_3.setTarPWM(-100);
        _delay(0.8);
        Encoder_3.setTarPWM(0);
      } else if (commandName == "CMA") {
        Serial.println("Close main arm");
        // close main arm
        Encoder_3.setTarPWM(50);
        _delay(1);
        Encoder_3.setTarPWM(0);

      } else if (commandName == "OSA") {
        Serial.println("Open secondary arm");
        // open secondary arm
        motor_sec.run(-255);
        _delay(0.3);
        motor_sec.run(0);

      } else if (commandName == "CSA") {
        // close secondary arm
        Serial.println("Close secondary arm");
        motor_sec.run(150);
        _delay(0.5);
        motor_sec.run(20);
      } else if (commandName == "JOY") {
        int direction = commandParam1;
        int strength = commandParam2;
        Serial.println("JOY: " + (String) direction + ";" +  (String) strength);
        move(direction, strength);
      } else if (commandName == "HOME") {
        int state = detectLigne.readSensors();
        if (state == S1_IN_S2_IN) {
          Serial.println("HOME");
          Serial3.println("HOME");
        }
        else {
          Serial.println("FIELD");
          Serial3.println("FIELD");
        }
      }
      else {
        Serial3.println("E: invalid commandName");
      }

      delay(2);
      commandParam1 = 0;
      commandParam2 = 0;
      commandParam3 = 0;
      commandParam4 = 0;
    }
  }
  if (rfidConnected) {
    handleRFID();
  }
}


void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();

  //  int state = detectLigne.readSensors();
  //  Serial.print("retour dans le loop : ");
  //  Serial.println(state);
  //  if (state == S1_IN_S2_IN){
  //    Serial.println("HOME");
  //    Serial3.println("HOME");
  //  }
  //  else {
  //  Serial.println("FIELD");
  //  Serial3.println("FIELD");
  //  }
  //  delay(1000);
}
