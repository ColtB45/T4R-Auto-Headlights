#include <IFCT.h>
/*
  uses the Improved Flex Can for Teensy libray
  https://github.com/tonton81/IFCT
*/


static CAN_message_t msg;
static int stopTalkingDelay = 30 * 1000;      // delay in seconds after the ignition is off before going silent on the CAN bus
static word nitLvlOn        = 350;            // level at which to turn on lights   (0 bright, to 65,535 dark)
static word nitLvlOff       = nitLvlOn - 60;  // level at which to turn off lights  (0 bright, to 65,535 dark)
static int onToOffTime      = 15 * 1000;      // how long in seconds the light level must be below nitLvlOff before the lights turn off
static int offToOnTime      = 5 * 1000;       // how long in seconds the light level must be above nitLvlOn before the lights turn on
static bool superBrights    = 0;              // 0 =off, 1 = on, allows the fog lights to operate while brights are on

elapsedMillis sinceLastIgnOnMsg;
elapsedMillis sinceIgnOn;
elapsedMillis onToOffTimer;
elapsedMillis offToOnTimer;

//byte itsDark = 0;
//byte talk = 0;
//byte brightsOn = 0;
//byte fogOn = 0;
//byte lightsOn = 0;

// combined the variables above into a single byte to conserve RAM. Not needed on the Teensy 3, but may be nessisary when porting to other platforms like STM32.
byte bitVar1 = 0;   // MSB to LSB, 7 = itsDark, 6 = talk, 5 = brightsOn, 4 = fogOn, 3 lightsOn, 2 available, 1 available, 0 available

void setup() {
  delay(1000);
  Serial.println(F("Toyota 4Runner 5th Gen Auto lights"
                   "\r\nv0.1 for Teensy 3.1/3.2"
                   "\r\nCopyright Colt Boyd 2019\r\n"));
  //  pinMode(2, OUTPUT); // for the transceiver enable pin
  Can0.setBaudRate(500000);
  Can0.enableMBInterrupt(MB0);
  Can0.enableMBInterrupt(MB1);
  Can0.setMBFilter(MB0, 0x758);
  Can0.setMBFilter(MB1, 0x620);
  Can0.onReceive(MB0, canMB0);
  Can0.onReceive(MB1, canMB1);
  Can0.intervalTimer(); // enable queue system and run callback in background.
  Serial.print("superBrights: ");
  Serial.print(superBrights, DEC);
  Serial.print(" nitLvlOn: ");
  Serial.print(nitLvlOn, DEC);
  Serial.print(" nitLvlOff: ");
  Serial.print(nitLvlOff, DEC);
  Serial.print(" onToOffTime: ");
  Serial.print(onToOffTime / 1000, DEC);
  Serial.print(" offToOnTime: ");
  Serial.print(offToOnTime / 1000, DEC);
  Serial.print(" stopTalkingDelay: ");
  Serial.println(stopTalkingDelay / 1000, DEC);
  Serial.println("\r\nInit OK!");
}

void loop() {
  //  if(talk==1 && sinceLastIgnOnMsg > stopTalkingDelay){
  if (bitRead(bitVar1, 6) && sinceLastIgnOnMsg > stopTalkingDelay) {
    //    talk = 0;
    bitWrite(bitVar1, 6, 0);
    sinceLastIgnOnMsg = 0;
  }
  querySwitchStatus();
  delay(500);
}

void querySwitchStatus() {
  // send switch status query
  //  if(talk){
  if (bitRead(bitVar1, 6)) {
    CAN_message_t msg;
    msg.ext = 0;
    msg.id = 0x750;
    msg.len = 8;
    msg.buf[0] = 0x40;
    msg.buf[1] = 0x02;
    msg.buf[2] = 0x21;
    msg.buf[3] = 0xA7;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);
  }
}

void setLightsOff() {
  // turn lights off
  //  if(talk){
  if (bitRead(bitVar1, 6)) {
    bitWrite(bitVar1, 3, 0);
    CAN_message_t msg;
    msg.ext = 0;
    msg.id = 0x750;
    msg.len = 8;
    msg.buf[0] = 0x40;
    msg.buf[1] = 0x06;
    msg.buf[2] = 0x30;
    msg.buf[3] = 0x15;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);
  }
}

void setLightsOn() {
  // turn lights on
  //  if(talk){
  if (bitRead(bitVar1, 6)) {
    bitWrite(bitVar1, 3, 1);
    CAN_message_t msg;
    msg.ext = 0;
    msg.id = 0x750;
    msg.len = 8;
    msg.buf[0] = 0x40;
    msg.buf[1] = 0x06;
    msg.buf[2] = 0x30;
    msg.buf[3] = 0x15;
    msg.buf[4] = 0x00;
    //    if(brightsOn){
    if (bitRead(bitVar1, 5)) {
      msg.buf[5] = 0xe0;
    } else {
      msg.buf[5] = 0xc0;
    }
    //    if(fogOn){
    if ((bitRead(bitVar1, 4) && bitRead(bitVar1, 5) == 0) || (bitRead(bitVar1, 4) && superBrights)) {
      msg.buf[6] = 0x80;
    } else {
      msg.buf[6] = 0x00;
    }
    msg.buf[7] = 0x00;
    Can0.write(msg);
  }
}

void canMB0(const CAN_message_t &msg) {
  // stuff here for reading switches status
  if (msg.buf[0] == 0x40 && msg.buf[1] == 0x05 && msg.buf[2] == 0x61 && msg.buf[3] == 0xA7) {
    //    if (((msg.buf[4] >> 5) & 1)){
    if (bitRead(msg.buf[4], 2)) {
      Serial.println("Auto Lights on");
      //      if (itsDark) {
      if (bitRead(bitVar1, 7)) {
        Serial.println("dark, lights on");
        //        if(((msg.buf[4] >> 3) & 1)){
        if (bitRead(msg.buf[4], 4)) {
          Serial.println("fog lights on");
          //          fogOn=1;
          bitWrite(bitVar1, 4, 1);
        } else {
          Serial.println("fog lights off");
          //          fogOn=0;
          bitWrite(bitVar1, 4, 0);
        }
        //        if(((msg.buf[4] >> 0) & 1)){
        if (bitRead(msg.buf[4], 7)) {
          Serial.println("Bright lights on");
          //          brightsOn=1;
          bitWrite(bitVar1, 5, 1);
        } else {
          Serial.println("Bright lights off");
          //          brightsOn=0;
          bitWrite(bitVar1, 5, 0);
        }
        onToOffTimer = 0;
        if (sinceIgnOn < 2000) {
          setLightsOn();
        } else {
          sinceIgnOn = 2001;
          if (offToOnTimer > offToOnTime) {
            setLightsOn();
            offToOnTimer = offToOnTime + 1;
          } else {
            setLightsOff();
          }
        }
      } else {
        offToOnTimer = 0;
        Serial.println("not dark, lights off");
        if (sinceIgnOn < 2000) {
          setLightsOff();
        } else {
          sinceIgnOn = 2001;
          if (onToOffTimer > onToOffTime) {
            setLightsOff();
            onToOffTimer = onToOffTime + 1;
          } else {
            setLightsOn();
          }
        }
      }
    } else {
      Serial.println("Auto Lights off");
      setLightsOff();
    }
  }
}

void canMB1(const CAN_message_t &msg) {
  // stuff here for reading ignition status
  //  if (((ignStatus >> 2) & 1) || ((ignStatus >> 3) & 1)) {
  //  if (((msg.buf[4] >> 2) & 1) || ((msg.buf[4] >> 3) & 1)) {
  if (bitRead(msg.buf[4], 4) || bitRead(msg.buf[4], 5)) {
    sinceLastIgnOnMsg = 0;
    if (bitRead(bitVar1, 6) == 0) {
      sinceIgnOn = 0;
      Serial.println("Ign just changed from OFF to ON");
    } else {
      Serial.println("Ign on");
    }
    //    talk = 1;
    bitWrite(bitVar1, 6, 1);
  } else {
    Serial.println("Ign off");
  }

  // stuff here for reading light sensor
  //  if (curBrightness >= nitLvlOn) {
  if (word(msg.buf[2], msg.buf[3]) >= nitLvlOn || (word(msg.buf[2], msg.buf[3]) >= nitLvlOff && bitRead(bitVar1, 3))) {
    //    itsDark = 1;
    bitWrite(bitVar1, 7, 1);
    Serial.print("Dark. Nits: ");
    Serial.println(word(msg.buf[2], msg.buf[3]), DEC);
  } else {
    //    itsDark = 0;
    bitWrite(bitVar1, 7, 0);
    Serial.print("Not dark. Nits: ");
    Serial.println(word(msg.buf[2], msg.buf[3]), DEC);
  }
}
