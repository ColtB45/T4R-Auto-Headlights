#include <HardwareCAN.h>
#include <math.h>
/*
   Uses STM32duino with Phono patch.
   https://github.com/Phonog/Arduino_STM32/tree/Phonog-patch-1
*/

static float clockCor         = 1.205;                  // time clock correction factor
static char* ver              = "0.1";                  // software version
static byte bootPause         = 28;                     // delay in seconds to pause in bootloader programing mode.
static int stopTalkingDelay   = (30 * 1000) * clockCor; // delay in seconds after the ignition is off before going silent on the CAN bus
static word nitLvlOn          = 350;                    // level at which to turn on lights   (0 bright, to 65,535 dark)
static word nitLvlOff         = nitLvlOn - 60;          // level at which to turn off lights  (0 bright, to 65,535 dark)
static int onToOffTime        = (15 * 1000) * clockCor; // how long in seconds the light level must be below nitLvlOff before the lights turn off
static int offToOnTime        = (5 * 1000) * clockCor;  // how long in seconds the light level must be above nitLvlOn before the lights turn on
static bool superBrights      = 0;                      // 0 =off, 1 = on, allows the fog lights to operate while brights are on

HardwareCAN canBus(CAN1_BASE);
CanMsg msg;

//elapsedMillis sinceLastIgnOnMsg;
word sinceLastIgnOnMsg = 0;
void sinceLastIgnOnMsgHandler1(void) {
  sinceLastIgnOnMsg++;
}

//elapsedMillis sinceIgnOn;
word sinceIgnOn = 0;
void sinceIgnOnHandler1(void) {
  sinceIgnOn++;
}

//elapsedMillis onToOffTimer;
word onToOffTimer = 0;
void onToOffTimerHandler1(void) {
  onToOffTimer++;
}

//elapsedMillis offToOnTimer;
word offToOnTimer = 0;
void offToOnTimerHandler1(void) {
  offToOnTimer++;
}

double blink;     // used for blinking light

// combined the variables above into a single byte to conserve RAM. Not needed on the Teensy 3, but may be nessisary when porting to other platforms.
byte bitVar1 = 0;   // MSB to LSB, 7 = itsDark, 6 = talk, 5 = brightsOn, 4 = fogOn, 3 lightsOn, 2 available, 1 available, 0 available

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) {
      Serial1.print("0");
    }
    Serial1.print(data[i], HEX);
    Serial1.print(" ");
  }
}

void timerSetup() {
  // Setup Counting Timers
  Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer1.pause();
  Timer1.setCount(0);
  Timer1.setOverflow(30000);
  //    Timer1.setCompare(TIMER_CH1, 20000);   // somewhere in the middle
  Timer1.attachInterrupt(TIMER_CH1, sinceLastIgnOnMsgHandler1);
  Timer1.resume();

  Timer2.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer2.pause();
  Timer2.setCount(0);
  Timer2.setOverflow(30000);
  //    Timer2.setCompare(TIMER_CH1, 20000);   // somewhere in the middle
  Timer2.attachInterrupt(TIMER_CH1, sinceIgnOnHandler1);
  Timer2.resume();

  Timer3.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer3.pause();
  Timer3.setCount(0);
  Timer3.setOverflow(30000);
  //    Timer3.setCompare(TIMER_CH1, 20000);   // somewhere in the middle
  Timer3.attachInterrupt(TIMER_CH1, onToOffTimerHandler1);
  Timer3.resume();

  Timer4.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer4.pause();
  Timer4.setCount(0);
  Timer4.setOverflow(30000);
  //    Timer4.setCompare(TIMER_CH1, 20000);   // somewhere in the middle
  Timer4.attachInterrupt(TIMER_CH1, offToOnTimerHandler1);
  Timer4.resume();
}

void CANSetup(void)
{
  CAN_STATUS Stat ;

  // Initialize CAN module
  canBus.map(CAN_GPIO_PB8_PB9);                         // default pin mapping, CAN_GPIO_PB8_PB9. alt is CAN_GPIO_PA11_PA12
  Stat = canBus.begin(CAN_SPEED_500, CAN_MODE_NORMAL);  // CAN bus speed and mode

  canBus.filter(0, 0x620, 0x7ff);                       // CAN filters: filter NUMBER (1-14), filter can ID, filter can MASK
  canBus.filter(1, 0x758, 0x7ff);
  canBus.set_irq_mode();                                // Use irq mode (recommended), so the handling of incoming messages
  // will be performed at ease in a task or in the loop. The software fifo is 16 cells long,
  // allowing at least 15 ms before processing the fifo is needed at 125 kbps
  Stat = canBus.status();
  if (Stat != CAN_OK)
    Serial1.print("CAN Init FAILED!");                        // Initialization failed
}


// Send one frame. Parameter is a pointer to a frame structure (above), that has previously been updated with data.
// If no mailbox is available, wait until one becomes empty. There are 3 mailboxes.
CAN_TX_MBX CANsend(CanMsg *pmsg) // Should be moved to the library?!
{
  CAN_TX_MBX mbx;

  do
  {
    mbx = canBus.send(pmsg) ;
#ifdef USE_MULTITASK
    vTaskDelay( 1 ) ;                 // Infinite loops are not multitasking-friendly
#endif
  }
  while (mbx == CAN_TX_NO_MBX) ;      // Waiting outbound frames will eventually be sent, unless there is a CAN bus failure.
  return mbx ;
}

// Send message
// Prepare and send a frame containing some value
void SendCANmessage(long id = 0x001, byte dlength = 8, byte d0 = 0x00, byte d1 = 0x00, byte d2 = 0x00, byte d3 = 0x00, byte d4 = 0x00, byte d5 = 0x00, byte d6 = 0x00, byte d7 = 0x00)
{
  // Initialize the message structure
  // A CAN structure includes the following fields:
  msg.IDE = CAN_ID_STD;          // Indicates a standard identifier ; CAN_ID_EXT would mean this frame uses an extended identifier
  msg.RTR = CAN_RTR_DATA;        // Indicated this is a data frame, as opposed to a remote frame (would then be CAN_RTR_REMOTE)
  msg.ID = id ;                  // Identifier of the frame : 0-2047 (0-0x3ff) for standard idenfiers; 0-0x1fffffff for extended identifiers
  msg.DLC = dlength;                   // Number of data bytes to follow

  // Prepare frame : send something
  msg.Data[0] = d0 ;
  msg.Data[1] = d1 ;
  msg.Data[2] = d2 ;
  msg.Data[3] = d3 ;
  msg.Data[4] = d4 ;
  msg.Data[5] = d5 ;
  msg.Data[6] = d6 ;
  msg.Data[7] = d7 ;

  CANsend(&msg) ;      // Send this frame
  delay(1);
}

void querySwitchStatus() {
  // send switch status query
  //  if(talk){
  if (bitRead(bitVar1, 6)) {
    SendCANmessage(0x750, 8, 0x40, 0x02, 0x21, 0xA7, 0x00, 0x00, 0x00, 0x00);
  }
}

void setLightsOff() {
  // turn lights off
  //  if(talk){
  if (bitRead(bitVar1, 6)) {
    bitWrite(bitVar1, 3, 0);
    SendCANmessage(0x750, 8, 0x40, 0x06, 0x30, 0x15, 0x00, 0x00, 0x00, 0x00);
  }
}

void setLightsOn() {
  // turn lights on
  //  if(talk){
  if (bitRead(bitVar1, 6)) {
    bitWrite(bitVar1, 3, 1);
    byte msgbuf[8];
    msgbuf[0] = 0x40;
    msgbuf[1] = 0x06;
    msgbuf[2] = 0x30;
    msgbuf[3] = 0x15;
    msgbuf[4] = 0x00;
    //    if(brightsOn){
    if (bitRead(bitVar1, 5)) {
      msgbuf[5] = 0xe0;
    } else {
      msgbuf[5] = 0xc0;
    }
    //    if(fogOn){
    if ((bitRead(bitVar1, 4) && bitRead(bitVar1, 5) == 0) || (bitRead(bitVar1, 4) && superBrights)) {
      msgbuf[6] = 0x80;
    } else {
      msgbuf[6] = 0x00;
    }
    msgbuf[7] = 0x00;
    SendCANmessage(0x750, 8, msgbuf[0], msgbuf[1], msgbuf[2], msgbuf[3], msgbuf[4], msgbuf[5], msgbuf[6], msgbuf[7]);
  }
}

void setup() {
  pinMode(PC13, OUTPUT);

  delay(2000 * clockCor);

  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.print("\r\nToyota 4Runner 5th Gen Auto lights\r\nv");
  Serial.print(ver);
  Serial.println(" for STM32F103C");
  Serial.println("Copyright Colt Boyd, 2019\r\n");

  Serial1.print("\r\nToyota 4Runner 5th Gen Auto lights\r\nv");
  Serial1.print(ver);
  Serial1.println(" for STM32F103C");
  Serial1.println("Copyright Colt Boyd, 2019\r\n");

  Serial.print("superBrights: ");
  Serial.print(superBrights, DEC);
  Serial.print(" nitLvlOn: ");
  Serial.print(nitLvlOn, DEC);
  Serial.print(" nitLvlOff: ");
  Serial.print(nitLvlOff, DEC);
  Serial.print(" onToOffTime: ");
  Serial.println((int) round ((onToOffTime / clockCor) / 1000), DEC);
  Serial.print("offToOnTime: ");
  Serial.print((int) round ((offToOnTime / clockCor) / 1000), DEC);
  Serial.print(" stopTalkingDelay: ");
  Serial.print((int) round ((stopTalkingDelay / clockCor) / 1000), DEC);
  Serial.print(" clockCor: ");
  Serial.println(clockCor, DEC);
  Serial.print("bootPause: ");
  Serial.println(bootPause, DEC);
  Serial.println("");
  Serial.println("Due to the way the STM32 modules handles shared IRQs & memory");
  Serial.println("between the USB & CAN hardware, this USB serial console will");
  Serial.println("cease operation after countdown until this device is reset.");
  Serial.println("");

  Serial1.print("superBrights: ");
  Serial1.print(superBrights, DEC);
  Serial1.print(" nitLvlOn: ");
  Serial1.print(nitLvlOn, DEC);
  Serial1.print(" nitLvlOff: ");
  Serial1.print(nitLvlOff, DEC);
  Serial1.print(" onToOffTime: ");
  Serial1.println((int) round ((onToOffTime / clockCor) / 1000), DEC);
  Serial1.print("offToOnTime: ");
  Serial1.print((int) round ((offToOnTime / clockCor) / 1000), DEC);
  Serial1.print(" stopTalkingDelay: ");
  Serial1.print((int) round ((stopTalkingDelay / clockCor) / 1000), DEC);
  Serial1.print(" clockCor: ");
  Serial1.println(clockCor, DEC);
  Serial1.print("bootPause: ");
  Serial1.println(bootPause, DEC);
  Serial1.println("");

  for (int i = 0; i <= bootPause; i++) {
    Serial.print("Pausing in USB update mode for ");
    Serial1.print("Pausing in USB update mode for ");
    Serial.print(bootPause - i, DEC);
    Serial1.print(bootPause - i, DEC);
    Serial.print(" seconds.  \r");
    Serial1.print(" seconds.  \r");
    delay(1000 * clockCor);
    digitalWrite(PC13, !digitalRead(PC13));
  }

  Serial.println("\r\nStarting main program...");
  Serial1.println("\r\nStarting main program...");
  timerSetup();       // Initialize timers
  CANSetup();         // Initialize the CAN module and prepare the message structures.
}

void loop() {
  blink++;
  if (blink > 16383) {
    digitalWrite(PC13, !digitalRead(PC13));
    blink = 0;
  }

  CanMsg *r_msg;
  if ((r_msg = canBus.recv()) != NULL) {
    Serial1.print("RECV: "); Serial1.print(r_msg->ID, HEX); Serial1.print("#"); PrintHex8(r_msg->Data, r_msg->DLC); Serial1.println();

    switch (r_msg->ID) {
      case 0x620: {

          // stuff here for reading ignition status
          if (bitRead(r_msg->Data[4], 4) || bitRead(r_msg->Data[4], 5)) {
            sinceLastIgnOnMsg = 0;
            if (bitRead(bitVar1, 6) == 0) {
              sinceIgnOn = 0;
              Serial1.println("Ign just changed from OFF to ON");
            } else {
              Serial1.println("Ign on");
            }
            //    talk = 1;
            bitWrite(bitVar1, 6, 1);
          } else {
            Serial1.println("Ign off");
          }

          // stuff here for reading light sensor
          word nits = (r_msg->Data[2] << 8) + r_msg->Data[3]; // bitwise shift all 8 bits of r_msg->Data[2] and then add r_msg->Data[3]
          if (nits >= nitLvlOn || (nits >= nitLvlOff && bitRead(bitVar1, 3))) {
            //    itsDark = 1;
            bitWrite(bitVar1, 7, 1);
            Serial1.print("Dark. Nits: ");
            Serial1.println(nits, DEC);
          } else {
            //    itsDark = 0;
            bitWrite(bitVar1, 7, 0);
            Serial1.print("Not dark. Nits: ");
            Serial1.println(nits, DEC);
          }
          break;
        }
      case 0x758: {

          // stuff here for reading switches status
          if (r_msg->Data[0] == 0x40 && r_msg->Data[1] == 0x05 && r_msg->Data[2] == 0x61 && r_msg->Data[3] == 0xA7) {
            //    if (((r_msg->Data[4] >> 5) & 1)){
            if (bitRead(r_msg->Data[4], 2)) {
              Serial.println("Auto Lights on");
              //      if (itsDark) {
              if (bitRead(bitVar1, 7)) {
                Serial.println("dark, lights on");
                //        if(((r_msg->Data[4] >> 3) & 1)){
                if (bitRead(r_msg->Data[4], 4)) {
                  Serial.println("fog lights on");
                  //          fogOn=1;
                  bitWrite(bitVar1, 4, 1);
                } else {
                  Serial.println("fog lights off");
                  //          fogOn=0;
                  bitWrite(bitVar1, 4, 0);
                }
                //        if(((r_msg->Data[4] >> 0) & 1)){
                if (bitRead(r_msg->Data[4], 7)) {
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
          break;
        }
      default: {
          break;
        }
    }
    if (r_msg->ID == 0x620) {
    }

    canBus.free(); // clears recv message
  }

  //  if(talk==1 && sinceLastIgnOnMsg > stopTalkingDelay){
  if (bitRead(bitVar1, 6) && sinceLastIgnOnMsg > stopTalkingDelay) {
    //    talk = 0;
    bitWrite(bitVar1, 6, 0);
    sinceLastIgnOnMsg = 0;
  }
  querySwitchStatus();
  delay(500 * clockCor);
}
