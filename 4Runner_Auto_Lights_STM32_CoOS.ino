#define USE_SEMAPHORE_DMA1
#include <HardwareCAN.h>
#include <MapleCoOS.h>
#include <math.h>
/*
   Uses STM32duino with Phonog patch
   https://github.com/Phonog/Arduino_STM32/tree/Phonog-patch-1
*/

// combined the variables above into a single byte to conserve RAM. Not needed on the Teensy 3, but may be nessisary when porting to other platforms.
byte bitVar1 = 0;   // MSB to LSB, 7 = itsDark, 6 = talk, 5 = brightsOn, 4 = fogOn, 3 lightsOn, 2 turnLightsOnOff, 1 DRLlastStatus, 0 functionMode

//static bool functionMode      = 0;                    // 1 = CAN only, 0 = Switch + CAN
static bool debug             = 1;                      // 0 = release, 1 = debug on, prints debug info out over PA9
static bool superBrights      = 0;                      // 0 =off, 1 = on, allows the fog lights to operate while brights are on
static word nitLvlOn          = 350;                    // level at which to turn on lights   (0 bright, to 65,535 dark)
static word nitLvlOff         = nitLvlOn - 60;          // level at which to turn off lights  (0 bright, to 65,535 dark)
static float clockCor         = 1.205;                  // time clock correction factor
static int onToOffTime        = (15 * 1000) * clockCor; // how long in seconds the light level must be below nitLvlOff before the lights turn off
static int offToOnTime        = (5 * 1000) * clockCor;  // how long in seconds the light level must be above nitLvlOn before the lights turn on
static int stopTalkDelay      = (30 * 1000) * clockCor; // delay in seconds after the ignition is off before going silent on the CAN buson
static byte bootPause         = 18;                      // delay in seconds to pause in bootloader programing mode.
static double sendFreq        = 250;                    /* How many times per second to query switch status.
                                                        This will directly affect how responsive or laggy switch input is.
                                                        Too frequent floods the bus. Too infrequent delays switch changes.
                                                        250ms, 4 times a second, is a safe value.*/
static char* ver              = "0.1";                  // software version

OS_STK   vLEDFlashStk[TASK_STK_SIZE];
OS_STK   vControlStk[TASK_STK_SIZE];
OS_STK   vMainLoopStk[TASK_STK_SIZE];

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
  if (Stat != CAN_OK && debug)
    Serial1.println("CAN Init FAILED!");                        // Initialization failed
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

static void vMainLoopTask(void *pdata) {
  for (;;) {

    CanMsg * r_msg;

    if ((r_msg = canBus.recv()) != NULL) {

      if (debug) {
        Serial1.print("RECV: ");
        Serial1.print(r_msg->ID, HEX);
        Serial1.print("#");
        PrintHex8(r_msg->Data, r_msg->DLC);
        Serial1.println();
      }

      switch (r_msg->ID) {

        case 0x620: {

            // stuff here for reading ignition status
            if (bitRead(r_msg->Data[4], 4) || bitRead(r_msg->Data[4], 5)) {
              sinceLastIgnOnMsg = 0;
              if (bitRead(bitVar1, 6) == 0) {
                sinceIgnOn = 0;
                if (debug) {
                  Serial1.println("Ign just changed from OFF to ON");
                }
              } else {
                if (debug) {
                  Serial1.println("Ign on");
                }
              }
              //    talk = 1;
              bitWrite(bitVar1, 6, 1);
            } else {
              if (debug) {
                Serial1.println("Ign off");
              }
            }

            // stuff here for reading light sensor
            word nits = (r_msg->Data[2] << 8) + r_msg->Data[3]; // bitwise shift all 8 bits of r_msg->Data[2] and then add r_msg->Data[3]
            if (nits >= nitLvlOn || (nits >= nitLvlOff && bitRead(bitVar1, 3))) {
              //    itsDark = 1;
              bitWrite(bitVar1, 7, 1);
              if (debug) {
                Serial1.print("Dark. Nits: ");
                Serial1.println(nits, DEC);
              }
            } else {
              //    itsDark = 0;
              bitWrite(bitVar1, 7, 0);
              if (debug) {
                Serial1.print("Not dark. Nits: ");
                Serial1.println(nits, DEC);
              }
            }
            break;
          }

        case 0x758: {

            // If DRL switch is deteched high, change function mode to Switch + CAN
            if (bitRead(bitVar1, 0) && digitalRead(PB14)) {
              bitWrite(bitVar1, 0, 0);
              if (debug) {
                Serial1.println("Changing function mode to 0, Switch + CAN");
              }
            }

            // stuff here for reading switches status
            if (r_msg->Data[0] == 0x40 && r_msg->Data[1] == 0x05 && r_msg->Data[2] == 0x61 && r_msg->Data[3] == 0xA7) {
              //    if (((r_msg->Data[4] >> 5) & 1)){
              if ((bitRead(bitVar1, 0) && bitRead(r_msg->Data[4], 2)) || (bitRead(bitVar1, 0) == 0 && digitalRead(PB14))) {
                if (debug) {
                  Serial1.println("Auto Lights on");
                }
                //      if (itsDark) {
                if (bitRead(bitVar1, 7)) {
                  if (debug) {
                    Serial1.println("dark, lights on");
                  }
                  //        if(((r_msg->Data[4] >> 3) & 1)){
                  if (bitRead(r_msg->Data[4], 4)) {
                    if (debug) {
                      Serial1.println("fog lights on");
                    }
                    //          fogOn=1;
                    bitWrite(bitVar1, 4, 1);
                  } else {
                    if (debug) {
                      Serial1.println("fog lights off");
                    }
                    //          fogOn=0;
                    bitWrite(bitVar1, 4, 0);
                  }
                  //        if(((r_msg->Data[4] >> 0) & 1)){
                  if (bitRead(r_msg->Data[4], 7)) {
                    if (debug) {
                      Serial1.println("Bright lights on");
                    }
                    //          brightsOn=1;
                    bitWrite(bitVar1, 5, 1);
                  } else {
                    if (debug) {
                      Serial1.println("Bright lights off");
                    }
                    //          brightsOn=0;
                    bitWrite(bitVar1, 5, 0);
                  }
                  onToOffTimer = 0;
                  if (sinceIgnOn < 2000) {
                    //                    setLightsOn();
                    if (bitRead(bitVar1, 6)) {
                      bitWrite(bitVar1, 2, 1);
                    }
                  } else {
                    sinceIgnOn = 2001;
                    if (offToOnTimer > offToOnTime) {
                      //                      setLightsOn();
                      if (bitRead(bitVar1, 6)) {
                        bitWrite(bitVar1, 2, 1);
                      }
                      offToOnTimer = offToOnTime + 1;
                    } else {
                      //                      setLightsOff();
                      if (bitRead(bitVar1, 6)) {
                        bitWrite(bitVar1, 2, 0);
                      }
                    }
                  }
                } else {
                  offToOnTimer = 0;
                  if (debug) {
                    Serial1.println("not dark, lights off");
                  }
                  if (sinceIgnOn < 2000) {
                    //                    setLightsOff();
                    if (bitRead(bitVar1, 6)) {
                      bitWrite(bitVar1, 2, 0);
                    }
                  } else {
                    sinceIgnOn = 2001;
                    if (onToOffTimer > onToOffTime) {
                      //                      setLightsOff();
                      if (bitRead(bitVar1, 6)) {
                        bitWrite(bitVar1, 2, 0);
                      }
                      onToOffTimer = onToOffTime + 1;
                    } else {
                      //                      setLightsOn();
                      if (bitRead(bitVar1, 6)) {
                        bitWrite(bitVar1, 2, 1);
                      }
                    }
                  }
                }
              } else {
                if (debug) {
                  Serial1.println("Auto Lights off");
                }
                //                setLightsOff();
                if (bitRead(bitVar1, 6)) {
                  bitWrite(bitVar1, 2, 0);
                }
              }
            }
            break;
          }

        default: {
            break;
          }

      }
      canBus.free(); // clears recv message

    }

    //  if(talk==1 && sinceLastIgnOnMsg > stopTalkDelay){
    if (bitRead(bitVar1, 6) && sinceLastIgnOnMsg > stopTalkDelay) {
      //    talk = 0;
      bitWrite(bitVar1, 6, 0);
      bitWrite(bitVar1, 2, 0);
      if (debug) {
        Serial1.print("Over ");
        Serial1.print((int) round ((stopTalkDelay / clockCor) / 1000), DEC);
        Serial1.println(" seconds since last CAN msg. Going to standby.");
      }
      sinceLastIgnOnMsg = 0;
    }
    CoTickDelay(1);
  }
}

static void vLEDFlashTask(void *pdata) {
  for (;;) {
    digitalWrite(PC13, !(digitalRead(PC13)));
    CoTickDelay(100 * clockCor);
  }
}

static void vControlTask(void *pdata) {
  for (;;) {
    // send switch status query
    //  if(talk){
    //    Serial1.println(bitRead(bitVar1, 6));
    if (bitRead(bitVar1, 6)) {

      // wakeup CAN driver
      digitalWrite(PA1, LOW);

      SendCANmessage(0x750, 8, 0x40, 0x02, 0x21, 0xA7, 0x00, 0x00, 0x00, 0x00);
      CoTickDelay((sendFreq * clockCor) / 2);
      if (bitRead(bitVar1, 2)) {
        bitWrite(bitVar1, 3, 1);
        byte msgbuf[1];
        //    if(brightsOn){
        if (bitRead(bitVar1, 5)) {
          msgbuf[0] = 0xe0;
        } else {
          msgbuf[0] = 0xc0;
        }
        //    if(fogOn){
        if ((bitRead(bitVar1, 4) && bitRead(bitVar1, 5) == 0) || (bitRead(bitVar1, 4) && superBrights)) {
          msgbuf[1] = 0x80;
        } else {
          msgbuf[1] = 0x00;
        }
        if (bitRead(bitVar1, 0)) {
          SendCANmessage(0x750, 8, 0x40, 0x06, 0x30, 0x15, 0x00, msgbuf[0], msgbuf[1], 0x00);
        } else if (superBrights && bitRead(bitVar1, 4) && bitRead(bitVar1, 5)) {
          SendCANmessage(0x750, 8, 0x40, 0x06, 0x30, 0x15, 0x00, 0x00, 0x80, 0x00);
        }
        // turn on taillights
        digitalWrite(PA11, HIGH);
        // turn on headlights
        digitalWrite(PB15, HIGH);
        // turn off DRLs
        digitalWrite(PB13, LOW);
      } else {
        bitWrite(bitVar1, 3, 0);
        if (bitRead(bitVar1, 0) || superBrights) {
          SendCANmessage(0x750, 8, 0x40, 0x06, 0x30, 0x15, 0x00, 0x00, 0x00, 0x00);
        }
        // turn off taillights
        digitalWrite(PA11, LOW);
        // turn off headlights
        digitalWrite(PB15, LOW);
        // turn on DRLs
        digitalWrite(PB13, HIGH);
      }
    } else {
      // turn off taillights
      digitalWrite(PA11, LOW);
      // turn off headlights
      digitalWrite(PB15, LOW);
      // turn off DRLs
      digitalWrite(PB13, LOW);
      // put can driver to sleep
      digitalWrite(PA1, HIGH);
    }
    CoTickDelay((sendFreq * clockCor) / 2);
  }
  CoTickDelay(1);
}

void setup() {

  // STM32 status LED
  pinMode(PC13, OUTPUT);

  // SN65HVD230 TX Sleep
  // drive high to put CAN driver to sleep, drive low to allow transmitting
  pinMode(PA1, OUTPUT);

  // optocoupler CH1
  // drive headlights to BCM status
  pinMode(PB15, OUTPUT);

  // optocoupler CH2
  // detect DRL from switch status
  pinMode(PB14, INPUT_PULLDOWN);
  // If DRL switch is deteched high, change function mode to Switch + CAN
  if (digitalRead(PB14)) {
    bitWrite(bitVar1, 0, 0);
  } else {
    bitWrite(bitVar1, 0, 1);
  }

  // optocoupler CH3
  // drive DRL to BCM status
  pinMode(PB13, OUTPUT);

  // optocoupler CH4
  // drive tailights to BCM status
  pinMode(PA11, OUTPUT);

  delay(2000 * clockCor);

  Serial.begin(115200);
  if (debug) {
    Serial1.begin(115200);
  }

  Serial.print("\r\nToyota 4Runner 5th Gen Auto lights\r\nv");
  Serial.print(ver);
  Serial.println(" for STM32F103C");
  Serial.println("Copyright Colt Boyd, 2019\r\n");

  Serial.print("functionMode:\t");
  Serial.print(bitRead(bitVar1, 0), DEC);
  Serial.print("\tsuperBrights:\t");
  Serial.println(superBrights, DEC);
  Serial.print("nitLvlOn:\t");
  Serial.print(nitLvlOn, DEC);
  Serial.print("\tnitLvlOff:\t");
  Serial.println(nitLvlOff, DEC);
  Serial.print("onToOffTime:\t");
  Serial.print((int) round ((onToOffTime / clockCor) / 1000), DEC);
  Serial.print("\toffToOnTime:\t");
  Serial.println((int) round ((offToOnTime / clockCor) / 1000), DEC);
  Serial.print("stopTalkDelay:\t");
  Serial.print((int) round ((stopTalkDelay / clockCor) / 1000), DEC);
  Serial.print("\tbootPause:\t");
  Serial.println(bootPause, DEC);
  Serial.print("sendFreq:\t");
  Serial.print((int) round (sendFreq), DEC);
  Serial.print("\tclockCor:\t");
  Serial.println(clockCor, DEC);
  Serial.println("");
  Serial.println("Due to the way the STM32 modules handles shared IRQs & memory");
  Serial.println("between the USB & CAN hardware, this USB serial console will");
  Serial.println("cease operation after countdown until this device is reset.");
  Serial.println("");

  if (debug) {
    Serial1.print("\r\nToyota 4Runner 5th Gen Auto lights\r\nv");
    Serial1.print(ver);
    Serial1.println(" for STM32F103C");
    Serial1.println("Copyright Colt Boyd, 2019\r\n");

    Serial1.print("functionMode:\t");
    Serial1.print(bitRead(bitVar1, 0), DEC);
    Serial1.print("\tsuperBrights:\t");
    Serial1.println(superBrights, DEC);
    Serial1.print("nitLvlOn:\t");
    Serial1.print(nitLvlOn, DEC);
    Serial1.print("\tnitLvlOff:\t");
    Serial1.println(nitLvlOff, DEC);
    Serial1.print("onToOffTime:\t");
    Serial1.print((int) round ((onToOffTime / clockCor) / 1000), DEC);
    Serial1.print("\toffToOnTime:\t");
    Serial1.println((int) round ((offToOnTime / clockCor) / 1000), DEC);
    Serial1.print("stopTalkDelay:\t");
    Serial1.print((int) round ((stopTalkDelay / clockCor) / 1000), DEC);
    Serial1.print("\tbootPause:\t");
    Serial1.println(bootPause, DEC);
    Serial1.print("sendFreq:\t");
    Serial1.print((int) round (sendFreq), DEC);
    Serial1.print("\tclockCor:\t");
    Serial1.println(clockCor, DEC);
    Serial1.println("");
  }

  for (int i = 0; i <= bootPause; i++) {
    Serial.print("Pausing in USB update mode for ");
    Serial.print(bootPause - i, DEC);
    Serial.print(" seconds.  \r");
    if (debug) {
      Serial1.print("Pausing in USB update mode for ");
      Serial1.print(bootPause - i, DEC);
      Serial1.print(" seconds.  \r");
    }
    delay(1000 * clockCor);
    digitalWrite(PC13, !digitalRead(PC13));
  }

  Serial.println("\r\nStarting main program...");
  if (debug) {
    Serial1.println("\r\nStarting main program...");
  }
  timerSetup();       // Initialize timers
  CANSetup();         // Initialize the CAN module and prepare the message structures.

  CoInitOS();
  CoCreateTask(vLEDFlashTask,
               (void *)0 ,
               2,
               &vLEDFlashStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoCreateTask(vControlTask,
               (void *)0 ,
               2,
               &vControlStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoCreateTask(vMainLoopTask,
               (void *)0 ,
               2,
               &vMainLoopStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );

  CoStartOS();
}

void loop() {
  // Do not write any code here, it would not execute.
}
