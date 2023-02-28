// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX

#include <SPI.h>
#include <RH_RF95.h>
#include <avr/interrupt.h>
#include <avr/io.h>
//hey look this is a new line of code
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// Controls
char controldata[20];
#define throttlePin PD5         // D5
#define rollPin     PD6         // D6
#define pitchPin    PD7         // D7
uint16_t throttleVal = 125;     // Motor Off
uint16_t rollVal = 187;         // Middle Position of Servo
uint16_t pitchVal = 187;        // Middle Position of Servo

// Interrupt variables
bool servos_on = false;                  // Used to turn on servos every 20ms
volatile bool servo1_on = true;          // Initialize to true to start cycle
volatile bool servo2_on = false;         // Servo2 turns on after servo1
volatile bool servo3_on = false;         // Servo3 turns on after servo2
volatile unsigned long start_time = 0;   // Start time for the beginning of the 20ms period

void setup() 
{
  // Lora Radio Setup
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Control Pins
  DDRD |= 0b11100000;   // set D7, D6, D5 to outputs

  // Timer2 Setup
  cli();                                      // disable interrupts
  TCCR2B = 0;                                 // clear TCCR2b
  OCR2A = 255;                                // initialize interrupt with max value
  TCCR2A = 1<<WGM21;                          // set to ctc mode
  TCCR2B = (1<<CS22) | (0<<CS21) | (1<<CS20); // set prescalar to 128 --> 8us period
  TIMSK2 = (1<<OCIE2A);                       // enable compare match interrupt for OCR2A
  sei();                                      // enable interrupts
  Serial.begin(115200);

  // Initialize start time for 1st cycle
  start_time = micros();  
}

//---------------------------
//      TURN ON SERVOS
//---------------------------
void setServo1() {
  servo1_on = true;           // set servo1 flag
  PORTD |= 0b00100000;        // turn on servo1 (D5)
  TCNT2 = 0;                  // reset timer2 counter
  OCR2A = throttleVal;        // set on-time of servo
  sei();                      // enable interrupts
  start_time = micros();      // record start-time
}

void setServo2() {
  servo2_on = true;           // set servo1 flag
  PORTD |= 0b01000000;        // turn on servo2 (D6)
  OCR2A = rollVal;            // set on-time of servo
}

void setServo3() {
  servo3_on = true;           // set servo1 flag
  PORTD |= 0b10000000;        // turn on servo3 (D7)
  OCR2A = pitchVal;            // set on-time of servo
}

//---------------------------
//         MAIN LOOP
//---------------------------

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t len = sizeof(controldata);
    
    if (rf95.recv(controldata, &len))
    {
      RH_RF95::printBuffer("Received: ", controldata, len);
      Serial.print("Got: ");
      Serial.println((char*)controldata);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }

    throttleVal = atoi((char*)controldata);
    rollVal = atoi((char*)controldata+4);
    pitchVal = atoi((char*)controldata+8);

    Serial.println("Throttle      Roll      Pitch");
    Serial.print(throttleVal); Serial.print("         ");
    Serial.print(rollVal); Serial.print("          ");
    Serial.println(pitchVal);
  }
  //if 20ms has passed, turn servos back on
  if ((micros() - start_time) >= 20000) {
    cli();                // disable interrupts
    servos_on = true;     // set servo flag
    setServo1();          // turn on servo1
  }
}

//---------------------------
//          PWM ISR
//---------------------------

// Whenever the ISR is called, it turns off the servo that is on and sets the next servo
// After all servos have been cycled, the servos_on flag is unset

ISR(TIMER2_COMPA_vect) {
  if (servos_on) {
    if (servo1_on) {
      PORTD &= 0b11011111;    // turn off servo1 (D5)
      servo1_on = false;
      setServo2();
    } else if (servo2_on) {
      PORTD &= 0b10111111;    // turn off servo2 (D6)
      servo2_on = false;
      setServo3();
    } else if (servo3_on) {
      PORTD &= 0b01111111;    // turn off servo3 (D7)
      servo3_on = false;
      servos_on = false;
    }
  }
}
