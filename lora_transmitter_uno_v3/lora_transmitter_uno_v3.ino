// LoRa 9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Controls
char controldata[20];
#define throttle    A0
#define roll        A1
#define pitch       A2
int throttleVal =   0;
int rollVal     =   0;
int pitchVal    =   0;

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(115200);
  delay(100);

  Serial.println("Arduino LoRa TX Test!");

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

  // Setup for Controller Pins
  pinMode(throttle, INPUT);
  pinMode(roll, INPUT);
  pinMode(pitch, INPUT);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  // Get control data from controller
  throttleVal = analogRead(throttle); //motor max = 75
  if (throttleVal > 1015) {
    throttleVal = 1015;
  }
  if (throttleVal < 75) {
    throttleVal = 75;
  }
  rollVal = analogRead(roll); //full left = 140, full right = 750
  if (rollVal > 750) {
    rollVal = 750;
  }
  if (rollVal < 140) {
    rollVal = 140;
  }
  pitchVal = analogRead(pitch); //full up = 175, full down = 435
  if (pitchVal > 435) {
    pitchVal = 435;
  }
  if (pitchVal < 175) {
    pitchVal = 175;
  }

  // Convert to byte-value for pwm signal
  throttleVal = map(throttleVal, 1015, 75, 125, 250);
  rollVal     = map(rollVal, 140, 750, 125, 250);
  pitchVal    = map(pitchVal, 175, 435, 125, 250);

//    Serial.println("Throttle      Roll      Pitch");
//    Serial.print(throttleVal); Serial.print("         ");
//    Serial.print(rollVal); Serial.print("          ");
//    Serial.println(pitchVal);

  // Convert to characters to be sent in data packet
  sprintf(controldata, "%u", throttleVal);
  controldata[3] = '\0';
  sprintf(controldata+4, "%u", rollVal);
  controldata[7] = '\0';
  sprintf(controldata+8, "%u", pitchVal);
  controldata[11] = '\0';
//  std::sprintf(controldata+12, "%u", yawVal);
//  controldata[15] = '\0';
//  std::sprintf(controldata+16, "%u", flapVal);
//  controldata[19] = '\0';

  Serial.println("Control Data: ");
  Serial.println(controldata);
  Serial.println(controldata+4);
  Serial.println(controldata+8);
  
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)controldata, 20);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  delay(100);
}
