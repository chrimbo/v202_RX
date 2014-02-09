


#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include "printf.h"
#include <MirfHardwareSpiDriver.h>

uint8_t data[16];

uint8_t freq_hopping[][16] = {
 { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
   0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
 { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
   0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
 { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
   0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
 { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
   0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

// table 3 incr 6
uint8_t myfreq[16];
uint8_t index = 0;

void setnextChannel(void)
{
  Mirf.configRegister(RF_CH,myfreq[index++]);
  if(index > 15)
    index = 0;
}
boolean calculateChecksum(uint8_t *recData)
{
  uint8_t checksum = 0;
  for(int i = 0; i< 15; i++)
  {
    checksum += *recData++;
  }
  checksum -= *recData;
  if(checksum == 0)
    return true;
  return false;
}
void calculateHoppingTable(uint8_t *bindpacket)
{
  uint8_t sum = bindpacket[7] + bindpacket[8] + bindpacket[9];
  uint8_t *fhrow = freq_hopping[sum&0x03];
  uint8_t incr = (sum & 0x1e) >> 2;
  for(int i = 0; i<16; i++)
  {
    sum = fhrow[i] + incr;
    myfreq[i] = (sum & 0x0f)?sum:sum-3;
  }
}
void printHoppingTable()
{
  Serial.println(F("Hopping table:"));
  for(int i = 0; i< 16; i++)
  {
    Serial.println(myfreq[i], HEX);
  }
}

uint8_t txid[] = {0,0,0};
void setTransmitter(uint8_t *id)
{
  txid[0] = id[0];
  txid[1] = id[1];
  txid[2] = id[2];
}

boolean ismyTransmitter(uint8_t *id)
{
  if(txid[0] != id[0])
    return false;
  if(txid[1] != id[1])
    return false;
  if(txid[2] != id[2])
    return false;
  return true;
}

uint8_t channels[] = {0,0,0,0,0};
boolean getChannels(uint8_t *packet)
{
  // Check for checksum
  if(!calculateChecksum(packet))
    return false; // Wrong checksum
  if(!ismyTransmitter(packet + 7))
    return false; // Wrong transmitter
  channels[0] = packet[0];
  channels[1] = packet[1]; 
  channels[2] = packet[2];
  channels[3] = packet[3];
  channels[4] = packet[4];
  return true;
}




uint8_t time = 0;
uint8_t timeout = 0;

void setup(){
  Serial.begin(115200);
  printf_begin();
  /*
   * Setup pins / SPI.
   */
   
  /* To change CE / CSN Pins:
   * 
   * Mirf.csnPin = 9;
   * Mirf.cePin = 7;
   */
  /*
  Mirf.cePin = 7;
  Mirf.csnPin = 8;
  */
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.configRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO));
  Mirf.configRegister(EN_AA, 0x00);      // no auto ack
  Mirf.configRegister(EN_RXADDR, 0x3f);  // all Pipes enabled 
  Mirf.configRegister(SETUP_AW, 0x03);   // 5 bytes - default
  Mirf.configRegister(SETUP_RETR, 0xff); // not relevant for receiver
  Mirf.configRegister(RF_CH, 0x08);      // channel eight
  Mirf.configRegister(RF_SETUP, 0x05);   // 1Mbps
  Mirf.configRegister(STATUS, 0x70);     
  Mirf.configRegister(OBSERVE_TX, 0x00); 
  Mirf.configRegister(CD, 0x00);         // CD-RPD 
  Mirf.configRegister(RX_PW_P1, 16); 
  Mirf.configRegister(FIFO_STATUS, 0x00);
  Mirf.setRADDR((byte*)("\x66\x88\x68\x68\x68"));
  Mirf.configRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO) |_BV(PWR_UP) | _BV(PRIM_RX));
  
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  
  Serial.println(F("Beginning ... "));
  uint8_t bindpacket[16];
  boolean bind = false;
  while(bind == false)
  {
    if(Mirf.dataReady())
    {
      Mirf.getData(bindpacket);
      if(!calculateChecksum(bindpacket))
      {
        Serial.println("Checksum error");
      }
      if(bindpacket[14] == 0xc0)
      {
        // Bindpacket received
        calculateHoppingTable(bindpacket);
        printHoppingTable();
        setnextChannel();
        bind = true;
        setTransmitter(bindpacket+7);
      }
    }
  }
  setnextChannel();
  digitalWrite(3, HIGH);
  
  
  time = millis();
  timeout = 0;
}

uint8_t packet[16];
uint8_t timeoutcounter = 0;

void printdata(byte *packet)
{
  printf("\r\nTHR:0x%02x  YAW:0x%02x  PIT:0x%02x  ROL:0x%02x FLGS:0x%02x \t", packet[0], packet[1], packet[2], packet[3], packet[14]);
}
void printchannels()
{
  printf("\r\nTHR:0x%02x  YAW:0x%02x  PIT:0x%02x  ROL:0x%02x FLGS:0x%02x \t", channels[0], channels[1], channels[2], channels[3], channels[14]);
}
void loop()
{
  
  timeout += millis() - time;
  time = millis();  
  if(Mirf.dataReady())
  {
    Serial.print("R");
    Mirf.getData(packet);
    setnextChannel();
    if(!getChannels(packet))
      Serial.print("FAIL\r\n");
    printchannels();
  //    printdata(packet);
  //  if(!calculateChecksum(packet))
  //  {
  //    Serial.println("Checksum error");
  //    return;
  //  }
    analogWrite(3, packet[0]);
    timeout = 0;
    timeoutcounter = 0;
  }
  if(timeout > 25)
  {
    // 24ms without new packet
    setnextChannel();
    setnextChannel();
    Serial.print("T");
    timeout = 0;
    if(++timeoutcounter > 3)
    {
      Serial.print("B");
      timeoutcounter = 0;
      
      index-= 9;
      index&=0x0f;
      setnextChannel();
      time = millis();
    }
  }
  
  
  
} 


  
  
  
