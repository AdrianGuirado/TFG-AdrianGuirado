/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors. 
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is a program that demonstrates the detailed setup of a LoRa test transmitter. 
  A packet containing ASCII text is sent according to the frequency and LoRa settings specified in the
  'Settings.h' file. The pins to access the lora device need to be defined in the 'Settings.h' file also.

  The details of the packet sent and any errors are shown on the Arduino IDE Serial Monitor, together with
  the transmit power used, the packet length and the CRC of the packet. The matching receive program,
  '104_LoRa_Receiver' can be used to check the packets are being sent correctly, the frequency and LoRa
  settings (in Settings.h) must be the same for the transmitter and receiver programs. Sample Serial
  Monitor output;

  10dBm Packet> Hello World 1234567890*  BytesSent,23  CRC,DAAB  TransmitTime,64mS  PacketsSent,2

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#define Program_Version "V1.1"

#include <SPI.h>                                               //the lora device is SPI based so load the SPI library                                         
#include <SX128XLT.h>                                          //include the appropriate library  

#define MIKROBUS0_LAMBDA80
//#define MIKROBUS0_E28
//#define MIKROBUS1_LAMBDA80
//#define MIKROBUS1_E28

#include "Settings.h"                            //include the setiings file, frequencies, LoRa settings etc   

SX128XLT LT;                                                   //create a library class instance called LT

uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

uint8_t buff[] = "Ja pots enviar";
uint8_t buffcheck[] = "check";
uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
int8_t  PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;          
String begin;
int i;

void loop()
{ 
  begin[0] = '0';
  if(Serial.available())
  {
    begin = Serial.readString();
    if(begin[0] == 'b' || begin[0] == 'B') // Option 2 menu
    {
      delay(3000);                                                 // Wait time
      TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array
      buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on reciver
      startmS =  millis();                                         //start transmit timer
      if (LT.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit error
      {
        endmS = millis();                                          //packet sent, note end time
        TXPacketCount++;
      }
      else
      {
        packet_is_Error_Transmissor();                                 //transmit packet returned 0, there was an error
      }
      i = 0;
      while(i<100)                                                 // Send 100 packets
      {
        i++;
        RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 600000000, WAIT_RX);
        PacketRSSI = LT.readPacketRSSI();              //read the recived RSSI value
        PacketSNR = LT.readPacketSNR();                //read the received SNR value
        if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
        {
          packet_is_Error_Receiver();
        }
        else
        {
          packet_is_OK_Receiver();
        }
      }
    }
    else if(begin[0] == 'c' || begin[0] == 'C') // Option 1 menu
    {
      TXPacketL = sizeof(buffcheck);                                    //set TXPacketL to length of array
      buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on reciver
      startmS =  millis();                                         //start transmit timer
      if (LT.transmit(buffcheck, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit error
      {
        endmS = millis();                                          //packet sent, note end time
      }
      else
      {
        packet_is_Error_Transmissor();                                 //transmit packet returned 0, there was an error
      }
      RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 1000, WAIT_RX);
      if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
      {
        Serial.print("Connection not succesfull");
        Serial.println();
      }
      else
      {
        Serial.print("Connection succesfull");
        Serial.println();
      }
    }    
  }
}

void packet_is_Error_Transmissor()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                      //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                        //print IRQ status
  LT.printIrqStatus();                                 //prints the text of which IRQs set
}


void packet_is_OK_Receiver() 
{
  // Print RSSI and SNR values
  Serial.print(PacketRSSI);
  Serial.print(";");
  Serial.print(PacketSNR);
  Serial.println();
}


void packet_is_Error_Receiver()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the device packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX); 
    LT.printIrqStatus();                            //print the names of the IRQ registers set
  }

  delay(250);                                       //gives a longer buzzer and LED flash for error 
  
}
void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello world");
  SPI.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
   if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {                       
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
  }

 //The function call list below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);
  LT.setRegulatorMode(USE_LDO);
  LT.setPacketType(PACKET_TYPE_LORA);
  LT.setRfFrequency(Frequency, Offset);
  LT.setBufferBaseAddress(0, 0);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  LT.setPacketParams(LenInSymb, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_UPLINK, 0, 0);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
  //***************************************************************************************************
}
