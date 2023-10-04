//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//
/////////////////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "ads1292r.h"
#include <SPI.h>

int j,i;

volatile byte SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived = false;

unsigned long uecgtemp = 0;
unsigned long resultTemp = 0;

signed long secgtemp = 0;

long statusByte=0;

uint8_t LeadStatus=0;

boolean ads1292r::getEcgAndRespirationSamples(const int dataReady,const int chipSelect,ads1292OutputValues *ecgRespirationValues)
{

  if ((digitalRead(dataReady)) == LOW)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  {
    SPI_RX_Buff_Ptr = ReadData(chipSelect); // Read the data,point the data to a pointer

    for (int i = 0; i < 9; i++)
    {
      SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
    }

    ads1292dataReceived = true;
    //Serial.println(ads1292dataReceived);
    //delay(1000);
    j = 0;

    for (i = 3; i < 9; i += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
    {
      uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff[i + 0] << 16) | ( (unsigned long) SPI_RX_Buff[i + 1] << 8) |  (unsigned long) SPI_RX_Buff[i + 2]);
      uecgtemp = (unsigned long) (uecgtemp << 8);
      secgtemp = (signed long) (uecgtemp);
      secgtemp = (signed long) (secgtemp >> 8);

      (ecgRespirationValues->sDaqVals)[j++] = secgtemp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
    }

    statusByte = (long)((long)SPI_RX_Buff[2] | ((long) SPI_RX_Buff[1]) <<8 | ((long) SPI_RX_Buff[0])<<16); // First 3 bytes represents the status
    statusByte  = (statusByte & 0x0f8000) >> 15;  // bit15 gives the lead status
    LeadStatus = (unsigned char ) statusByte ;
    resultTemp = (uint32_t)((0 << 24) | (SPI_RX_Buff[3] << 16)| SPI_RX_Buff[4] << 8 | SPI_RX_Buff[5]);//6,7,8
    resultTemp = (uint32_t)(resultTemp << 8);
    ecgRespirationValues->sresultTempResp = (long)(resultTemp);


    if(!((LeadStatus & 0x1f) == 0 ))
    {
      ecgRespirationValues->leadoffDetected  = true;
    }

    else
    {
      ecgRespirationValues->leadoffDetected  = false;
    }

    ads1292dataReceived = false;
    SPI_RX_Buff_Count = 0;
    return true;
  }

  else
    return false;
}

char* ads1292r::ReadData(const int chipSelect)
{
  static char SPI_Dummy_Buff[10];
  digitalWrite(chipSelect, LOW);

  for (int i = 0; i < 9; ++i)
  {
    SPI_Dummy_Buff[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  }

  digitalWrite(chipSelect, HIGH);
  return SPI_Dummy_Buff;
}

void ads1292r::Init(const int chipSelect,const int pwdnPin,const int startPin)
{
  // start the SPI library:
  Reset(pwdnPin);
  delay(100);
  DisableStart(startPin);
  EnableStart(startPin);
  HardStop(startPin);
  StartDataConvCommand(chipSelect);
  SoftStop(chipSelect);
  delay(50);
  StopReadDataContinuous(chipSelect);					// SDATAC command
  delay(300);
  RegWrite(ADS1292_REG_CONFIG1, 0x00,chipSelect); 		//Set sampling rate to 125 SPS
  delay(10);
  RegWrite(ADS1292_REG_CONFIG2, 0b10100000,chipSelect);	//Lead-off comp off, test signal disabled
  delay(10);
  RegWrite(ADS1292_REG_LOFF, 0b00010000,chipSelect);		//Lead-off defaults
  delay(10);
  RegWrite(ADS1292_REG_CH1SET, 0b01000000,chipSelect);	//Ch 1 enabled, gain 6, connected to electrode in
  delay(10);
  RegWrite(ADS1292_REG_CH2SET, 0b01100000,chipSelect);	//Ch 2 enabled, gain 6, connected to electrode in
  delay(10);
  RegWrite(ADS1292_REG_RLDSENS, 0b00101100,chipSelect);	//RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  delay(10);
  RegWrite(ADS1292_REG_LOFFSENS, 0x00,chipSelect);		//LOFF settings: all disabled
  delay(10);													//Skip register 8, LOFF Settings default
  RegWrite(ADS1292_REG_RESP1, 0b11110010,chipSelect);		//Respiration: MOD/DEMOD turned only, phase 0
  delay(10);
  RegWrite(ADS1292_REG_RESP2, 0b00000011,chipSelect);		//Respiration: Calib OFF, respiration freq defaults
  delay(10);
  StartReadDataContinuous(chipSelect);
  delay(10);
  EnableStart(startPin);
}

void ads1292r::Reset(const int pwdnPin)
{
  digitalWrite(pwdnPin, HIGH);
  delay(100);					// Wait 100 mSec
  digitalWrite(pwdnPin, LOW);
  delay(100);
  digitalWrite(pwdnPin, HIGH);
  delay(100);
}

void ads1292r::DisableStart(const int startPin)
{
  digitalWrite(startPin, LOW);
  delay(20);
}

void ads1292r::EnableStart(const int startPin)
{
  digitalWrite(startPin, HIGH);
  delay(20);
}

void ads1292r::HardStop (const int startPin)
{
  digitalWrite(startPin, LOW);
  delay(100);
}

void ads1292r::StartDataConvCommand (const int chipSelect)
{
  SPICommandData(START,chipSelect);					// Send 0x08 to the ADS1x9x
}

void ads1292r::SoftStop (const int chipSelect)
{
  SPICommandData(STOP,chipSelect);                   // Send 0x0A to the ADS1x9x
}

void ads1292r::StartReadDataContinuous (const int chipSelect)
{
  SPICommandData(RDATAC,chipSelect);					// Send 0x10 to the ADS1x9x
}

void ads1292r::StopReadDataContinuous (const int chipSelect)
{
  SPICommandData(SDATAC,chipSelect);					// Send 0x11 to the ADS1x9x
}

void ads1292r::SPICommandData(unsigned char dataIn,const int chipSelect)
{
  byte data[1];
  //data[0] = dataIn;
  digitalWrite(chipSelect, LOW);
  delay(2);
  digitalWrite(chipSelect, HIGH);
  delay(2);
  digitalWrite(chipSelect, LOW);
  delay(2);
  SPI.transfer(dataIn);
  delay(2);
  digitalWrite(chipSelect, HIGH);
}

//Sends a write command to ADS1292
void ads1292r::RegWrite (unsigned char READ_WRITE_ADDRESS, unsigned char DATA,const int chipSelect)
{

  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
	          break;
    case 2:
            DATA = DATA & 0xFB;
	          DATA |= 0x80;
	          break;
    case 3:
      	    DATA = DATA & 0xFD;
      	    DATA |= 0x10;
      	    break;
    case 7:
      	    DATA = DATA & 0x3F;
      	    break;
    case 8:
    	      DATA = DATA & 0x5F;
	          break;
    case 9:
      	    DATA |= 0x02;
      	    break;
    case 10:
      	    DATA = DATA & 0x87;
      	    DATA |= 0x01;
      	    break;
    case 11:
      	    DATA = DATA & 0x0F;
      	    break;
    default:
            break;
  }
  // now combine the register address and the command into one byte:
  byte dataToSend = READ_WRITE_ADDRESS | WREG;
  digitalWrite(chipSelect, LOW);
  delay(2);
  digitalWrite(chipSelect, HIGH);
  delay(2);
  // take the chip select low to select the device:
  digitalWrite(chipSelect, LOW);
  delay(2);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(0x00);		//number of register to wr
  SPI.transfer(DATA);		//Send value to record into register
  delay(2);
  // take the chip select high to de-select:
  digitalWrite(chipSelect, HIGH);
}


/* 
Private RegRead
Send read cmd and read 1 ADS1292 register
*/
unsigned char ads1292r::RegRead (unsigned char READ_ADDRESS, const int chipSelect)
{
  unsigned char  dataRead ;
  // now combine the register address and the command into one byte (opcode 1):
  byte dataToSend = READ_ADDRESS | RREG;
  digitalWrite(chipSelect, LOW);
  delay(2);
  digitalWrite(chipSelect, HIGH);
  delay(2);
  // take the chip select low to select the device:
  digitalWrite(chipSelect, LOW);
  delay(2);
  SPI.transfer(dataToSend); //Send register address to be read (opcode 1)
  SPI.transfer(0x00);		// number of register to read (n-1) (opcode 2)
  dataRead = SPI.transfer(0x00);		// Receive and read data from ADS register
  delay(2);
  // take the chip select high to de-select:
  digitalWrite(chipSelect, HIGH);
  return dataRead;
}

/* 
Public getID
method to read chip ID Control Register 
*/
unsigned char ads1292r::getID (const int chipSelect)
{
    return RegRead (0x00, chipSelect); 
}