//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   Heartrate and respiration computation based on original code from Texas Instruments
//
//   This example plots ECG and respiartion wave through protocentral_openview processing GUI.
//   GUI URL: https://github.com/Protocentral/protocentral_openview.git
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
//   If you have bought the breakout the connection with the Arduino board is as follows:
//
//  |ads1292r pin label| Arduino Connection   |Pin Function      |
//  |----------------- |:--------------------:|-----------------:|
//  | VDD              | +5V                  |  Supply voltage  |
//  | PWDN/RESET       | D4                   |  Reset           |
//  | START            | D5                   |  Start Input     |
//  | DRDY             | D6                   |  Data Ready Outpt|
//  | CS               | D7                   |  Chip Select     |
//  | MOSI             | D11                  |  Slave In        |
//  | MISO             | D12                  |  Slave Out       |
//  | SCK              | D13                  |  Serial Clock    |
//  | GND              | Gnd                  |  Gnd             |
//
/////////////////////////////////////////////////////////////////////////////////////////


#include "ads1292r.h"
#include <SPI.h>

//Pin declartion the other you need are controlled by the SPI library
const int ADS1292_DRDY_PIN = 6;
const int ADS1292_CS_PIN = 7;
const int ADS1292_START_PIN = 5;
const int ADS1292_PWDN_PIN = 4;

ads1292r ADS1292R;


void setup()
{


  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  //CPOL = 0, CPHA = 1
  SPI.setDataMode(SPI_MODE1);
  // Selecting 1Mhz clock for SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  pinMode(ADS1292_DRDY_PIN, INPUT);
  pinMode(ADS1292_CS_PIN, OUTPUT);
  pinMode(ADS1292_START_PIN, OUTPUT);
  pinMode(ADS1292_PWDN_PIN, OUTPUT);

  Serial.begin(57600);
  ADS1292R.Init(ADS1292_CS_PIN,ADS1292_PWDN_PIN,ADS1292_START_PIN);

  Serial.println("starting");
  delay(1000);
  Serial.println("ID: ");
  ADS1292R.StopReadDataContinuous(ADS1292_CS_PIN);
  //Serial.println(ads1292getID(), BIN);
  print8bits(ADS1292R.getID(ADS1292_CS_PIN));

  Serial.println("Initiliziation is done");
}

void loop()
{
 
}


void print8bits(int var) {
  for (unsigned int test = 0x80; test; test >>= 1) {
    Serial.write(var  & test ? '1' : '0');
  }
  Serial.println();
}
