//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   Heartrate and respiration computation based on original code from Texas Instruments
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
#include "ecg_respiration_samples.h"
#include <SPI.h>

volatile uint8_t global_HeartRate; 
volatile uint8_t global_RespirationRate=0;

//Pin declartion the other you need are controlled by the SPI library
const int ADS1292_DRDY_PIN = 26;//26 Kalam //26 Healthy pi  
const int ADS1292_CS_PIN = 13;//17 Kalam//13 Healthy pi  
const int ADS1292_START_PIN = 14;//16 Kalam//14 Healthy pi  
const int ADS1292_PWDN_PIN = 27;//2 Kalam//27 Healthy pi  
                
int16_t ecg_wave_buff, ecg_filterout;
int16_t res_wave_buff,resp_filterout;

long time_elapsed=0;  

ads1292r ADS1292R; // define class ads1292r
ecg_respiration_algorithm ECG_RESPIRATION_ALGORITHM; // define class ecg_algorithm

void setup()
{
  delay(2000); // initalize the  data ready and chip select pins:
  pinMode(ADS1292_DRDY_PIN, INPUT);  //6
  pinMode(ADS1292_CS_PIN, OUTPUT);    //7
  pinMode(ADS1292_START_PIN, OUTPUT);  //5
  pinMode(ADS1292_PWDN_PIN, OUTPUT);  //4
  Serial.begin(115200);  // Baud rate for serial communica
  ADS1292R.ads1292_Init(ADS1292_CS_PIN,ADS1292_PWDN_PIN,ADS1292_START_PIN);  //initalize ADS1292 slave
  Serial.println("Initiliziation is done");
}

void loop()
{
  ads1292_output_values ecg_respiration_values;
  boolean ret = ADS1292R.ads1292_ecg_and_respiration_samples(ADS1292_DRDY_PIN,ADS1292_CS_PIN,&ecg_respiration_values);
  
  if (ret == true)
  {
    ecg_wave_buff = (int16_t)(ecg_respiration_values.s_Daq_Vals[1] >> 8) ;  // ignore the lower 8 bits out of 24bits 
    Serial.println(ecg_wave_buff);
    res_wave_buff = (int16_t)(ecg_respiration_values.sresultTempResp>>8) ; 
    Serial.println(res_wave_buff);

    if(ecg_respiration_values.leadoff_detected == false) 
    {
      ECG_RESPIRATION_ALGORITHM.ECG_ProcessCurrSample(&ecg_wave_buff, &ecg_filterout);   // filter out the line noise @40Hz cutoff 161 order
      ECG_RESPIRATION_ALGORITHM.QRS_Algorithm_Interface(ecg_filterout,&global_HeartRate);// calculate 
      resp_filterout = ECG_RESPIRATION_ALGORITHM.Resp_ProcessCurrSample(res_wave_buff); 
      ECG_RESPIRATION_ALGORITHM.RESP_Algorithm_Interface(resp_filterout,&global_RespirationRate);

    }else{
      
      ecg_filterout = 0; 
      resp_filterout = 0;
    }
    
    if(millis() > time_elapsed)  // update every one second
    {
      if(ecg_respiration_values.leadoff_detected == true) // lead in not connected
      {
        Serial.println("ECG lead error!!! ensure the leads are properly connected");
      }else{
        
        Serial.print("Heart rate: ");
        Serial.print(global_HeartRate);
        Serial.println("BPM");
        Serial.print("Respiration Rate :");
        Serial.println(global_RespirationRate);
      }
      time_elapsed += 1000;
    }
  }    
 }
