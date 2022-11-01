
#include <Arduino.h>        // Core Arduino
#include <Wire.h>   //  I2C communication SDA,SCL



#include <Adafruit_ADS1X15.h> //  ADS1115 16-bit ADC
#include <Adafruit_INA219.h>  //  INA218  12-bit current sensing IC using shunt




#define SERIAL_BAUD_RATE 115200

// Use i2c scanner to get the bus addresses , https://github.com/RalphBacon/ADS1115-ADC/blob/master/I2C_HEX_ADDRESS_SCANNER.ino
#define I2C_ADD_ADS1115  0x48  // Bus HEX code of ADS1115   also,  0x48, 0x49, 0x4a, 0x4b.
#define I2C_ADD_INA219 0x45   // Bus HEX code of INA219  also ,   0x40, 0x41, 0x42, 0x43,.... 0x4C, 0x4D, 0x4E, 0x4F.  


Adafruit_INA219 ina219;
Adafruit_ADS1115 ads1115;


void setup_ina219(){
     // Initialize the INA219.
    // By default the initialization will use the largest range (32V, 2A).  However
    // you can call a setCalibration function to change this range (see comments).

    
    if (! ina219.begin()) {
      Serial.println("Failed to find INA219 chip");
      while (1) { delay(10); }
    }
    else
    {
      Serial.println("Success to initialize INA219.");
      delay(700);
    }
    
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    //ina219.setCalibration_32V_1A();
    
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    //ina219.setCalibration_16V_400mA();
  
    Serial.println("Measuring voltage and current with INA219 ...");
  }

  
void setup_ads1115()
{
  // Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  // Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  ads1115.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads1115.begin())
  {
    Serial.println("Failed to initialize ADS.");
    delay(2000);

  }
  else
  {
    Serial.println("Success to initialize ADS.");
    delay(700);
  }
}


void print_ads_readings(){
    int16_t results;

    /* Be sure to update this value based on the IC and the gain settings! */
    float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
    //float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
  
    results = ads1115.readADC_Differential_0_1();
  
    Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(results * multiplier); Serial.println("mV)");

  }


void print_ina_readings(){
      float shuntvoltage = 0;
      float busvoltage = 0;
      float current_mA = 0;
      float loadvoltage = 0;
      float power_mW = 0;
    
      shuntvoltage = ina219.getShuntVoltage_mV();
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      power_mW = ina219.getPower_mW();
      loadvoltage = busvoltage + (shuntvoltage / 1000);
      
      Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
      Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
      Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
      Serial.println("");
        
  }

  
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("Hello!");

  setup_ads1115();
  setup_ina219();
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("------------ADS1115------------");
  
  print_ads_readings();
  Serial.println("------------INA219-------------");
  
  print_ina_readings();
  
  delay(1000); // Can be replaced by millis concept;

}


//Resources:
//https://www.best-microcontroller-projects.com/ads1115.html
//https://www.best-microcontroller-projects.com/ina219.html
