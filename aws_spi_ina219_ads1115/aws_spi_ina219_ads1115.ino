
#include <Arduino.h>        // Core Arduino
#include <Wire.h>   //  I2C communication SDA,SCL



#include <Adafruit_ADS1X15.h> //  ADS1115 16-bit ADC
#include <Adafruit_INA219.h>  //  INA218  12-bit current sensing IC using shunt

#include "FS.h"
#include <ESP8266WiFi.h>;
#include <PubSubClient.h>;
#include <NTPClient.h>;
#include <WiFiUdp.h>;

const char *ssid = "WIFI id";
const char *password = "WIFI passowrd";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

const char *AWS_endpoint = "axxxxxxxxx-ats.iot.us-west-2.amazonaws.com"; // MQTT broker ip

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

WiFiClientSecure espClient;
PubSubClient client(AWS_endpoint, 8883, callback, espClient); // set MQTT port number to 8883 as per //standard
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  espClient.setBufferSizes(512, 512);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  while (!timeClient.update())
  {
    timeClient.forceUpdate();
  }

  espClient.setX509Time(timeClient.getEpochTime());
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("SPI_testing"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      char buf[256];
      espClient.getLastSSLError(buf, 256);
      Serial.print("WiFiClientSecure SSL error: ");
      Serial.println(buf);

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


//#define SERIAL_BAUD_RATE 115200

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

int16_t results_ads115;

 /* Be sure to update this value based on the IC and the gain settings! */
    float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
    //float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
  

void print_ads_readings(){
    

   
    results_ads115 = ads1115.readADC_Differential_0_1();
  
    Serial.print("Differential: "); Serial.print(results_ads115); Serial.print("("); Serial.print(results_ads115 * multiplier); Serial.println("mV)");

  }

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

void print_ina_readings(){
     
    
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


void setup_aws_mqtt(){
      // initialize digital pin LED_BUILTIN as an output.
      pinMode(LED_BUILTIN, OUTPUT);
      setup_wifi();
      delay(1000);
      if (!SPIFFS.begin())
      {
        Serial.println("Failed to mount file system");
        return;
      }
    
      Serial.print("Heap: ");
      Serial.println(ESP.getFreeHeap());
    
      // Load certificate file , may not work on other systems , check libs version ,and load certificates SPIFFs
      File cert = SPIFFS.open("/cert.der", "r"); // replace cert.crt eith your uploaded file name
      if (!cert)
      {
        Serial.println("Failed to open cert file");
      }
      else
        Serial.println("Success to open cert file");
    
      delay(1000);
    
      if (espClient.loadCertificate(cert))
        {Serial.println("cert loaded");}
      else
        {Serial.println("cert not loaded");}
    
      // Load private key file
      File private_key = SPIFFS.open("/private.der", "r"); // replace private eith your uploaded file name
      if (!private_key)
      {
        Serial.println("Failed to open private cert file");
      }
      else
        Serial.println("Success to open private cert file");
    
      delay(1000);
    
      if (espClient.loadPrivateKey(private_key))
        Serial.println("private key loaded");
      else
        Serial.println("private key not loaded");
    
      // Load CA file
      File ca = SPIFFS.open("/ca.der", "r"); // replace ca eith your uploaded file name
      if (!ca)
      {
        Serial.println("Failed to open ca ");
      }
      else
        Serial.println("Success to open ca");
    
      delay(1000);
    
      if (espClient.loadCACert(ca))
        Serial.println("ca loaded");
      else
        Serial.println("ca failed");
    
    
        //any set up, if required
         
    
    
      Serial.print("Heap: ");
      Serial.println(ESP.getFreeHeap());
  }
  
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Hello!");
  
  setup_ads1115();
  setup_ina219();

  setup_aws_mqtt();
  

}


String toAwsData = "";

String support_str = "";

void loop_aws(){
  
        if (!client.connected())
        {
          reconnect(); // try to reconnect
        }
        
        client.loop(); // Keeps the connection alive

        // Sending message every 2 second interval
        long now = millis();
        if (now - lastMsg > 2000)
        {
          lastMsg = now;
          ++value;
          snprintf(msg, 75, "{\"message\": \"hello world #%ld\"}", value);
          Serial.print("Publish message: ");
      //    Serial.println(msg);
      //    client.publish("outTopic", msg);

           // Publishing the Data to aws topic
           boolean ok = client.publish("outTopic", toAwsData.c_str());
           
           if (ok)? Serial.println("AWS send Pass!") : Serial.println("AWS send fail!"); 
           
           Serial.println(toAwsData); 
        
        
          Serial.print("Heap: ");
          Serial.println(ESP.getFreeHeap()); // Low heap can cause problems
        }

        
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
        delay(100);                      // wait for a second
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
        delay(100);                      // wait for a second
      
      
//       Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(results * multiplier); Serial.println("mV)");

//       Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
//      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
//      Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
//      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
//      Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
//      Serial.println("");

          //JSON data
          toAwsData = support_str + "{" +
                      "\"differential_mv\":" +  "\""+ String((results_ads115 * multiplier),7) + "\"," +
                      "\"current_mA\":"      +  "\""+ String(current_mA,7)                    + "\","  +
                      "\"shuntvoltage\":"    +  "\""+ String(shuntvoltage,3)                  + "\""  +
                      "}";

                      
                      
      smartDelay(1000);
      
        
      
        
      
//        if (millis() > 5000 )
//          {Serial.println(F("No  data received: check wiring"));}
          
}





//// This custom version of delay() ensures that the data object
//// is being "fed".
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do 
    {
      while (ss.available())
        gps.encode(ss.read());
    } while (millis() - start < ms);
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("------------ADS1115------------");
  
  print_ads_readings();
  Serial.println("------------INA219-------------");
  
  print_ina_readings();

  loop_aws();
  
  delay(1000); // Can be replaced by millis concept;

}


//Resources:
//https://www.best-microcontroller-projects.com/ads1115.html
//https://www.best-microcontroller-projects.com/ina219.html
