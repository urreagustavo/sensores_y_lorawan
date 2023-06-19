#include "LoRaWan_APP.h" //Librería de LoRaWan para arduino que soporta el Heltec HTCC-AB01
#include "Arduino.h"    //Librería de arduino que permite trabajar con diferentes micros (AVR) y con el IDE(Integrated Development Environment)de manera más completa

/*Para el cube cell
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */
 
#include <Wire.h>
#include "SparkFunHTU21D.h"

//Create an instance of the object
HTU21D myHumidity;
// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin 
// Connect SDA to I2C data pin 

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x05, 0x4c, 0x96 };  
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x3c, 0x7f, 0xfa, 0x9c, 0x48, 0x7a, 0x20, 0x09, 0x14, 0x6e, 0xb5, 0x7c, 0xf3, 0x1b, 0xcd, 0xbd };

/* ABP para*/
uint8_t nwkSKey[] = { 0x91, 0xf9, 0xdc, 0x70, 0x4d, 0xae, 0xa8, 0x7d, 0x4c, 0xf3, 0x42, 0xb3, 0x27, 0x14, 0x1d,0x4a };
uint8_t appSKey[] = { 0xaf, 0x03, 0x4c, 0x42, 0x7f, 0xbc, 0xe2, 0x44, 0xdb, 0xe2, 0x93, 0x6d, 0xdf, 0xe2, 0x68,0xde };
uint32_t devAddr =  ( uint32_t )0x260C3514;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION; 

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 180000; // 3 min    

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR (Adaptative Data Rate) enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

int ledPower = GPIO5;   //Global variable which connect 3 led driver pins of dust sensor to HTCC-AB01 GPIO5 pin

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
    // Vext ON
    digitalWrite(Vext, LOW);
    delay(100);
    
  /* For Smoke detection*/

    // Variables    
    int measurePin = ADC; //Connect dust sensor to HTCC-AB01 ADC pin
    int samplingTime = 280;
    int deltaTime = 40;
    int sleepTime = 9680;
    float voMeasured = 0;
    float calcVoltage = 0;
    float dustDensity = 0;
    int smk = 0; //Smoke flag: 0 there is not smoke, 1 there is smoke
    
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);

    voMeasured = analogRead(measurePin); // read the dust value

    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);

    // 0 - 5V mapped to 0 - 1023 integer values
    // recover voltage
    calcVoltage = voMeasured * (5.0 / 1024.0);

    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    dustDensity = 0.170 * calcVoltage - 0.1;
  
    Serial.print("- Dust Density: ");
    Serial.print(dustDensity,3); // unit: mg/m3
    Serial.print(" mg/m3"); // unit
    delay(1000);
    
    // Send 1 if there is smoke or 0 if there is not smoke
    if (dustDensity>2.5)
    {
        Serial.print("\t Smoke \n");
        smk = 1;
    }
    else
    {
        Serial.print("\t No Smoke \n");
        smk = 0;
    }

  /*For temperature and humidity*/
    
    myHumidity.begin();
    float humd = myHumidity.readHumidity();
    float temp = myHumidity.readTemperature();
    
    // Vext OFF
    digitalWrite(Vext, HIGH);

    Serial.print("- Temperature: ");
    if (temp == ERROR_I2C_TIMEOUT) {
      Serial.print("i2c timeout");
    } else if (temp == ERROR_BAD_CRC) {
      Serial.print("bad crc");
    } else {
      Serial.print(temp, 2);
      Serial.println(" ºC");
    }
    Serial.print("- Humidity: ");
    Serial.print(humd, 2);
    Serial.println(" % \n");  
    delay(1000);

    

  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  //Para cambiar el conenido del mensaje
    appDataSize = 3;
    appData[0] = temp;
    appData[1] = humd;
    appData[2] = smk;
}

void setup() {
  pinMode(GPIO6, OUTPUT);
  pinMode(ledPower,OUTPUT);
  Serial.begin(115200);
  //LoRaWAN
#if(AT_SUPPORT)
  enableAt();
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass,loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      //delay(300000);  // Agregado por mi para que transmita cada 5mi. No funcionó!
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      digitalWrite(GPIO6, HIGH); // - GPIO 6 --> LOW --> Vext(3.3V/300mA) ON 
                                 // - GPIO 6 --> HIGH --> Vext(3.3V/300mA) OFF
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
