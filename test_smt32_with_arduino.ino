#include <OneWireSTM.h>
#include <SoftWire.h>
#include <LiquidCrystal.h>

#define BLYNK_PRINT Serial 
#define LED PB12   //Định nghĩa chân LED
int EC_sensorPin = 0; 
int EC_sensorValue = 0;
OneWire ds (8);
OneWire DHT(8);
TwoWire SoftWire(PB6,PB7,SOFT_FAST);
//String Buffer;
//byte buff[];
const int rs = 1, en = 2, d4 = 3, d5 = 4, d6 = 5, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {
  pinMode(EC_sensorPin, INPUT_ANALOG);        //EC
  pinMode(LED, OUTPUT);                       //Led status
  Serial.begin(115200);                       //Usb serial Port
  SoftWire.begin();                           //Bh1750
  SoftWire.setClock(400000);                  //Bh1750   
  Serial.println("\nI2C Scanner");            
  lcd.begin(16,2);
  lcd.print("Hello World!");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED, HIGH); 
  delay(200);// turn the LED on (HIGH is the voltage level)
  DS_Manager ();
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(200);
//  BH_Manager();
  EC_Manager();
  LCD_Manager();
  DHT_Manager();
 /*  Wire.beginTransmission(0x23);
      __wire_write(0x23);
      Wire.endTransmission();
      */
 /* uint16_t lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);
*/
}
void DS_Manager ()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
}

void BH_Manager ()
{
  byte error, address;
  int nDevices;
  uint16_t bh;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    SoftWire.beginTransmission(address);
    error = SoftWire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
///////////////////////////////Read Lux///////////////      
    if(address==0x23)
    {
      SoftWire.beginTransmission(address);
      SoftWire.write(0x10);
      SoftWire.endTransmission();
      delay(200);
     // Bh1750_Read(address);
     
      SoftWire.requestFrom(address, 2);
      bh = SoftWire.read();
 //     Serial.print("Highbyte BH1750: ");
  //    Serial.println(bh,HEX);
      bh<<=8;
      bh |= SoftWire.read();
      Serial.print("Value RAW Bh1750:  ");
      Serial.println(bh,HEX);
      bh /= 1.2;
      Serial.print(F("[BH1750] Converted value: "));
      Serial.print(bh);
      Serial.println("Lux");
      delay(200);
    }
////////////////////////////////////////////////////////
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
}

void EC_Manager()
{
  EC_sensorValue = analogRead(EC_sensorPin);
   Serial.print("Value EC: ");
   Serial.print(EC_sensorValue);
   Serial.println(" PPM");
}
void LCD_Manager()
{
  lcd.setCursor(0,1);
  lcd.print("How are you");
}
void DHT_Manager()
{
  float Hum, TempAir;
    
  DHT.write_bit(0x00);
  delay(20);
  Hum = DHT.read();
  Serial.print("Gia tri do am la:");
  Serial.println(Hum);
  
  
}

