#include <OneWireSTM.h>
#include <SoftWire.h>
#include <LiquidCrystal.h>
#include <dht11.h>
/////////////define status///////////////////////////////
#define ON  1
#define OFF 0
#define AUTO       0
#define MANUAL     1

//#define LED_STT       PB12   //Led on board STM32f103
/////////////////////////define touchpad /////////////////
#define MODE          29     //Key8  PB13  
#define FAN           26     //Key7  PB10
#define LIGHT         17      //Key6  PB1
#define PUMP_O        16     //Key5  PB0
#define PUMP_A        7     //Key4  PA7
#define PUMP_B        6     //Key3  PA6
#define VALVE_I       30     //Key2  PB14
#define VALVE_O       31     //Key1  PB15 

////////////////define out port////////////////
#define FAN_OUT       20   //PB4
#define LIGHT_OUT     27   //PB11
#define PO_OUT        21   //PB5
#define PA_OUT        10   //PA10
#define PB_OUT        9   //PA9
#define VI_OUT        19    //PB3
#define VO_OUT        PA15   //PA15
/////////////// define sensor in///////////////
#define EC_sensorPin  0
#define WL_Low        PB12     //PB12
#define WL_High       PB12    //PA10
#define DHT11PIN      8     //Pin PA8 for DHT11
const int rs = 24, en = 25, d4 = 32, d5 = 1, d6 = 4, d7 = 5;//PB8, PB9, PC13, PA1, PA4, PA5
OneWire ds (8);                       //PA8
dht11 DHT11;                          //PA8
TwoWire SoftWire(PB6,PB7,SOFT_FAST); // PB6 SCL, PB7 SDA

////////////////JSON string/////////
//    {"mode":"1","p_o":"1":"0","p_a":"1":"0","p_b":"1":"0","fa":"1":"0","lt":"1":"0","v_i":"1":"0","v_o":"1":"0","wtp":"29.5","bh":"56321","ec":"4021","hu":"42","atp":"26","wl":"50"}

struct keypad
{ 
int Pressed = 0  ;
int Press= 0 ;
int Status;
int WaitRelease = 0;
int Send=0;

};
struct System 
{
 int  cnt = 0;
 int Value =0;
};

struct keypad Mode;
struct keypad Pump_O;
struct keypad Fan;
struct keypad Pump_A;
struct keypad Pump_B;
struct keypad Valve_I;
struct keypad Valve_O;
struct keypad Light;
struct System SENSOR;
struct System WATERLEVEL;

int EC_sensorValue = 0;
int DHT11_cnt= 0;

int   buff[100];      //Buffer USB serial
char Buff[256];       //Buffer Esp8266
char Comand_Fan[20] =     "Fan:Switch";
char Comand_Mode[20] =    "Mode:Switch";
char Comand_Light[20] =   "Light:Switch";
char Comand_Pump_O[20] =  "Pump_O:Switch";
char Comand_Pump_A[20] =  "Pump_A:Switch";
char Comand_Pump_B[20] =  "Pump_B:Switch";
char Comand_Valve_I[20] = "Valve_I:Switch";
char Comand_Valve_O[20] = "Valve_O:Switch";


LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
 
  pinMode(EC_sensorPin, INPUT_ANALOG);        //EC
  //pinMode(LED_STT, OUTPUT);                       //Led status
  Serial.begin(115200);                       //Usb serial Port
  Serial2.begin(115200);
  SoftWire.begin();                           //Bh1750
  SoftWire.setClock(400000);                  //Bh1750     
 
  pinMode(MODE,INPUT);
  pinMode(PUMP_O,INPUT);
  pinMode(PUMP_A,INPUT);
  pinMode(PUMP_B,INPUT);
  pinMode(VALVE_I,INPUT);
  pinMode(VALVE_O,INPUT);
  pinMode(LIGHT,INPUT); 
  pinMode(FAN,INPUT);
  
  pinMode (WL_Low,INPUT);
  pinMode (WL_High,INPUT);
  
  pinMode(FAN_OUT,OUTPUT);
  pinMode(LIGHT_OUT,OUTPUT);
  
  pinMode(PO_OUT,OUTPUT);
  pinMode(PA_OUT,OUTPUT);
  pinMode(PB_OUT,OUTPUT);
  pinMode(PA15,OUTPUT);
  pinMode(PB3,OUTPUT);
  
  lcd.begin(16,2);
}

// the loop function runs over and over again forever
void loop() {
    GET_Mode();
    if(Mode.Status == AUTO)
    {
      SENSOR_Manager();
      Serial.println("Mode auto");
      LCD_Manager(); 
      EX_Port();  
    }
 
    else if(Mode.Status == MANUAL)    //Mode Manual control
    {
      SENSOR_Manager();
      LCD_Manager();
      GET_Key();
      PORT_Manager();
      EX_Port();
      Serial.println("Mode Manual");
    }
     ESP_Manager();

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
    Serial.println("Value WaterTemp Error");
    Serial.println();
    ds.reset_search();
    delay(250);
    //return;
  }  
  //Serial.print("ROM =");
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
    //  Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
     // Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
     // Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
    //  Serial.println("Device is not a DS18x20 family device.");
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

 // Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
   // Serial.print(data[i], HEX);
   // Serial.print(" ");
  }
  //Serial.print(" CRC=");
 // Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

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
  buff[0] = celsius;
  Serial.print("Temperature = ");
  Serial.print(celsius);
  Serial.println(" Celsius ");
 // Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit");
  Serial2.print("wt:");
  Serial2.print(buff[0]);
  Serial2.print("\0");
}

void BH_Manager ()
{
  byte error, address;
  int nDevices;
  uint16_t bh;
 // Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++) {
  

    SoftWire.beginTransmission(address);
    error = SoftWire.endTransmission();
    
    if (error == 0) {  
    if(address==0x23)
    {
      SoftWire.beginTransmission(address);
      SoftWire.write(0x10);
      SoftWire.endTransmission();
      delay(200);
     // Bh1750_Read(address);
     
      SoftWire.requestFrom(address, 2);
      bh = SoftWire.read();
      bh<<=8;
      bh |= SoftWire.read();
      bh /= 1.2;
      buff[1] = bh;
      Serial2.print("bh:");
      Serial2.print(buff[1]);
      Serial2.print("\0");
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
    ;//Serial.println("done");
}

void EC_Manager()
{
  EC_sensorValue = analogRead(EC_sensorPin);
  buff[2]= EC_sensorValue;
   Serial2.print("ec:");
   Serial2.print(buff[2]);
   Serial2.print("\0");
}

void LCD_Manager()
{  // int cnt;
  lcd.clear();
  delay(1);
  lcd.setCursor(0,0);
   lcd.print(buff[0]);        //WaterTemp
   lcd.print(" ");
    lcd.print(buff[1]);       //bh1750
   lcd.print(" ");
    lcd.print(buff[2]);        //EC
   lcd.print(" ");
    lcd.print(buff[3]);       //Hum
   lcd.print(" ");
  lcd.print(buff[4]);       //Air Temp
   lcd.print(" ");

    if(!Mode.Status)
    {    
    lcd.setCursor(12,1);
      lcd.print("Auto");
    }
      else 
      {       
      lcd.setCursor(10,1);
      lcd.print("Manual");
      }
}

void DHT_Manager()
{
  
  int chk = DHT11.read(DHT11PIN);
  //Serial.println(chk);
  if(chk == 0||chk == -1) {
    int temperature = (DHT11.temperature);
    int humidity = DHT11.humidity;
    buff[3]= humidity;
    buff[4] = temperature;
    if(DHT11_cnt==1)
    {
    Serial2.print("hu:");
    Serial2.print(buff[3]);
    Serial2.print("\0");
    }
    else if(DHT11_cnt ==2)
    {
      
    Serial2.print("at:");
    Serial2.print(buff[4]);
    Serial2.print("\0");
    DHT11_cnt=0;
    }
    DHT11_cnt++;
    
  }
}
void GET_Key ()
 {
//////////////////////////Key Fan//////////////////////////
Fan.Press = digitalRead(FAN); //Press switch fan
if(Fan.Press == 0)
{
  if(Fan.Pressed ==1 && Fan.WaitRelease == 0 )
  {
    Fan.Status = !Fan.Status;
    Fan.Pressed = 0;
  }
  else if (Fan.Pressed ==1 && Fan.WaitRelease == 1) 
  {
   Fan.WaitRelease = 0;
  }
}
else if (Fan.Press == 1)
{ 
   if(Fan.Pressed == 0 &&Fan.WaitRelease == 0 )
   {
    Fan.Pressed = 1;
    Fan.WaitRelease = 1;
   }
} 
//////////////////////////Key Pump_O//////////////////////////
Pump_O.Press = digitalRead(PUMP_O); //Press switch pump o
if(Pump_O.Press == 0)
{
  if(Pump_O.Pressed ==1 && Pump_O.WaitRelease == 0 )
  {
    Pump_O.Status = !Pump_O.Status;
    Pump_O.Pressed = 0;
  }
  else if (Pump_O.Pressed ==1 && Pump_O.WaitRelease == 1) 
  {
   Pump_O.WaitRelease = 0;
  }
}
else if (Pump_O.Press == 1)
{ 
   if(Pump_O.Pressed == 0 &&Pump_O.WaitRelease == 0 )
   {
    Pump_O.Pressed = 1;
    Pump_O.WaitRelease = 1;
   }
}  
//////////////////////////Key Pump_A//////////////////////////
Pump_A.Press = digitalRead(PUMP_A); //Press switch pump a
if(Pump_A.Press == 0)
{
  if(Pump_A.Pressed ==1 && Pump_A.WaitRelease == 0 )
  {
    Pump_A.Status = !Pump_A.Status;
    Pump_A.Pressed = 0;
  }
  else if (Pump_A.Pressed ==1 && Pump_A.WaitRelease == 1) 
  {
   Pump_A.WaitRelease = 0;
  }
}
else if (Pump_A.Press == 1)
{ 
   if(Pump_A.Pressed == 0 &&Pump_A.WaitRelease == 0 )
   {
    Pump_A.Pressed = 1;
    Pump_A.WaitRelease = 1;
   }
} 
//////////////////////////Key Pump_B//////////////////////////
Pump_B.Press = digitalRead(PUMP_B); //Press switch pump b
if(Pump_B.Press == 0)
{
  if(Pump_B.Pressed ==1 && Pump_B.WaitRelease == 0 )
  {
    Pump_B.Status = !Pump_B.Status;
    Pump_B.Pressed = 0;
  }
  else if (Pump_B.Pressed ==1 && Pump_B.WaitRelease == 1) 
  {
   Pump_B.WaitRelease = 0;
  }
}
else if (Pump_B.Press == 1)
{ 
   if(Pump_B.Pressed == 0 &&Pump_B.WaitRelease == 0 )
   {
    Pump_B.Pressed = 1;
    Pump_B.WaitRelease = 1;
   }
} 
//////////////////////////Key Light//////////////////////////
Light.Press = digitalRead(LIGHT); //Press switch light bulb
if(Light.Press == 0)
{
  if(Light.Pressed ==1 && Light.WaitRelease == 0 )
  {
    Light.Status = !Light.Status;
    Light.Pressed = 0;
  }
  else if (Light.Pressed ==1 && Light.WaitRelease == 1) 
  {
   Light.WaitRelease = 0;
  }
}
else if (Light.Press == 1)
{ 
   if(Light.Pressed == 0 &&Light.WaitRelease == 0 )
   {
    Light.Pressed = 1;
    Light.WaitRelease = 1;
   }
}  
//////////////////////////Key Valve_I//////////////////////////
Valve_I.Press = digitalRead(VALVE_I); //Press switch valve in
if(Valve_I.Press == 0)
{
  if(Valve_I.Pressed ==1 && Valve_I.WaitRelease == 0 )
  {
    Valve_I.Status = !Valve_I.Status;
    Valve_I.Pressed = 0;
  }
  else if (Valve_I.Pressed ==1 && Valve_I.WaitRelease == 1) 
  {
   Valve_I.WaitRelease = 0;
  }
}
else if (Valve_I.Press == 1)
{ 
   if(Valve_I.Pressed == 0 &&Valve_I.WaitRelease == 0 )
   {
    Valve_I.Pressed = 1;
    Valve_I.WaitRelease = 1;
   }
}   
//////////////////////////Key Valve_O//////////////////////////
Valve_O.Press = digitalRead(VALVE_O); //Press switch valve out
if(Valve_O.Press == 0)
{
  if(Valve_O.Pressed ==1 && Valve_O.WaitRelease == 0 )
  {
    Valve_O.Status = !Valve_O.Status;
    Valve_O.Pressed = 0;
  }
  else if (Valve_O.Pressed ==1 && Valve_O.WaitRelease == 1) 
  {
   Valve_O.WaitRelease = 0;
  }
}
else if (Valve_O.Press == 1)
{ 
   if(Valve_O.Pressed == 0 &&Valve_O.WaitRelease == 0 )
   {
    Valve_O.Pressed = 1;
    Valve_O.WaitRelease = 1;
   }
} 
}

void GET_Mode ()
{
//////////////////////////Key Mode//////////////////////////
Mode.Press = digitalRead(MODE); //Press switch Mode
if(Mode.Press == 0)
{
  if(Mode.Pressed ==1 && Mode.WaitRelease == 0 )
  {
    Mode.Status = !Mode.Status;
    Mode.Pressed = 0;
  }
  else if (Mode.Pressed ==1 && Mode.WaitRelease == 1) 
  {
   Mode.WaitRelease = 0;
  }
}
else if (Mode.Press == 1)
{ 
   if(Mode.Pressed == 0 &&Mode.WaitRelease == 0 )
   {
    Mode.Pressed = 1;
    Mode.WaitRelease = 1;
   }
} 
}

void PORT_Manager()
{
  if(Fan.Status == 1)
  {
     Serial.println("Fan ON");
      digitalWrite(FAN_OUT,HIGH);
  }
    else 
    {
      Serial.println("Fan OFF");
      digitalWrite(FAN_OUT,LOW);
    }
//////////////////////////////////////////////////////////////////////////    
    if(Light.Status == 1)
    {
      Serial.println("Light ON"); 
     digitalWrite(LIGHT_OUT,HIGH);
    }
    else 
    {
      Serial.println("Light OFF"); 
      digitalWrite(LIGHT_OUT,LOW); 
    }
//////////////////////////////////////////////////////////////////////////
    if(Pump_O.Status == 1)
    {
      Serial.println("PUMP_O ON"); 
      digitalWrite(PO_OUT,HIGH);
    }
    else 
    {
      Serial.println("PUMP_O OFF");
      digitalWrite(PO_OUT,LOW);
    }
//////////////////////////////////////////////////////////////////////////
    if(Pump_A.Status == 1)
    {
      Serial.println("PUMP_A ON"); 
      digitalWrite(PA_OUT,HIGH);
    }
    else 
    {
      Serial.println("PUMP_A OFF");
      digitalWrite(PA_OUT,LOW);
    }
//////////////////////////////////////////////////////////////////////////
    if(Pump_B.Status == 1)
    {
      Serial.println("PUMP_B ON"); 
      digitalWrite(PB_OUT,HIGH);
    }
    else 
    {
      Serial.println("PUMP_B OFF"); 
      digitalWrite(PB_OUT,LOW);
    }
//////////////////////////////////////////////////////////////////////////
    if(Valve_I.Status == 1)
    {
      Serial.println("VALVE_I ON"); 
      digitalWrite(VI_OUT,HIGH);
    }
    else 
    {
      Serial.println("VALVE_I OFF");
      digitalWrite(VI_OUT,LOW);
    }
//////////////////////////////////////////////////////////////////////////
    if(Valve_O.Status == 1)
    {
      Serial.println("VALVE_O ON"); 
      digitalWrite(VO_OUT,ON);
    }
    else 
    {
      Serial.println("VALVE_O OFF");
      digitalWrite(VO_OUT,OFF);
    }
}

void  SENSOR_Manager()
{ 
   if(SENSOR.cnt == 1000)      //time loop switch case
   {   
    SENSOR.cnt=0; 
    switch(SENSOR.Value){
      case 0: 
              DS_Manager ();
              break;
      case 1: BH_Manager();
              break;
      case 2: EC_Manager();
              break;
      case 3: DHT_Manager();
              break; 
      case 4: WL_Manager();
              break;        
   }
       SENSOR.Value++;
       if(SENSOR.Value==5)   // number of case
       SENSOR.Value=0;
   }  
   SENSOR.cnt++; 
}
void ESP_Manager()
{
  int i,legth;
  legth= Serial2.available();
    if(legth)
     {
       for(i = 0; i < legth; i++)
       {
       Buff[i] = Serial2.read();
       }
       if (strcmp(Buff,Comand_Fan)==0)
       {
         Fan.Status=!Fan.Status;
       }else if (strcmp(Buff,Comand_Mode)==0)
       {
     
         Mode.Status=!Mode.Status;
       }else if (strcmp(Buff,Comand_Pump_O)==0)
       {
 
         Pump_O.Status=!Pump_O.Status;
       }else if (strcmp(Buff,Comand_Pump_A)==0)
       {
 
         Pump_A.Status=!Pump_A.Status;
       }else if (strcmp(Buff,Comand_Pump_B)==0)
       {
       
         Pump_B.Status=!Pump_B.Status;
       }else if (strcmp(Buff,Comand_Valve_I)==0)
       {
         
         Valve_I.Status=!Valve_I.Status;
       }else if (strcmp(Buff,Comand_Valve_O)==0)
       {
      
         Valve_O.Status=!Valve_O.Status;
       }else if (strcmp(Buff,Comand_Light)==0)
       {
        
         Light.Status=!Light.Status;
       }else 
       {
         for (i=0;i<legth;i++)
         Serial.println(Buff[i]);
         Serial.println("Wrong String");
       }
        memset(Buff, 0, sizeof(Buff));
     }
}     
void WL_Manager()
{
  WATERLEVEL.cnt = digitalRead(WL_Low);
  
  if(WATERLEVEL.cnt == 0)
  {
    WATERLEVEL.cnt = digitalRead(WL_High);
    if(WATERLEVEL.cnt==0)
    {
      WATERLEVEL.Value = 0;
      buff[5]= WATERLEVEL.Value;
      Serial.println("Water level 0%");
    }
    else
    {
      WATERLEVEL.Value = 0;
      buff[5]= WATERLEVEL.Value;
      Serial.println("Sensor Water level error...");
    }
  }
  else
  {
    WATERLEVEL.cnt = digitalRead(WL_High);
    if(WATERLEVEL.cnt==0)
    {
      WATERLEVEL.Value = 25;
      buff[5]= WATERLEVEL.Value;
      Serial.println("Water level 20%");
    }
    else
    {  
      WATERLEVEL.Value = 100;
      buff[5]= WATERLEVEL.Value;
      Serial.println("Water level 100%");
    }
  }
    Serial2.print("wl:");
    Serial2.print(buff[5]);
    Serial2.print("\0");
}

void EX_Port()
{
  /////////////////Send Pump_O Status///////////////
  if(Pump_O.Status==0 && Pump_O.Send ==0){
     Serial2.print("po:0");
     Serial2.print("\0");
     Pump_O.Send = 1;
   }
   else if(Pump_O.Status==1&&Pump_O.Send ==1){
     Serial2.print("po:1");
     Serial2.print("\0");  
     Pump_O.Send = 0;
   }  
   /////////////////////////////////////////////
  /////////////////Send Pump_A Status///////////////
  if(Pump_A.Status==0 && Pump_A.Send ==0){
     Serial2.print("pa:0");
     Serial2.print("\0");
     Pump_A.Send = 1;
   }
   else if(Pump_A.Status==1&&Pump_A.Send ==1){
     Serial2.print("pa:1");  
     Serial2.print("\0");
     Pump_A.Send = 0;
   }  
   /////////////////////////////////////////////
  /////////////////Send Pump_B Status///////////////
  if(Pump_B.Status==0 && Pump_B.Send ==0){
     Serial2.print("pb:0");
     Serial2.print("\0");
     Pump_B.Send = 1;
   }
   else if(Pump_B.Status==1&&Pump_B.Send ==1){
     Serial2.print("pb:1"); 
     Serial2.print("\0"); 
     Pump_B.Send = 0;
   }  
   /////////////////////////////////////////////
  /////////////////Send Fan Status///////////////
  if(Fan.Status==0 && Fan.Send ==0){
     Serial2.print("fa:0");
     Serial2.print("\0");
     Fan.Send = 1;
   }
   else if(Fan.Status==1&&Fan.Send ==1){
     Serial2.print("fa:1"); 
     Serial2.print("\0"); 
     Fan.Send = 0;
   }  
   /////////////////////////////////////////////
  /////////////////Send Light Status///////////////
  if(Light.Status==0 && Light.Send ==0){
     Serial2.print("lt:0");
     Serial2.print("\0");
     Light.Send = 1;
   }
   else if(Light.Status==1&&Light.Send ==1){
     Serial2.print("lt:1");  
     Serial2.print("\0");
     
     Light.Send = 0;
   }  
   /////////////////////////////////////////////
  /////////////////Send Mode Status///////////////
  if(Mode.Status==0 && Mode.Send ==0){
     Serial2.print("mo:0");
     Serial2.print("\0");
     Mode.Send = 1;
   }
   else if(Mode.Status==1&&Mode.Send ==1){
     Serial2.print("mo:1");
     Serial2.print("\0");  
     Mode.Send = 0;
   }  
   /////////////////////////////////////////////

}


