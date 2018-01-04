#include <OneWireSTM.h>
#include <SoftWire.h>
#include <LiquidCrystal.h>
#include <dht11.h>
#define ON  1
#define OFF 0
#define AUTO       0
#define MANUAL     1
#define LED_STT       PB12   //Led on board STM32f103
#define MODE          29     // PB13  Key 8
#define FAN           6     //PA6       Key3
#define LIGHT         7     //  PA7    key4
#define PUMP_O        30      //PB14    //key 1
#define PUMP_A        17     //  PB1     key6  
#define PUMP_B        26       //PB10     key7
#define VALVE_I       16       //PB0       key5
#define VALVE_O       31       //PB15       key2
#define EC_sensorPin  0
#define PUMP_O_OUT    21  //PB5       
#define DHT11PIN      8     //Pin PA8 for DHT11 
#define WL_Low        9     //PA9
#define WL_High       10    //PA10
//#define FAN_OUT       9     //PA9

////////////////JSON string/////////
//    {"mode":"1","p_o":"1":"0","p_a":"1":"0","p_b":"1":"0","fa":"1":"0","lt":"1":"0","v_i":"1":"0","v_o":"1":"0","wtp":"29.5","bh":"56321","ec":"4021","hu":"42","atp":"26","wl":"50"}

struct keypad
{ 
int Pressed  ;
int Press ;
int Status;
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
struct keypad EX_PORT;
int EC_sensorValue = 0;
OneWire ds (8);                       //PA8
dht11 DHT11;                          //PA8
int DHT11_cnt= 0;
int EX_port_cnt =0;
TwoWire SoftWire(PB6,PB7,SOFT_FAST); // PB6 SCL, PB7 SDA
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
//char Comand_All[20] =   "Fan:ON";

const int rs = 24, en = 25, d4 = 32, d5 = 1, d6 = 4, d7 = 5;//PB8, PB9, PC13, PA1, PA4, PA5
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  pinMode(EC_sensorPin, INPUT_ANALOG);        //EC
  pinMode(LED_STT, OUTPUT);                       //Led status
  Serial.begin(115200);                       //Usb serial Port
  SoftWire.begin();                           //Bh1750
  SoftWire.setClock(400000);                  //Bh1750   
  Serial.println("\nI2C Scanner");   
  ///////////////////Key matrix/////////////////
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
  

 // pinMode(FAN_OUT,OUTPUT);
  Serial2.begin(115200);
  Serial2.println("Hello world! This is the debug channel.");
  
  /////////////////////////////////////////////         
//  lcd.begin_16x2(rs, en, d4, d5, d6, d7);
  lcd.begin(16,2);
   DS_Manager ();
   // BH_Manager();
   EC_Manager();
   DHT_Manager();
   LCD_Manager();
   WL_Manager();
   EX_PORT.Press=0;
   EX_PORT.Pressed =0;
}

// the loop function runs over and over again forever
void loop() {
    GET_Mode();
    if(Mode.Status == AUTO)
    {
     // SENSOR_Manager();
      Serial.println("Mode auto");
      LCD_Manager(); 
      EX_Port();  
    }
 
    else if(Mode.Status == MANUAL)    //Mode Manual control
    {
     // SENSOR_Manager();
      LCD_Manager();
     // GET_Key();
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
  Serial2.print("wtp:");
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
  Fan.Press = digitalRead(FAN); //Press switch Mode
if(Fan.Press == HIGH)
{
  Fan.Pressed = Fan.Press;
  while(Fan.Pressed)
  {
    if(digitalRead(FAN))
    Fan.Pressed =1;
    else 
    Fan.Pressed =0;
  }
  Fan.Status = !Fan.Status;
}
/////////////////////////////////////////////////////////////// 
//////////////////////////Key Pump_O//////////////////////////
  Pump_O.Press = digitalRead(PUMP_O); //Press switch Mode
if(Pump_O.Press == HIGH)
{
  Pump_O.Pressed = Pump_O.Press;
  while(Pump_O.Pressed)
  {
    if(digitalRead(PUMP_O))
    Pump_O.Pressed =1;
    else 
    Pump_O.Pressed =0;
  }
  Pump_O.Status = !Pump_O.Status;
}
///////////////////////////////////////////////////////////////  
//////////////////////////Key Light//////////////////////////
  Light.Press = digitalRead(LIGHT); //Press switch Mode
if(Light.Press == HIGH)
{
  Light.Pressed = Light.Press;
  while(Light.Pressed)
  {
    if(digitalRead(LIGHT))
    Light.Pressed =1;
    else 
    Light.Pressed =0;
  }
  Light.Status = !Light.Status;
}
///////////////////////////////////////////////////////////////

//////////////////////////Key Pump_A//////////////////////////
  Pump_A.Press = digitalRead(PUMP_A); //Press switch Mode
if(Pump_A.Press == HIGH)
{
  Pump_A.Pressed = Pump_A.Press;
  while(Pump_A.Pressed)
  {
    if(digitalRead(PUMP_A))
    Pump_A.Pressed =1;
    else 
    Pump_A.Pressed =0;
  }
  Pump_A.Status = !Pump_A.Status;
}
///////////////////////////////////////////////////////////////  
//////////////////////////Key Pump_B//////////////////////////
  Pump_B.Press = digitalRead(PUMP_B); //Press switch Mode
if(Pump_B.Press == HIGH)
{
  Pump_B.Pressed = Pump_B.Press;
  while(Pump_B.Pressed)
  {
    if(digitalRead(PUMP_B))
    Pump_B.Pressed =1;
    else 
    Pump_B.Pressed =0;
  }
  Pump_B.Status = !Pump_B.Status;
}
/////////////////////////////////////////////////////////////// 
//////////////////////////Key Valve_I//////////////////////////
  Valve_I.Press = digitalRead(VALVE_I); //Press switch Mode
if(Valve_I.Press == HIGH)
{
  Valve_I.Pressed = Valve_I.Press;
  while(Valve_I.Pressed)
  {
    if(digitalRead(VALVE_I))
    Valve_I.Pressed =1;
    else 
    Valve_I.Pressed =0;
  }
  Valve_I.Status = !Valve_I.Status;
}
///////////////////////////////////////////////////////////////
//////////////////////////Key Valve_O//////////////////////////
  Valve_O.Press = digitalRead(VALVE_O); //Press switch Mode
if(Valve_O.Press == HIGH)
{
  Valve_O.Pressed = Valve_O.Press;
  while(Valve_O.Pressed)
  {
    if(digitalRead(VALVE_O))
    Valve_O.Pressed =1;
    else 
    Valve_O.Pressed =0;
  }
  Valve_O.Status = !Valve_O.Status;
}
///////////////////////////////////////////////////////////////
}


Mode.Press = 0;
Mode.Pressed =0;
Mode.WaitRelease = 0;

void GET_Mode ()
{
//////////////////////////Key Mode//////////////////////////
Mode.Press = digitalRead(MODE); //Press switch Mode
if(Mode.Press == 0 && Mode.Pressed ==1 )
{
  if( Mode.WaitRelease == 0)
  {
    Mode.Status = !Mode.Status;
    Mode.Pressed = 0;
  }
  else 
  {
   // Mode.WaitRelease = 0;
  }

}

else if (Mode.Press == 1 && Mode.Pressed ==0)
{
    Mode.Pressed = 1;
    Mode.WaitRelease = 1;
}


    

}



void PORT_Manager()
{
  if(Fan.Status == 1)
  {
      digitalWrite(LED_STT,HIGH);
  }
    else 
    {
      digitalWrite(LED_STT,LOW);
    }
    if(Light.Status == 1)
      Serial.println("Light ON"); 
    else 
      Serial.println("Light OFF");  

    if(Pump_O.Status == 1)
      Serial.println("PUMP_O ON"); 
    else 
      Serial.println("PUMP_O OFF");

    if(Pump_A.Status == 1)
      Serial.println("PUMP_A ON"); 
    else 
      Serial.println("PUMP_A OFF");

    if(Pump_B.Status == 1)
      Serial.println("PUMP_B ON"); 
    else 
      Serial.println("PUMP_B OFF"); 

    if(Valve_I.Status == 1)
      Serial.println("VALVE_I ON"); 
    else 
      Serial.println("VALVE_I OFF");

    if(Valve_O.Status == 1)
      Serial.println("VALVE_O ON"); 
    else 
      Serial.println("VALVE_O OFF");
}
/*
void EX_Port()
{  
   if(EX_PORT.cnt == 800)      //time loop switch case
   {
    EX_PORT.cnt=0;
    switch(EX_PORT.Value){
      case 0: buff[6] = Fan.Status;
              Serial2.print("fa:");
              Serial2.print(buff[6]);
              Serial2.print("\0");
              break;   
      case 1: buff[7] = Light.Status;
              Serial2.print("lt:");
              Serial2.print(buff[7]);
              Serial2.print("\0");
              break; 
      case 2: buff[9] = Pump_A.Status;
              Serial2.print("pa:");
              Serial2.print(buff[9]);
              Serial2.print("\0");  
              break; 
      case 3: buff[10] = Pump_B.Status;
              Serial2.print("pb:");
              Serial2.print(buff[10]);
              Serial2.print("\0");  
              break;   
      case 4: buff[11] = Valve_I.Status;
              Serial2.print("vi:");
              Serial2.print(buff[11]);
              Serial2.print("\0");
              break;  
      case 5: buff[12] = Mode.Status;
              Serial2.print("mo:");
              Serial2.print(buff[12]);
              Serial2.print("\0");
              break;
    }
       EX_PORT.Value++;
       if(EX_PORT.Value==6)   // number of case
       EX_PORT.Value=0;
   }  
   EX_PORT.cnt++;
}
  */
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
      WATERLEVEL.Value = 3;
      buff[5]= WATERLEVEL.Value;
      Serial.println("Sensor Water level error...");
    }
  }
  else
  {
    WATERLEVEL.cnt = digitalRead(WL_High);
    if(WATERLEVEL.cnt==0)
    {
      WATERLEVEL.Value = 1;
      buff[5]= WATERLEVEL.Value;
      Serial.println("Water level 20%");
    }
    else
    {  
      WATERLEVEL.Value = 2;
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
   EX_PORT.Press = Fan.Status; //Press switch Mode
   if(EX_PORT.Press==0 && EX_PORT.Pressed ==0){
     Serial2.print("Fa:0\0\n");
     EX_PORT.Pressed = 1;
   }
   else if(EX_PORT.Press==1&&EX_PORT.Pressed ==1){
     Serial2.print("fa:1\0\n");  
     EX_PORT.Pressed = 0;
   }  
   
  



   

}

