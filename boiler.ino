



//Sample using LiquidCrystal library

#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
/*******************************************************

  This program will test the LCD panel and the buttons
  Mark Bramwell, July 2010

********************************************************/

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


// Pass our oneWire reference to Dallas Temperature.


// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define ONE_WIRE_BUS 2

#define RELAY1  4                        
#define RELAY2  11                       
#define RELAY3  12                        

#define INFO_MODE 0;
#define SET_TEMPERATURE_MODE 1
#define SET_RELAY_MODE 2

int mode = INFO;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int count = 0;
// read the buttons

int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 250)  return btnUP;
  if (adc_key_in < 450)  return btnDOWN;
  if (adc_key_in < 650)  return btnLEFT;
  if (adc_key_in < 850)  return btnSELECT;

  // For V1.0 comment the other threshold and use the one below:
  /*
    if (adc_key_in < 50)   return btnRIGHT;
    if (adc_key_in < 195)  return btnUP;
    if (adc_key_in < 380)  return btnDOWN;
    if (adc_key_in < 555)  return btnLEFT;
    if (adc_key_in < 790)  return btnSELECT;
  */


  return btnNONE;  // when all others fail, return this...
}

struct Settings {
  float delta;
  int sensor_map_id;
};

Settings settings;

void setup()
{

  
  EEPROM.get(0, settings);
  pinMode(RELAY1, OUTPUT);       
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);

  digitalWrite(RELAY1,HIGH);          // Turns Relay Off
  digitalWrite(RELAY2,HIGH);
  digitalWrite(RELAY3,HIGH);
  
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  sensors.begin();
  lcd.begin(16, 2);              // start the library
  lcd.setCursor(0, 1);
  lcd.print("                "); 
  lcd.print("Init:           "); 
  
  // init settings
  
}

byte sensor_map[6][3]={
 {0,1,2},
 {0,2,1},
 {1,0,2},
 {1,2,0},
 {2,0,1},
 {2,1,0}
};

void print_temperature(){
  lcd.setCursor(0, 0);           // move cursor to second line "1" and 9 spaces over
  for(int i=0;i<3;i++)
    lcd.print(sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][i]),1);      // display seconds elapsed since power-up
    lcd.print(";");
  }
    lcd.print("  ");  
}


unsigned long last_keypress=0;

void check_idle(){

  if (abs(millis()-last_keypress)>5000){
    mode = 0;  
  }
  
}

void logic(){
  float t_out = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][2]); //outcome water t
  float t_in = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][1]);  //income water t
  float t_outdoor = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][0]); // outdoor t
  float target_t;
  float hysteresis = 1;
  target_t =43-0.8*t_outdoor;
  if ( target_t+hysteresis < t_in ){
    power_relay_on()
  } else if(target_t-hysteresis>t_in){
    power_relay_off();
  }
}

void relay_status();

void power_relay_on(){
  digitalWrite(RELAY1,LOW);
  digitalWrite(RELAY2,LOW);
  digitalWrite(RELAY3,LOW);
};

void power_relay_off(){
  digitalWrite(RELAY1,HIGH);
  digitalWrite(RELAY2,HIGH);
  digitalWrite(RELAY3,HIGH);
};


void process_key(int lcd_key){
  
  last_keypress = millis();
  
  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
        break;
    case btnLEFT:
        break;
    case btnUP:
        break;
    case btnDOWN:
        break;
    case btnSELECT:
        break;
    case btnNONE:
        break;
  }

}

void loop()
{
 
 lcd_key = read_LCD_buttons();
 if(lcd_key!=5){
    process_key(lcd_key);
 }
 
 check_idle();
 sensors.requestTemperatures();
 if(mode==0){
  print_temperature();
  logic();
  print_relay_status();
 }
 lcd.setCursor(0, 0); 
  return ;

}


