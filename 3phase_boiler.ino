// WARNING!!!
// This code was created for study and tests purposes for my personal use only.
// Any reproducing of the unit and this code usage with high voltage devices is at your own risk!

#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

/* 
   DS1307RTC required for day-night feature of heating algorithm 
   and it will not require ATMega328P board because of firmware memory consumption) 


#include <TimeLib.h>  
  #include <Time.h>
  #include <DS1307RTC.h> */

#define ONE_WIRE_BUS 2

#define INFO_MODE 0;
#define SET_TEMPERATURE_MODE 1
#define SET_RELAY_MODE 2
#define SENSOR_RESOLUTION 10

int relay_map[3] = {11, 10, 12}; // relay pins
boolean relay_state[3];
int t_watchdog = 0;
unsigned long last_keypress = 0;

unsigned long timeout0 = 0;
byte print_mode_0 = 0;

byte sensors_map[6][3] = {
  {0, 1, 2},
  {0, 2, 1},
  {1, 0, 2},
  {1, 2, 0},
  {2, 0, 1},
  {2, 1, 0}
};

float temperatures[3] = { 0, 0, 0 };
float hysteresis = 2;

float target_t;
float last_temp_in = 0;
boolean must_grow = true;

int mode = INFO_MODE;
int lcd_key     = 0;
int adc_key_in  = 0;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//LiquidCrystal_I2C lcd(0x3F, 16, 2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct Settings {
  float delta;
  byte sensor_map_id;
};

volatile byte sensor_map_id;
volatile float delta;

unsigned long previousMillis = 0;
unsigned long logicMillis = 0;
unsigned long pause_switch_logic_millis = 0; 
Settings settings;

DeviceAddress sensorDeviceAddress;
boolean saved = true;

void setup()
{

  Serial.begin(9600);
  /*setSyncProvider(RTC.get);
    if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
    else
    Serial.println("RTC has set the system time");
  */

  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.print("Init:           ");



  EEPROM.get(0, settings);

  if (isnan(settings.sensor_map_id) ) { // First Time check
    Serial.println("set default settings");
    settings.sensor_map_id = 0;
    settings.delta = 2;
  }

  sensor_map_id = settings.sensor_map_id;
  delta = settings.delta;
  saved = true;
  for (byte i = 0; i < 3; i++) {
    pinMode(relay_map[i], OUTPUT);
  }
  all_relays_off();


  sensors.begin();
  sensors.setResolution(SENSOR_RESOLUTION);
  //sensors.setWaitForConversion(false);
  //////Serial.println(sensors.getDeviceCount());
  //sensors.requestTemperatures();

  sensors.getAddress(sensorDeviceAddress, 0);
  printAddress(sensorDeviceAddress);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);

  sensors.getAddress(sensorDeviceAddress, 1);
  printAddress(sensorDeviceAddress);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);
  sensors.getAddress(sensorDeviceAddress, 2);
  printAddress(sensorDeviceAddress);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);


  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  PCICR =  0b00000010;
  PCMSK1 = 0b00001110;

}

volatile byte seqA = 0;
volatile byte seqB = 0;
volatile byte seqC = 0;
volatile byte cnt1 = 0;
volatile byte cnt2 = 0;


int volume = 0;
byte up = 0;
byte isClick = 0;

volatile byte reading = 0;
#define ENCODER_LEFT 0x01
#define ENCODER_RIGHT 0x02
#define ENCODER_DOWN 0x04

// Work with Encoder

ISR(PCINT1_vect) {

  boolean C_val = digitalRead(A3);
  boolean A_val = digitalRead(A1);
  boolean B_val = digitalRead(A2);
  byte enc = 0;

  seqA <<= 1;
  seqA |= A_val;

  seqB <<= 1;
  seqB |= B_val;

  seqA &= 0b00001111;
  seqB &= 0b00001111;

  if (seqA == 0b00001001 && seqB == 0b00000011)
  {
    enc |= ENCODER_RIGHT;
  }

  if (seqA == 0b00000011 && seqB == 0b00001001)
  {
    enc |= ENCODER_LEFT;
  }

  if (!C_val) {
    enc |= ENCODER_DOWN;
  }

  input(enc);
}


void input(byte enc) {
  //Serial.println(enc);
  saved = true;
  last_keypress = millis();
  timeout0 = millis();

  if ((enc & ENCODER_DOWN ) == ENCODER_DOWN) {
    mode++;
    if (mode > 2) {
      mode = 0;
    }
    return;
  }
  if ( enc == 0 ) {
    return;
  }
  if (mode == 0) {
    mode ++;
  }

  if (mode == 1) {
    print_mode_0 = 0;
    if ( (enc & ENCODER_LEFT) == ENCODER_LEFT ) {
      delta += 0.5;
    } else {
      delta -= 0.5;
    }
  }
  if (mode == 2) {
    if ( (enc & ENCODER_LEFT) == ENCODER_LEFT ) {
      if (sensor_map_id < 5) {
        sensor_map_id ++;
      }
      else {
        sensor_map_id = 0 ;
      }
    } else {
      if (sensor_map_id > 0) {
        sensor_map_id --;
      } else {
        sensor_map_id = 5;
      }
    }
  }
  return;
}

int last_button;
int timeout = 94 << (SENSOR_RESOLUTION - 9);

void loop() {

  print_mode();
  ask();

}


void ask() {

  if (sensor_map_id != settings.sensor_map_id) {
    settings.sensor_map_id = sensor_map_id;
    saved = false;
  }
  if (delta != settings.delta) {
    settings.delta = delta;
    saved = false;
    logicMillis = 0;
  }

  if (sensor_map_id > 5) {
    sensor_map_id = 0;
    delta = 0;
  }
  last_button = 0;
  check_idle();

  if (millis() - logicMillis > 2000) {
    logic();
    logicMillis = millis();
  }

  if (millis() - previousMillis > timeout * 4 + 10000);
  {
    for (int i = 0; i < 3; i++) {
      float temp_read = sensors.getTempCByIndex(i);
      if (temp_read > -99 && temp_read < 99) {
        temperatures[i] = temp_read;
      }
    }
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);
    previousMillis = millis();
  }
}

void print_mode() {

  if (mode == 0) {
    print_mode0();
  }
  if (mode == 1) {
    print_mode_0 = 0;

    print_mode1();
  }
  if (mode == 2) {
    print_mode2();
  }
}

void print_mode0() {
  if (abs(millis() - timeout0) > 2000) {
    timeout0 = millis();
    print_mode_0 ^= 1;
  }

  if (print_mode_0 == 1) {
    print_temperature();
    print_relay_status();
  } else {
    print_target_t();
  }
}

void print_mode1() {
  print_target_t();
}

void print_mode2() {
  lcd.setCursor(0, 0);
  lcd.print("In Out Outdr");
  lcd.setCursor(0, 1);           // move cursor to second line "1" and 9 spaces over
  for (int i = 0; i < 3; i++) {
    int ssid = sensors_map[sensor_map_id][i];
    float temp = temperatures[ssid];
    if (temp > 50) {
      temp = 99;
    }
    lcd.print(temp, 1);
    lcd.print(";");
  }
  lcd.print("   ");
}

void print_temperature() {
  lcd.setCursor(0, 0);           // move cursor to second line "1" and 9 spaces over
  for (int i = 0; i < 3; i++) {
    int ssid = sensors_map[sensor_map_id][i];
    float temp = temperatures[ssid];
    if (temp > 50) {
      temp = 20;
    }
    if (temp < -99) {
      temp = -99;
    }
    lcd.print(temp, 1);
    lcd.print(";");
  }
  lcd.print("   ");
}

void check_idle() {

  if (abs(millis() - last_keypress) > 5000 ) {
    mode = 0;
    if (!saved) {
      lcd.setCursor(0, 1);
      lcd.print("sav X ");
      EEPROM.put(0, settings);
      saved = true;
    }
  }
}


void logic() {

  float t_out = temperatures[1]; //not implemented yet
  float t_in = temperatures[0];
  float t_outdoor = temperatures[2]; //not implemented yet
  //Serial.println("logic");
  if (t_out < -50) {
    t_out = 20; // ignore sensor
  }

  if (t_in < -50) {
    t_in = 90; // shut down heaters if error with this sensor
  }
  if (t_outdoor < -50) {
    t_outdoor = 20; // ignore sensor
  }

  target_t = 43 - 0.8 * t_outdoor + delta;

  if (last_temp_in == t_in) {
    t_watchdog ++;
  } else {
    t_watchdog = 0;
  }

  if (t_in - hysteresis * 2 > target_t || t_watchdog > 20) {
    all_relays_off();
    must_grow = false;
    checkin(t_in);
  }


  if(pause_switch_logic_millis>millis()){ //
    return; 
  }
  
  if (t_in < target_t - hysteresis) {
    must_grow = true;
    power_up(t_in);
  }

  if (t_in > target_t + hysteresis) {
    must_grow = false;
    power_down(t_in);
  }

  if ( t_in < target_t + hysteresis && t_in > target_t - hysteresis ) {
    if (must_grow && t_in < last_temp_in) {
      power_up(t_in);
  }

    if (!must_grow && t_in > last_temp_in) {
      power_down(t_in);
    }
  }
}


void power_up(float t_in) {
  for (byte i = 0; i < 3; i++) {
    if (!relay_state[i]) {
      relay_on(i);
      break;
    }
  }
  checkin(t_in);
}

void power_down(float t_in) {
  for (byte i = 3; i > 0; i--) {
    if (relay_state[i - 1]) {
      relay_off(i - 1);
      break;
    }
  }
  checkin(t_in);
}

void checkin(float t_in){
  last_temp_in = t_in;
  pause_switch_logic_millis = millis() + 90000;  // next switch in 1.5 minute
}
void relay_off(byte relay_id) {
  relay_state[relay_id] = false;
  digitalWrite(relay_map[relay_id], LOW);
}

void relay_on(byte relay_id) {
  if (relay_state[relay_id]) return;
  relay_state[relay_id] = true;
  digitalWrite(relay_map[relay_id], HIGH);
}


void all_relays_on() {
  for (byte i = 0; i < 3; i++) {
    relay_on(i);
  }
};

void all_relays_off() {
  for (byte i = 0; i < 3; i++) {
    relay_off(i);
  }
};



void shift_mode() {
  mode++;
  if (mode > 2) {
    mode = 0;
  }
}

void print_relay_status() {

  lcd.setCursor(0, 1);
  lcd.print("Head:");
  for (byte i = 0; i < 3; i++) {
    if (relay_state[i]) {
      lcd.print("[*]");
    } else {
      lcd.print("[ ]");
    }
  }
  lcd.print("    ");
}

void print_target_t() {
  lcd.setCursor(0, 0);
  lcd.print("Target temp:    ");
  lcd.setCursor(0, 1);
  lcd.print(target_t, 1);
  lcd.print(" C               ");
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16)
      Serial.print(deviceAddress[i], HEX);
  }
}

