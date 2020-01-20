// WARNING!!!
// This code was created for study and tests purposes for my personal use only.
// Any reproducing of the unit and this code usage with high voltage devices is at your own risk!

#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>

/*
   DS1307RTC required for day-night feature of heating algorithm
   and it will not require ATMega328P board because of firmware memory consumption)


  #include <TimeLib.h>
  #include <Time.h>
  #include <DS1307RTC.h> */

#define ONE_WIRE_BUS 2

#define TARGET_MODE 0
#define STATUS_MODE 1
#define LOG_MODE 2
#define CONFIGURE_SENSORMAP_MODE 3
#define MAX_MODE 3

#define SENSOR_RESOLUTION 10

int relay_map[3] = {11, 10, 12}; // relay pins
boolean relay_state[3];
int t_watchdog = 0;
unsigned long last_keypress = 0;
unsigned long mode_timeout = 0;
unsigned long logic_delay = 0;



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
boolean pause = false;


byte mode = TARGET_MODE;
boolean draw_flag = false;
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

  //sensors.getAddress(sensorDeviceAddress, 0);
  //printAddress(sensorDeviceAddress);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);

  //sensors.getAddress(sensorDeviceAddress, 1);
  //printAddress(sensorDeviceAddress);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);
  //sensors.getAddress(sensorDeviceAddress, 2);
  //printAddress(sensorDeviceAddress);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  PCICR =  0b00000010;
  PCMSK1 = 0b00001110;
  wdt_disable();
  delay(3000);
  wdt_enable(WDTO_2S);

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
volatile byte encoder_data;
volatile boolean encoder_event = false;
#define ENCODER_LEFT 0x01
#define ENCODER_RIGHT 0x02
#define ENCODER_DOWN 0x04

int last_button;
int timeout = 94 << (SENSOR_RESOLUTION - 9);

// Encoder events by interrupt

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
    encoder_event = true;
  }

  if (seqA == 0b00000011 && seqB == 0b00001001)
  {
    enc |= ENCODER_LEFT;
    encoder_event = true;
  }

  if (!C_val) {
    enc |= ENCODER_DOWN;
    encoder_event = true;
  }
  encoder_data = enc;

}


void loop() {
  if (encoder_event) {
    input(encoder_data);
  }

  print_mode();
  pass();
  wdt_reset();
}

inline void  input(byte enc) {
  //Serial.println(enc);

  saved = true;
  encoder_event = false;
  last_keypress = millis();
  mode_timeout = last_keypress;
  draw_flag = true;
  if ((enc & ENCODER_DOWN ) == ENCODER_DOWN) {
    mode++;
    if (mode > MAX_MODE) {
      mode = 0;
    }
    return;
  }
  if ( enc == 0 ) {
    return;
  }
  if (mode == STATUS_MODE || mode == LOG_MODE) {
    mode = TARGET_MODE;
  }

  if (mode == TARGET_MODE) {
    if ( (enc & ENCODER_LEFT) == ENCODER_LEFT ) {
      delta += 0.5;
    } else {
      delta -= 0.5;
    }
  }

  if (mode == CONFIGURE_SENSORMAP_MODE) {
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
}



void pass() {
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

  if (millis() - previousMillis > timeout * 8)
  {
    for (int i = 0; i < 3; i++) {
      float temp_read = sensors.getTempCByIndex(i);
      if (temp_read > -99 && temp_read < 99) {
        temperatures[i] = temp_read;
      }
    }
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    //sensors.setWaitForConversion(true);
    previousMillis = millis();
  }

  if (millis() - logicMillis > 2000) {
    logic();
    logicMillis = millis();
  }
}

void print_mode() {
  if (draw_flag) {
    if (mode == STATUS_MODE ) {
      print_status();
    }
    if (mode == TARGET_MODE) {
      print_target_t();
    }
    if (mode == CONFIGURE_SENSORMAP_MODE) {
      print_sensormap();
    }
    if (mode == LOG_MODE) {
      print_log();
    }
    draw_flag = false;
  }
  if (abs(millis() - mode_timeout) > 2000) {
    mode_timeout = millis();
    mode ++;
    draw_flag = true;
    if (mode > 2) {
      mode = 0;
    }
  }
}

void print_status() {
  print_temperature();
  print_relay_status();
}



void print_sensormap() {
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

void print_log() {
  // want see uptime...
  lcd.setCursor(0, 0);

  lcd.print("Up:");
  lcd.print((millis() / 3600000));
  lcd.print("h         ");
  lcd.setCursor(0, 1);
  lcd.print(last_temp_in);
  lcd.print(":");
  lcd.print(t_watchdog);
  lcd.print("    ");
}

void print_temperature() {
  lcd.setCursor(0, 0);           // move cursor to second line "1" and 9 spaces over
  for (int i = 0; i < 3; i++) {
    int ssid = sensors_map[sensor_map_id][i];
    float temp = temperatures[ssid];
    if (temp > 99) {
      temp = 99;
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

  if (t_in - hysteresis * 2 > target_t || t_watchdog > 1000) {
    all_relays_off();
    must_grow = false;
    checkin(t_in);
  }

  if (t_in < target_t - hysteresis) {
    must_grow = true;
    power_up(t_in);
  }

  if ( millis() - pause_switch_logic_millis < logic_delay ) { //
    return;
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

void checkin(float t_in) {
  last_temp_in = t_in;

  // empirical heating timeouts TODO
  if (must_grow) {
    pause_switch_logic_millis = millis();
    logic_delay = 180000;
  } else {
    pause_switch_logic_millis = millis();
    logic_delay = 180000;
  }
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

