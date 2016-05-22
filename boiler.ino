
#include <EEPROM.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <IRLib.h>


#define MY_PROTOCOL NEC

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define IR_BUS 2
#define ONE_WIRE_BUS 4


#define INFO_MODE 0;
#define SET_TEMPERATURE_MODE 1
#define SET_RELAY_MODE 2

int relay_map[3] = {10, 11, 12}; // relay pins
boolean relay_state[3];

int mode = INFO_MODE;
int lcd_key     = 0;
int adc_key_in  = 0;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 20, 4);
IRrecv My_Receiver(IR_BUS);
IRdecode My_Decoder;


struct Settings {
  float delta;
  int sensor_map_id;
};

Settings settings;


void setup()
{
   lcd.init();                      // initialize the lcd

  for (byte i = 0; i < 3; i++) {
    pinMode(relay_map[i], OUTPUT);
  }
  power_relay_off();

  My_Receiver.No_Output();

  My_Receiver.enableIRIn();

  EEPROM.get(0, settings);

  Serial.begin(9600);
  Serial.println("Boiler Debug Console:");

  sensors.begin();

  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.print("Init:           ");

  // init settings

}

void loop() {


  if (My_Receiver.GetResults(&My_Decoder)) {
    My_Decoder.decode();
    Serial.println(My_Decoder.decode_type, HEX);
    Serial.println(My_Decoder.value, HEX);
    if (My_Decoder.decode_type == MY_PROTOCOL) {
      process_key(My_Decoder.value);

    }
    My_Receiver.resume();
  } else {

    check_idle();
    sensors.requestTemperatures();
    if (mode == 0) {
      print_temperature();
      logic();
      print_relay_status();
    }
    lcd.setCursor(0, 0);
  }

}


byte sensors_map[6][3] = {
  {0, 1, 2},
  {0, 2, 1},
  {1, 0, 2},
  {1, 2, 0},
  {2, 0, 1},
  {2, 1, 0}
};

void print_temperature() {
  lcd.setCursor(0, 0);           // move cursor to second line "1" and 9 spaces over
  for (int i = 0; i < 3; i++) {
    lcd.print(sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][i]), 1);     // display seconds elapsed since power-up
    lcd.print(";");
  }
  lcd.print("   ");
}


unsigned long last_keypress = 0;

void check_idle() {

  if (abs(millis() - last_keypress) > 5000) {
    mode = 0;
  }

}

void logic() {
  float t_out = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][2]); //outcome water t
  float t_in = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][1]);  //income water t
  float t_outdoor = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][0]); // outdoor t
  float target_t;
  float hysteresis = 1;
  target_t = 43 - 0.8 * t_outdoor;
  if ( target_t + hysteresis < t_in ) {
    power_relay_on();
  } else if (target_t - hysteresis > t_in) {
    power_relay_off();
  }
}



void relay_off(byte relay_id) {
  relay_state[relay_id] = false;
  digitalWrite(relay_map[relay_id], HIGH);
}

void relay_on(byte relay_id) {
  relay_state[relay_id] = true;
  digitalWrite(relay_map[relay_id], LOW);
}


void power_relay_on() {
  for (byte i = 0; i < 3; i++) {
    relay_on(i);
  }
};

void power_relay_off() {
  for (byte i = 0; i < 3; i++) {
    relay_off(i);
  }
};


void process_key(int lcd_key) {

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

void print_relay_status() {
  lcd.setCursor(0, 1);
  lcd.print("HEAD:");
  for (byte i = 0; i < 3; i++) {
    
    if (relay_state[i]) {
      lcd.print("[*]");
    } else {
      lcd.print("[ ]");
    }
    
  }
  lcd.print("    ");
}


