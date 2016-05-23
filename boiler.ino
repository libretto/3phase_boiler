
#include <EEPROM.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>



#define btnUP     0xFF906F
#define btnDOWN   0xFFA857
#define btnLEFT   0xFF02FD
#define btnRIGHT  0xFFC23D
#define btnMODE   0xFF629D
#define btnREPEAT 0xFFFFFFFF

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
IRrecv irrecv(IR_BUS);

decode_results results;

struct Settings {
  float delta;
  int sensor_map_id;
};

Settings settings;


void setup()
{
  Serial.begin(9600);
  Serial.println("Boiler Debug Console:");

  lcd.init();                      // initialize the lcd

  for (byte i = 0; i < 3; i++) {
    pinMode(relay_map[i], OUTPUT);
  }
  power_relay_off();

  irrecv.enableIRIn(); // Start the receiver


  EEPROM.get(0, settings);
 
  if ( isnan(settings.delta) ) { // First Time check
    settings.delta = 0;
    settings.sensor_map_id = 0;
  }

  sensors.begin();

  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.print("Init:           ");

  // init settings

}
int last_button;

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    if (results.value != btnREPEAT) {
      last_button = results.value;
    }
    process_key(last_button);
    irrecv.resume(); // Receive the next value
  } else {
    last_button = 0;
  }

  check_idle();
  sensors.requestTemperatures();
  logic();
  if (mode == 0) {
    print_mode0();
  }
  lcd.setCursor(0, 0);

}

unsigned long timeout0 = 0;
byte print_mode_0 = 0;
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


byte sensors_map[6][3] = {
  {0, 1, 2},
  {0, 2, 1},
  {1, 0, 2},
  {1, 2, 0},
  {2, 0, 1},
  {2, 1, 0}
};

float hysteresis = 2;


void print_temperature() {
  lcd.setCursor(0, 0);           // move cursor to second line "1" and 9 spaces over
  for (int i = 0; i < 3; i++) {
    float temp = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][i]);
    if (temp < 50) {
      temp = 20;
    }
    lcd.print(temp, 1);     // display seconds elapsed since power-up
    lcd.print(";");
  }
  lcd.print("   ");
}


unsigned long last_keypress = 0;

void check_idle() {

  if (abs(millis() - last_keypress) > 5000) {
    mode = 0;
    EEPROM.put(0, settings);
  }

}

float target_t;
int last_boiler_action=0;
void logic() {
  float t_out = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][2]); //outcome water t
  float t_in = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][1]);  //income water t
  float t_outdoor = sensors.getTempCByIndex(sensors_map[settings.sensor_map_id][0]); // outdoor t
  if (t_out < 50) {
    t_out = 20;
  }
  if (t_in < 50) {
    t_in = 20;
  }
  if (t_outdoor < 50) {
    t_outdoor = 20;
  }

  
  Serial.println(t_outdoor);
  Serial.println(settings.delta);
  target_t = 43 - 0.8 * t_outdoor + settings.delta;
  
  if ( t_in + hysteresis < target_t ) {
    Serial.println("POWER ON");
    //power_relay_on();
    diff_power_action(target_t-t_in);
    last_boiler_action=1;
    
  } else if ( t_in - hysteresis > target_t) {
    Serial.println("POWER OFF");
    power_relay_off();
    last_boiler_action=0;
  }
  
}

void diff_power_action(float delta_t){
  if(delta_t>(hysteresis)){
    relay_on(0);
  } else {
      relay_off(0);
  }
  if(delta_t>hysteresis+1){
    relay_on(1);
  } else {
      relay_off(1);
  }
  if(delta_t>hysteresis+2){
    relay_on(2);
  } else {
    relay_off(2);
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
      settings.delta += 0.5;
      timeout0 = millis();
      print_mode_0 = 0;
      break;
    case btnDOWN:
      settings.delta -= 0.5;
      timeout0 = millis();
      print_mode_0 = 0;
      break;
    case btnMODE:
      shift_mode();
      break;
    default:
      last_button = 0;
      break;
  }

}

void shift_mode() {

  mode++;
  if (mode > 3) {
    mode ++;
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



