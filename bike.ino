//    July 11, 2021

/*
  Pins
  hall        D2
  sw uart tx  D3
  sw uart rx  D4
  voltage     A6
  current     A7
  RTC SCL     A5
  RTC SDA     A4
*/

#include <stdio.h>
#include <stdlib.h>
#include <SoftwareSerial.h>
#include "RTClib.h"
#include <EEPROM.h>

#define R1 430 // voltage divider, 430k ohms
#define R2 47 // voltage divider, 47k ohms
#define GEAR_REDUCTION 9/33
#define M_PER_REV 25 * 2.54 / 100 * 3.14159 // diameter,to cm, to m, pi

// Prototypes
float get_voltage(void);
float get_current(void);
float fuel_guage(float);

// Global variables
volatile long int motor_revolutions = 0;
volatile long int previous_motor_revolutions = 0;  // to determine if the bike is moving
volatile int motor_speed;           // RPM
volatile float charge_used = 0;           // A_Hr
volatile int led_state;
volatile long int previous_time = 0;
unsigned long int previous_loop_time = 0;
volatile long int dt = 0;
unsigned long int loop_time = 0;

float discharge_curve[22] = {10.4, 0, 12.4, 9, 12.6, 14, 12.8, 17, 12.9, 20, 13.00, 30, 13.20, 40, 13.30, 70, 13.40, 90, 14.00, 100};

int bike_speed;                     // km/hr
float distance_traveled = 0;          // km
float distance_remaining;             // km
float charge_capacity = 10;           // A_Hr
float charge_remaining;               // A_Hr
float energy_used = 0;                // W_Hr
float energy_remaining;               // W_Hr
float soc_v;                          // %   state of charge (voltage)
float soc_c;                          // %   state of charge (counting coulombs)
float time_remaining;                 // hours
float efficiency;                     // W_HR/km
int i;
char string[30];

char buff[30];
float pi = 3.1425625;

SoftwareSerial mySerial(4, 3); // RX, TX

RTC_DS3231 rtc;

//-------------------------------------------------------------

void setup() {

  // configure devices
  pinMode(2, INPUT); // hall effect sensor
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, INPUT);
  pinMode(6, INPUT_PULLUP);

  mySerial.begin(9600);
  //  LCD_CLEAR

  //  lcd_clear();

  Serial.begin(9600);
  // Serial.print("serial ");

  attachInterrupt(digitalPinToInterrupt(2), hall_sensor_isr, RISING);

  // Determine remaining battery life based on voltage
//  charge_remaining = charge_capacity * fuel_guage(get_voltage() / 3) / 100;
//  charge_used = charge_capacity - charge_remaining;

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

}

void loop() {

  float voltage = 0;           // A
  float current = 0;           // A
  float power = 0;             // A

  // LOOP TIME
  loop_time = millis() - previous_loop_time;
  previous_loop_time = millis();

  // BASIC ELECTRICAL
  voltage = get_voltage();                            // V
  current = get_current();                            // A
  power = voltage * current;                          // W

  // STATE OF CHARGE
  charge_used += current * loop_time / 1000 / 3600;   // amps * hours
  charge_remaining = charge_capacity - charge_used;   // A_Hr
  soc_c = charge_remaining * 100 / charge_capacity ;  // %
  soc_v = fuel_guage(voltage / 3);

  // OTHER
  //acceleration = speed * loop_time;

  DateTime now = rtc.now();

  time_remaining = charge_remaining / current;        // Hr
  efficiency = power / bike_speed;                    //  W_HR/km
  distance_remaining = bike_speed * time_remaining;   // km
  energy_remaining = voltage * charge_remaining;      // W_Hr
  energy_used = voltage * charge_used;                // W_Hr

  // DISPLAY
  mySerial.write(0x03); // lcd clear;

  // Floating point hacks
  int power_int = (int)power;
  int bike_speed_int = (int)bike_speed;
  int distance_traveled_int = (int)distance_traveled;
  int distance_traveled_low = (distance_traveled - (float)distance_traveled_int) * 10;
  int voltage_high = (int)voltage / 3;
  int voltage_low = (int)(((voltage / 3) - voltage_high) * 100) ;
  int soc_v_int = (int)soc_v;
  int soc_c_int = (int)soc_c;

  sprintf(string, "%2u    ", bike_speed_int);       send_string(string);

  dtostrf(distance_traveled, 4, 1, buff); mySerial.print(buff);

  int hour12 = now.hour(); if (hour12 > 12) hour12 -= 12;
  sprintf(string, "  %02u:%02u ", hour12, now.minute()); send_string(string); 
  sprintf(string, "%4dW", power_int);     send_string(string);

  mySerial.write(0x0a); // 2nd line
  delay(2);

  mySerial.print(voltage/3, 2);
  sprintf(string, "  %3u%% %3u%%   ", soc_v_int, soc_c_int);     send_string(string);

  dtostrf(charge_used, 4, 1, buff); mySerial.print(buff);
 // sprintf(string, " %2u", (int)charge_remaining);     send_string(string);

  Serial.println(motor_speed);
  
   Serial.println(motor_revolutions); 
  Serial.println(dt);
  Serial.println(" ");
  delay(500);
}


//-------------------------------------------------------------------------

float get_voltage(void) {

  float voltage;

  voltage = (float)analogRead(A6)  * 5 / 1023;
  return voltage / R2 * (R1 + R2);
}

//---------------------------------------------------------

// ACS750 current sensor, -50A to +50A
float get_current(void) {

#define OFFSET 2.5 // 2.5V indicates zero amps
#define AMPS_PER_VOLT 20

  float _voltage; // voltage from the sensor

  _voltage = (float)analogRead(A7) * 5 / 1023;
  return (_voltage - OFFSET) * AMPS_PER_VOLT;
}

//-----------------------------------------------------------------------

void move_cursor(char horiz, char vert)
{
  unsigned char aa;

  aa = horiz + 0x7F;
  if (vert != 1) {
    aa = aa + 0x40;
  }
  //  mySerial.write(0x03);
  //  mySerial.write(aa);
  //display(0x03);      // ^c makes the next byte an LCD command
  //display(aa);         // cursor position code

}

//-----------------------------------------------------------------------

float fuel_guage(float battery_voltage) {

  int i;

  for (i = 0; i < 22; i += 2) {
    if (battery_voltage <= discharge_curve[i] ) break;
  }
  return discharge_curve[i + 1];
}

//------------------------------------------------------

void send_string(char* string) {

  int n;

  for (n = 0; n < 25; n++) {
    if (string[n] == 0) return;
    mySerial.write(string[n]);
    delay(2);
  }
}

//=======================================================

void hall_sensor_isr(void) {

  previous_motor_revolutions = motor_revolutions;
  motor_revolutions++; // for odometer
  
  dt = millis() - previous_time;
  previous_time = millis();

  motor_speed = (float)(1000 * 60 / (dt)); // RPM
  if (motor_revolutions - previous_motor_revolutions < 2) bike_speed = 0;
  else bike_speed = (int)motor_speed * GEAR_REDUCTION * M_PER_REV * 60 / 1000; // km/hr

  distance_traveled = motor_revolutions * GEAR_REDUCTION * M_PER_REV / 1000; //  km

  if (led_state != 0) {
    led_state = 0;
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
  }
  else {
    led_state = 1;
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }

}
