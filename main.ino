#include "Arduino.h"
#include "mcu_api.h"
#include "protocol.h"
#include "wifi.h"
#include "Servo.h"


#define led_pin1 PB14
#define led_pin2 PB15
#define buzzer_pin PA7
#define sensor1_pin PA5 
#define sensor2_pin PA6 
#define servo1_pin PB0 
#define servo2_pin PB1 
#define btn1_pin PB12
#define btn2_pin PB13 

#define BUZZER_INTERVAL 15
#define SERVO_INTERVAL 500
#define UPDATE_TUYA_INTERVAL 1000

uint32_t last_time1 = 0;
uint32_t last_time2 = 0;
uint32_t last_time3 = 0; 
uint8_t status_1 = 0; //led status 
uint8_t status_2 = 0; //led status 
uint8_t status_3 = 1; //senro1 status 
uint8_t status_4 = 1; //senro2 status 
uint8_t status_5 = 1; //btn1 status 
uint8_t status_6 = 1; //btn2 status 
uint8_t status_7 = 0; //tuya light status
Servo Servo1;
Servo Servo2;
uint8_t pos0 = 0 ;
uint8_t pos1 = 0 ;

typedef void(*cb)() ;
typedef void(*cb1)(Servo*) ;
typedef void(*cb2)(uint8_t*) ;

void led_blink(int pin, int interval, uint32_t *t, uint8_t *st)
{ 
  if(millis() - *t > interval) 
  {
    *t = millis();
    *st = 1 - *st;
  }
  digitalWrite(pin ,*st ? HIGH : LOW);
}

void pulse_detect(int pin, uint8_t *st, cb callback )
{
  
  uint8_t pin_value = digitalRead(pin);
  if ( (pin_value == 1) &&  (*st == 0) ) {
    (*callback)();   
  }
  *st = pin_value;
}

void btn_detect(int pin, uint8_t *st, cb1 callback , Servo * sv)
{
  
  uint8_t pin_value = digitalRead(pin);
  if ( (pin_value == 1) &&  (*st == 0) ) {
    (*callback)(sv);   
  }
  *st = pin_value;
}

void btn_detect(int pin, uint8_t *st, cb2 callback , uint8_t *st1)
{
  
  uint8_t pin_value = digitalRead(pin);
  if ( (pin_value == 1) &&  (*st == 0) ) {
    (*callback)(st1);   
  }
  *st = pin_value;
}

void buzzer_echo()
{
  digitalWrite(buzzer_pin,HIGH);
  delay(BUZZER_INTERVAL);
  digitalWrite(buzzer_pin,LOW);
}

void servo_action(Servo * sv) 
{
    sv->write(180);
    delay(SERVO_INTERVAL);
    sv->write(0);
}

void servo_action(Servo * sv1, Servo * sv2) 
{
    sv1->write(180);
    sv2->write(180);
    delay(SERVO_INTERVAL);
    sv1->write(0);
    sv2->write(0);
}

void toggle_value(uint8_t * st)
{
  *st = 1 - *st;
}

void led_show(uint8_t led_pin, uint8_t  st)
{
  digitalWrite(led_pin, st ? LOW : HIGH);
}

//tuya
void update_tuya_dp(uint32_t *t, uint8_t led_status)
{
  uint32_t bat_value = 100;
  uint8_t supply1 = digitalRead(sensor1_pin);
  uint8_t supply2 = digitalRead(sensor2_pin);  
  uint8_t all_supply = (supply1*100 + supply2*100) / 2 ;
  if(millis() - *t > UPDATE_TUYA_INTERVAL)
  {
    *t = millis();
    mcu_dp_value_update(DPID_BATTERY_PERCENTAGE,bat_value);
    mcu_dp_value_update(DPID_SURPLUS_GRAIN,all_supply);
    mcu_dp_bool_update(DPID_LIGHT,led_status ? 1 : 0); 
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);

  wifi_protocol_init();

  pinMode(led_pin1,OUTPUT);
  pinMode(led_pin2,OUTPUT);
  pinMode(buzzer_pin,OUTPUT);
  pinMode(sensor1_pin,INPUT);
  pinMode(sensor2_pin,INPUT);
  Servo1.attach(servo1_pin);
  Servo2.attach(servo2_pin);
  Servo1.write(0);
  Servo2.write(0);
  
}

void loop()
{

  wifi_uart_service();

  if (Serial2.available())
  {
    unsigned char ch = (unsigned char)Serial2.read();
    uart_receive_input(ch);
  }

  led_blink(led_pin1,500,&last_time1,&status_1);
  pulse_detect(sensor1_pin, &status_3, buzzer_echo);
  pulse_detect(sensor2_pin, &status_4, buzzer_echo);
  btn_detect(btn1_pin,&status_5, servo_action, &Servo1);
  btn_detect(btn2_pin,&status_6, servo_action, &Servo2);
  btn_detect(btn2_pin,&status_6, toggle_value, &status_7);
  update_tuya_dp(&last_time3,status_7);
  led_show(led_pin2,status_7);

  // // 涂鸦模块直连模式, 调试时使用
  // if (Serial2.available())
  // {
  //   unsigned char ch = (unsigned char)Serial2.read();
  //   Serial.write(ch);
  // }
  // if (Serial.available())
  // {
  //   unsigned char ch = (unsigned char)Serial.read();
  //   Serial2.write(ch);
  // }
}

