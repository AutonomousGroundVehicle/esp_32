#include <ESP32Encoder.h>
#include <ESP32Servo.h>

int motor_enable_pin = 21;
int motor_fwd_pin = 19;
int motor_bck_pin = 18;

int encoder_a_pin = 26;
int encoder_b_pin = 25;

ESP32Encoder encoder;

unsigned long last_read;
unsigned long last_count;

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  pinMode(motor_enable_pin, OUTPUT);
  pinMode(motor_fwd_pin, OUTPUT);
  pinMode(motor_bck_pin, OUTPUT);
  pinMode(encoder_a_pin, INPUT);
  pinMode(encoder_b_pin, INPUT);

  encoder.attachFullQuad(encoder_a_pin, encoder_b_pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(motor_enable_pin, 255);
  digitalWrite(motor_fwd_pin, LOW);
  digitalWrite(motor_bck_pin, HIGH);

  long interval;
  if((interval = millis() - last_read) >= 25){
    long delta = encoder.getCount() - last_count;
    long velo = delta * 1000 / interval / 64;
    printf("VELO = %d\n", velo);
    last_count = encoder.getCount();
    last_read = millis();
  }
}
