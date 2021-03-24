#include <ESP32Servo.h>

#define DEBUG 0

#define MAX_TURN_DEG 26
#define MAX_TURN_RAD MAX_TURN_DEG * PI / 180
#define WHEEL_BASE 342
#define HALF_WHEEL_BASE WHEEL_BASE/2
#define TRACK_WIDTH 191

// Pin Descriptions
int servo_left_pin = 23;
int servo_right_pin = 22;
int motor_enable_pin = 21;
int motor_fwd_pin = 19;
int motor_bck_pin = 18;

Servo left_steer, right_steer;

void setup() {
  #if DEBUG
  Serial.begin(9600);
  #endif
  Serial2.begin(9600);
  
  left_steer.attach(servo_left_pin);
  right_steer.attach(servo_right_pin);
  pinMode(motor_enable_pin, OUTPUT);
  pinMode(motor_fwd_pin, OUTPUT);
  pinMode(motor_bck_pin, OUTPUT);

  left_steer.writeMicroseconds(1500);
  right_steer.writeMicroseconds(1500);
  analogWrite(motor_enable_pin, 0);
}

void loop() {
  if(Serial2.available() >= 8){
    float turn, velocity, angle;
    byte drive_msg[8];
    Serial2.readBytes(drive_msg, 8);
    
    union {
      float f;
      byte b[4];
    } u;

    u.b[0] = drive_msg[3];
    u.b[1] = drive_msg[2];
    u.b[2] = drive_msg[1];
    u.b[3] = drive_msg[0];
    turn = u.f;

    u.b[0] = drive_msg[7];
    u.b[1] = drive_msg[6];
    u.b[2] = drive_msg[5];
    u.b[3] = drive_msg[4];
    velocity = u.f;

    /* STEERING */
    float theta, turn_radius, theta_in, theta_out, angle_left, angle_right;

    if(turn == 0){
      angle_left = angle_right = HALF_PI;
    }
    else{
      /* Turn Scaling */
      theta = turn * MAX_TURN_RAD;
  
      /* Ackermann Steering Geometry */
      turn_radius = WHEEL_BASE / tan(theta);
      theta_in = atan2(WHEEL_BASE, turn_radius - HALF_WHEEL_BASE);
      theta_out = atan2(WHEEL_BASE, turn_radius + HALF_WHEEL_BASE);
  
      if(turn > 0){
        angle_left = theta_in + HALF_PI;
        angle_right = theta_out + HALF_PI;
      }
      else{
        angle_left = theta_in - HALF_PI;
        angle_right = theta_out - HALF_PI;
      }
    }

    /* Steering Servo Control */
    int pwm_left = round(2000 * angle_left / PI + 500);
    int pwm_right = round(2000 * angle_left / PI + 500);
    left_steer.writeMicroseconds(pwm_left);
    right_steer.writeMicroseconds(pwm_right);

    /* MOTOR*/
    int speed = round(abs(velocity) * 255);
    analogWrite(motor_enable_pin, speed);
    
    if(velocity > 0){
      digitalWrite(motor_fwd_pin, HIGH);
      digitalWrite(motor_bck_pin, LOW);
    }
    else if(velocity < 0){
      digitalWrite(motor_fwd_pin, LOW);
      digitalWrite(motor_bck_pin, HIGH);
    }
    else {
      digitalWrite(motor_fwd_pin, LOW);
      digitalWrite(motor_bck_pin, LOW);
    }

    #if DEBUG
    printf("TURN = %f\nVELO = %f\n", turn, velocity);
    printf("ANGLE_L = %f\nANGLE_R = %f\n\n", angle_left, angle_right);
    #endif
  }
}
