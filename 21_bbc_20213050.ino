#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN 800 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1420 // servo neutral position (90 degree)
#define _DUTY_MAX 2180 // servo full counterclockwise position (180 degree)

#define alpha 0.1

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _SERVO_SPEED 30  // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

// global variables
unsigned long last_sampling_time; // unit: ms
float duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
float dist_ema;
Servo myservo;
float duty_target, duty_curr;
int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.writeMicroseconds(1420);
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (float)(_SERVO_SPEED / 180.0) * (float)(INTERVAL / 1000.0);

// remove next three lines after finding answers
  Serial.print("duty_chg_per_interval:");
  Serial.println(duty_chg_per_interval);

  duty_target = 0;
  duty_curr = 0;


// initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
  
  a = 82; //70;
  b = 319; //300;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);

  dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;

  if(dist_ema < 255.0){
    duty_target = _DUTY_MAX;
     //myservo.writeMicroseconds(_DUTY_MAX);
     //analogWrite(PIN_LED, 0);
  }
  else {
    duty_target = _DUTY_MIN;
     //myservo.writeMicroseconds(_DUTY_MIN);
     //analogWrite(PIN_LED, 255);
  }

  // adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }

// update servo position
  myservo.writeMicroseconds(duty_curr);
  
  last_sampling_time += INTERVAL;
}
