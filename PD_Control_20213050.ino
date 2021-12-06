#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

//Unstable state    : 0.66 62
//Underdamped       : 0.66  4.85
//Overdamped        : 0.67 35.05
//Critically damped : 0.62 14.05

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 11
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.07 // EMA filter is disabled 0.05

// Servo range
#define _DUTY_MIN 1170
#define _DUTY_NEU 1667
#define _DUTY_MAX 2115 

// Servo speed control
#define _SERVO_ANGLE 20 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 150 // servo speed limit (deg/sec)

// Event periods
#define _INTERVAL_DIST 9 // distance sensor interval (ms)
#define _INTERVAL_SERVO 3  // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)

// PID parameters
#define _KP 0.62 // proportional gain ****
#define _KD 14.05    // derivative gain 63 1420


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
float duty_chg_per_interval; // maximum duty difference per interval
float duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

// Fiter variables
float x[10] = {69, 79, 106, 131, 170, 201, 217, 228, 256, 271};
float y[10] = {87, 100, 125, 150, 200, 250, 300, 350, 400, 425};
float nd = sizeof(x)/sizeof(float);

float elapsedTime, previousTime;

void setup() {
  // initialize GPIO pins for LED and attach servo
  myservo.attach(PIN_SERVO); 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
  // initialize global variables
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = true;
  dist_target = _DIST_TARGET;
  dist_ema = 0;
  duty_curr = _DUTY_NEU;
  error_prev = 0;
  previousTime = 0;
  elapsedTime = 0;
  
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  
  // initialize serial port
  Serial.begin(57600);
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (float)(_SERVO_SPEED / 180.0) * (float)(_INTERVAL_SERVO / 1000.0);
}

void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  unsigned long time_curr = millis();
  if(time_curr >=   + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  
  ////////////////////
  // Event handlers //
  ////////////////////
  
  if(event_dist) {
    event_dist = false;
    
    // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();
    dist_ema = _DIST_ALPHA * dist_raw + (1-_DIST_ALPHA) * dist_ema;
    if(dist_ema<_DIST_MIN) {
      dist_ema = _DIST_MIN;
    }
    else if(dist_ema>_DIST_MAX) {
      dist_ema = _DIST_MAX;
    }
    
    // PID control logic
    elapsedTime = time_curr - previousTime;
    error_curr = _DIST_TARGET - dist_ema;
    pterm = _KP * error_curr;
    dterm = (error_curr - error_prev)/elapsedTime * _KD * 100;
    control = pterm + dterm;
    
    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

    // keep duty_target value within the range of 
    // [_DUTY_MIN, _DUTY_MAX] 
    if(duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;
    }
    else if(duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX;
    }
    
    // update erorr_prev
    error_prev = error_curr;
    previousTime = time_curr;
  }
  
  if(event_servo) {
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
    myservo.writeMicroseconds((int)duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
  
float ir_distance_filtered(void){ // return value unit: mm
  // for now, just use ir_distance() without noise filter.
  float val;
  float volt = float(analogRead(PIN_IR));
  float b = 0;
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  
  if(val > x[9]) val = x[9];   
    else if(val < x[0]) val = x[0];
  
  for(int i=0;i<nd;i++) {
      float p=1;
      for(int j=0;j<nd;j++) {
          if(i!=j) {
            p = p*(val-x[j])/(x[i]-x[j]);
          }
      }
      b += p * y[i];
  }
  return b;
}
