#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 11
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.3 // EMA filter is disabled

// Servo range
#define _DUTY_MIN 800
#define _DUTY_NEU 1645
#define _DUTY_MAX 2100 

// Servo speed control
#define _SERVO_ANGLE 20 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 100 // servo speed limit (deg/sec)

// Event periods
#define _INTERVAL_DIST 20// distance sensor interval (ms)
#define _INTERVAL_SERVO 1 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)

// PID parameters
#define _KP 0.66 // proportional gain ****

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
float x[10] = {70, 83, 110, 132, 175, 210, 226, 246, 285, 330};
float y[10] = {87, 100, 125, 150, 200, 250, 300, 350, 400, 425};
float nd = sizeof(x)/sizeof(float);

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
  duty_curr = 0;
  
  
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
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
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
    //dist_raw = ir_distance();
    //float dist_cali = 100 + 300.0 / (256 - 82) * (dist_raw - 82);
    dist_raw = ir_distance_filtered();
    
    dist_ema = _DIST_ALPHA * dist_raw + (1-_DIST_ALPHA) * dist_ema;
    if(dist_ema<_DIST_MIN) {
      dist_ema = _DIST_MIN;
    }
    else if(dist_ema>_DIST_MAX) {
      dist_ema = _DIST_MAX;
    }
    
    // PID control logic
    error_curr = _DIST_TARGET - dist_ema;
    pterm = error_curr;
    control = _KP * pterm; 
    
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
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
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
  //return ir_distance();
  float val;
  float volt = float(analogRead(PIN_IR));
  float b = 0;
  if(val > x[9]){
    val = x[9];   
  }
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
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
