#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DIST_ALPHA 0.1
#define _DIST_MIN 50
#define _DIST_MAX 450

#define _DUTY_MIN 1625 //up
#define _DUTY_NEU 1736
#define _DUTY_MAX 1903 //down

float dist_min, dist_max, raw_dist, dist_cali, dist_prev, dist_ema, alpha;
int a, b; // unit: mm
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  
// initialize serial port
  Serial.begin(57600);

  a = 73; //real 100
  b = 337; //real 400
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_ema = raw_dist = dist_cali = dist_prev = 0.0;
  alpha = _DIST_ALPHA;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  if (val < dist_min || val > dist_max) val = 0.0;

  if (val == 0.0) val = dist_prev;
  else dist_prev = val;
  
  return val;
}

void loop() {
  raw_dist = ir_distance();
  dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  dist_ema = alpha * dist_cali + (1 - alpha) * dist_ema;
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.println(myservo.read());

  if (dist_ema < 255.0) {
    myservo.writeMicroseconds(_DUTY_MAX);
  } else {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(50);
}
