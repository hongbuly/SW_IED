#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DIST_TARGET 255
#define _DIST_ALPHA 0.2
#define _DIST_MIN 70
#define _DIST_MAX 410

#define _DUTY_MIN 1240 //up
#define _DUTY_NEU 1380
#define _DUTY_MAX 1510 //down

#define _SERVO_SPEED 1000 // servo speed limit (unit: degree/second)
#define _RAMPUP_TIME 360 //servo speed rampup (0 to max) time (unit: ms)
#define INTERVAL 20 // servo update interval

// Event periods
#define _INTERVAL_DIST 10
#define _INTERVAL_SERVO 10
#define _INTERVAL_SERIAL 400

#define _ITERM_MAX 20;

// PID parameters
#define _KP 0.281;
#define _KI 0.000672;
#define _KD 32.1;

Servo myservo;

// Distance sensor
float dist_target; //location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed contrl
int duty_chg_per_interval; //maximum duty difference per interval
int duty_target, duty_curr;
int duty_chg_max, duty_chg_adjust;

// PID varibles
float error_curr, error_prev, contrl, pterm, iterm, dterm;
float kp, ki, kd;
int iterm_max;

float dist_min, dist_max, dist_cali, dist_prev, alpha;
int a, b; // unit: mm

float median_q[5];
float arr[5];
int now = 0;
float median_pivot;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);

  // move servo to neutral position
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);

  // initialize global varibles
  a = 80; //real 100
  b = 370; //real 400
  dist_raw = 0;
  dist_target = _DIST_TARGET;
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_ema = dist_raw = dist_cali = dist_prev = median_pivot = 0.0;
  alpha = _DIST_ALPHA;

  for (int i = 0; i < 5; i++)
    median_q[i] = arr[i] = 0.0;

  // convert angle speed into duty change per interval
  duty_chg_max = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
  duty_chg_adjust = duty_chg_max * INTERVAL / _RAMPUP_TIME;
  duty_chg_per_interval = 0;
  duty_curr = duty_target = _DUTY_NEU;

  // initialize variables for PID
  kp = _KP;
  ki = _KI;
  kd = _KD;
  pterm = iterm = dterm = 0.0;
  error_prev = 0.0;
  iterm_max = _ITERM_MAX;

  // initialize event variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  if (val < dist_min || val > dist_max) val = 0.0;
  if (dist_prev != 0)
    if (dist_prev - val > 40 && dist_prev - val < 180)
      val = 0.0;

  if (val == 0.0) val = dist_prev;
  else dist_prev = val;

  return val;
}

void swap(float arr[], int a, int b) {
  float temp = arr[a];
  arr[a] = arr[b];
  arr[b] = temp;
}

void partition(float *median_q, int start, int ends) {
  for (int i = 0; i < 5; i++) {
    arr[i] = median_q[i];
  }
  int pivot = start;
  int i = pivot + 1;
  int j = ends;

  while (i <= j)
  {
    while (i <= ends && arr[i] <= arr[pivot]) ++i;
    while (j > start && arr[j] >= arr[pivot]) --j;

    if (i >=  j) {
      break;
    }

    swap(arr, i, j);
  }
  swap(arr, j, pivot);
  median_pivot = arr[2];
}

void loop() {
  // wait until next sampling time.
  // millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    event_dist = false;

    // get a distance reading from ir_distance
    dist_raw = 0;
    for (int i = 0; i < 100; i++) {
      dist_raw += ir_distance();
    }
    dist_raw /= 100;

    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);

    // median filter
    if (now > 4) {
      now--;
      for (int i = 1; i < 5; i++) {
        median_q[i - 1] = median_q[i];
      }
    }
    median_q[now++] = dist_cali;
    partition(median_q, 0, 4);
    dist_ema = alpha * median_pivot + (1 - alpha) * dist_ema;

    // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = kp * error_curr;
    iterm +=  ki * error_curr;
    dterm = kd * (error_curr - error_prev);

    // control iterm max
    if (iterm > iterm_max) {
      iterm = _ITERM_MAX;
    }
    else if (iterm < -iterm_max) {
      iterm = -_ITERM_MAX;
    }
    if (dist_ema > 220 && dist_ema < 265) {
      iterm /= 5;
    }

    // control smoothly d
    if ((pterm < 0 && dterm < 0) || (pterm > 0 && dterm > 0)) dterm *= -1;
    if (dist_ema > 240 && dist_ema < 270) dterm /= 10;
    if (dterm < -70 || dterm > 70) dterm /= 6;

    contrl = pterm + iterm + dterm;
    duty_target = _DUTY_NEU + contrl;

    //keep control value within the range
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;
    // adjust servo position according to the ir_distance
    if (duty_target > duty_curr) {
      if (duty_chg_per_interval < duty_chg_max) {
        duty_chg_per_interval += duty_chg_adjust;
        if (duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
      }
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else if (duty_target < duty_curr) {
      if (duty_chg_per_interval > -duty_chg_max) {
        duty_chg_per_interval -= duty_chg_adjust;
        if (duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
      }
      duty_curr += duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }
    else {
      duty_chg_per_interval = 0;
    }

    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;
    // output the read value to the serial port
    //        Serial.print("R:");
    //        Serial.print(dist_raw);
    //        Serial.print(",E:");
    //        Serial.print(dist_ema);
    //        Serial.print(",T:");
    //        Serial.print(duty_target);
    //        Serial.print(",P:");
    //        Serial.print(pterm);
    //        Serial.print(",D:");
    //        Serial.print(dterm);
    //        Serial.print(",I:");
    //        Serial.println(iterm);

    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",D:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",I:");
    Serial.print(map(iterm, -1000, 1000, 510, 610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",-G:245,+G:265,m=0,M:800");
  }

  if (dist_ema > 245 && dist_ema < 265) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
}
