#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DIST_TARGET 255
#define _DIST_ALPHA 0.1
#define _DIST_MIN 70
#define _DIST_MAX 410

#define _DUTY_MIN 1330 //up
#define _DUTY_NEU 1450
#define _DUTY_MAX 1650 //down

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30 // servo speed limit (unit: degree/second)
#define INTERVAL 20 // servo update interval

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 500

// PID parameters
#define _KP 0.62;
#define _KI 0.0014;
#define _KD 16.7;

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

// PID varibles
float error_curr, error_prev, contrl, pterm, iterm, dterm;
float kp, ki, kd;

float dist_min, dist_max, dist_cali, dist_prev, alpha;
int a, b; // unit: mm

float median_q[5];
float arr[5];
int now = 0;
float median_pivot = 0.0;

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
  a = 70; //real 100
  b = 393; //real 400
  dist_target = _DIST_TARGET;
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_ema = dist_raw = dist_cali = dist_prev = 0.0;
  alpha = _DIST_ALPHA;

  for (int i = 0; i < 5; i++)
    median_q[i] = arr[i] = 0.0;

  // convert angle speed into duty change per interval
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
  duty_curr = duty_target = 0;

  // initialize variables for PID
  kp = _KP;
  ki = _KI;
  kd = _KD;
  pterm = iterm = dterm = 0.0;
  error_prev = 0.0;

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
    if (dist_prev - val > 50 && dist_prev - val < 150)
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
    dist_raw = ir_distance();
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
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

    if (-8 < error_curr && error_curr < 8)
    {
      iterm = 0;
    }
    else if (-45 < error_curr && error_curr < 45)
    {
      iterm +=  ki * error_curr;
    }
    else {
      iterm = 0;
    }
    dterm = kd * (error_curr - error_prev);

    // control smoothly pd
    //    if (pterm < 0)
    //      pterm *= -((int)pterm / 90.0);
    //    else
    //      pterm *= ((int)pterm / 70.0);
    if (dterm > 45 || dterm < -40) dterm /= (dterm / 3);
    if ((pterm < 0 && dterm < 0) || (pterm > 0 && dterm > 0)) dterm *= -1;

    contrl = pterm + dterm;
    duty_target = _DUTY_NEU + contrl;
    duty_curr = alpha * duty_target + (1 - alpha) * duty_target;

    //keep control value within the range
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;
    // adjust servo position according to the ir_distance
    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;
    // output the read value to the serial port
    //    Serial.print("ir:");
    //    Serial.print(dist_raw);
    //    Serial.print(",cal:");
    //    Serial.print(dist_cali);
    //    Serial.print(",m:");
    //    Serial.print(median_pivot);
    //    Serial.print(",ema:");
    //    Serial.print(dist_ema);
    //    Serial.print(",p:");
    //    Serial.print(pterm);
    //    Serial.print(",i:");
    //    Serial.print(iterm);
    //    Serial.print(",d:");
    //    Serial.print(dterm);
    //    Serial.print(",ser:");
    //    Serial.println(duty_target);

    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm * kp, -300, 300, 510, 610));
    Serial.print(",dterm:");
    Serial.print(map(dterm * kd, -300, 300, 510, 610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }

  if (dist_ema > 200 && dist_ema < 350) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
}
