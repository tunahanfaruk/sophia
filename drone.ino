#include <Wire.h>
#include <ESP32Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "Adafruit_BMP3XX.h"









// ======== BMP390 ========
#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP_I2C_SDA 19
#define BMP_I2C_SCL 18
Adafruit_BMP3XX bmp;
double calib = 0.0;
float loo = 0;







// ======== BNO08x ========
TwoWire WireBNO(1);
BNO08x myIMU;
#define BNO08X_ADDR 0x4B
#define BNO08X_INT -1
#define BNO08X_RST -1






// Raw + filtered angles
float roll_raw=0, pitch_raw=0, yaw_raw=0;
float roll=0, pitch=0, yaw=0;






// Setpoints
float x_fin_angle=0, y_fin_angle=0;






// PID state
float x_integral=0, y_integral=0;
float x_previousError=0, y_previousError=0;
float kp=2.65, ki=0.145, kd=0.5; 
float dt=0.0022;







// integral limit (anti-windup)
const float INTEGRAL_LIMIT = 200.0;







// ======== Motors ========
Servo right, left, rightb, leftb;
const int pinright=5, pinleft=2, pinrightb=13, pinleftb=14;
float speed=1050, goal_speed=1350;
float acc = 8.0;
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;

unsigned long currentMillis_cal = 0;






// Loop timing (micros)
unsigned long lastLoopMicros = 0;
const unsigned long loopTimeMicros = 2200; // 5000us -> 200Hz








// LPF alpha (0..1). Küçük alpha = daha fazla smoothing.
const float LPF_ALPHA = 0.25;








// ======== Functions ========
void right_speed(int us){ right.writeMicroseconds(us); }
void left_speed(int us){ left.writeMicroseconds(us); }
void rightb_speed(int us){ rightb.writeMicroseconds(us); }
void leftb_speed(int us){ leftb.writeMicroseconds(us); }

void setReports(){
  if(myIMU.enableRotationVector()) Serial.println(F("Rotation vector enabled"));
  else Serial.println("Could not enable rotation vector");

  if(myIMU.enableAccelerometer()) Serial.println(F("Accelerometer enabled"));
  else Serial.println("Could not enable accelerometer");
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  while(!Serial) delay(10);








  // --- BNO (I2C ikinci hattı) ---
  WireBNO.begin(22,21);
  WireBNO.setClock(400000); // 400kHz ile daha az gecikme
  if(!myIMU.begin(BNO08X_ADDR, WireBNO, BNO08X_INT, BNO08X_RST)){
    Serial.println("BNO08x not found!"); while(1);
  }
  setReports();








  // --- Motors ---
  right.setPeriodHertz(50); right.attach(pinright,PWM_MIN,PWM_MAX);
  left.setPeriodHertz(50); left.attach(pinleft,PWM_MIN,PWM_MAX);
  rightb.setPeriodHertz(50); rightb.attach(pinrightb,PWM_MIN,PWM_MAX);
  leftb.setPeriodHertz(50); leftb.attach(pinleftb,PWM_MIN,PWM_MAX);






  // ESC calibration (sadeleştirilmiş) - yalnızca ilk kurulumda açıksa kullan
  right_speed(PWM_MAX); left_speed(PWM_MAX); rightb_speed(PWM_MAX); leftb_speed(PWM_MAX);
  delay(1500);
  right_speed(PWM_MIN); left_speed(PWM_MIN); rightb_speed(PWM_MIN); leftb_speed(PWM_MIN);
  delay(2000);

  currentMillis_cal = millis();
  lastLoopMicros = micros();
}

double time_started = 0.0;







// ======== Loop ========
void loop() {
  unsigned long nowMicros = micros();
  if (nowMicros - lastLoopMicros < loopTimeMicros) return; // sabit döngü hızı
  // hesapla dt güvenli
  unsigned long elapsed = nowMicros - lastLoopMicros;
  lastLoopMicros = nowMicros;
  dt = elapsed / 1000000.0;
  if (dt <= 0) dt = 0.001;

  float currentMillis = (millis() - currentMillis_cal);
  time_started = currentMillis * 0.001;








  // smooth stop after 20s (örnek)
  if (time_started > 20.0){
    goal_speed = 1000;
  }







  // --- BNO Orientation & Accel (raw) ---
  float ax=0, ay=0, az=0;
  if(myIMU.wasReset()) setReports();
  if(myIMU.getSensorEvent()){
    if(myIMU.getSensorEventID()==SENSOR_REPORTID_ROTATION_VECTOR){
      roll_raw  = myIMU.getRoll()  * 180.0/PI -1.62;
      pitch_raw = myIMU.getPitch() * 180.0/PI - 0.57;
      yaw_raw   = myIMU.getYaw()   * 180.0/PI;
    } else if(myIMU.getSensorEventID()==SENSOR_REPORTID_ACCELEROMETER){
      ax = myIMU.getAccelX();
      ay = myIMU.getAccelY();
      az = myIMU.getAccelZ();
    }
  }








  // --- Low-pass filter (simple exponential) ---
  roll  = (roll  * (1.0 - LPF_ALPHA) + roll_raw  * LPF_ALPHA);
  pitch = (pitch * (1.0 - LPF_ALPHA) + pitch_raw * LPF_ALPHA);
  yaw   = yaw   * (1.0 - LPF_ALPHA) + yaw_raw   * LPF_ALPHA;







  // --- PID Control (roll / pitch) ---
  float x_error = x_fin_angle - roll;
  x_integral += x_error * dt;
  // anti-windup
  if (x_integral > INTEGRAL_LIMIT) x_integral = INTEGRAL_LIMIT;
  if (x_integral < -INTEGRAL_LIMIT) x_integral = -INTEGRAL_LIMIT;
  float x_derivative = (x_error - x_previousError) / dt;
  float x_output = kp * x_error + ki * x_integral + kd * x_derivative;
  x_previousError = x_error;








  float y_error = y_fin_angle - pitch;
  y_integral += y_error * dt;
  if (y_integral > INTEGRAL_LIMIT) y_integral = INTEGRAL_LIMIT;
  if (y_integral < -INTEGRAL_LIMIT) y_integral = -INTEGRAL_LIMIT;
  float y_derivative = (y_error - y_previousError) / dt;
  float y_output = kp * y_error + ki * y_integral + kd * y_derivative;
  y_previousError = y_error;







  // --- smooth throttle ramp (acc) ---
  if (goal_speed > speed) speed += acc;
  if (goal_speed < speed) speed -= acc;
  // clamp speed for safety
  if (speed > PWM_MAX) speed = PWM_MAX;
  if (speed < PWM_MIN) speed = PWM_MIN;







  // --- Motor mixing (X config) ---
  int rightPWM  = (int)constrain(speed + (-x_output - y_output), PWM_MIN, PWM_MAX);
  int leftPWM   = (int)constrain(speed + (+x_output - y_output), PWM_MIN, PWM_MAX);
  int rightbPWM = (int)constrain(speed + (-x_output + y_output), PWM_MIN, PWM_MAX);
  int leftbPWM  = (int)constrain(speed + (+x_output + y_output), PWM_MIN, PWM_MAX);

  right_speed(rightPWM);
  left_speed(leftPWM);
  rightb_speed(rightbPWM);
  leftb_speed(leftbPWM);








  // Debug (isteğe bağlı)
  /*Serial.print("dt: "); Serial.print(dt, 4);
  Serial.print(" | R: "); Serial.print(roll,2);
  Serial.print(" P: "); Serial.print(pitch,2);
  Serial.print(" | outX: "); Serial.print(x_output,2);
  Serial.print(" outY: "); Serial.print(y_output,2);
  Serial.print(" | spd: "); Serial.print(speed);
  Serial.print(" goal: "); Serial.println(goal_speed);*/
}
