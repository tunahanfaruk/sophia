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
TwoWire WireBNO(1); // ESP32 ikinci I2C hattı
BNO08x myIMU;
#define BNO08X_ADDR 0x4B
#define BNO08X_INT -1
#define BNO08X_RST -1








float roll=0, pitch=0, yaw=0;
float x_fin_angle=0, y_fin_angle=0;
float x_integral=0, y_integral=0;
float x_previousError=0, y_previousError=0;
float kp=2.0, ki=0.0, kd=0.0;
float dt=0.001;
float previousMillis=0;








// ======== Motors ========
Servo right, left, rightb, leftb;
const int pinright=5, pinleft=2, pinrightb=13, pinleftb=14;
float speed=1050, goal_speed=1400, smth=1;




float currentMillis_cal = millis();




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








// ======== Setup ========
void setup() {

  delay(10000);
  Serial.begin(115200);
  while(!Serial) delay(10);








/*
  // --- BMP ---
  Wire.begin(BMP_I2C_SDA, BMP_I2C_SCL);
  if(!bmp.begin_I2C()){ Serial.println("BMP not found!"); while(1); }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);









  // BMP Calibration

  // waiting for sensor to get heat
  for(int i=0; i<1000; i++){
    bmp.performReading();
    delay(20);
  }

  float sum=0;
  for(int i=0;i<1000;i++){
    if(bmp.performReading()) sum += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }









  calib = sum / 1000.0;
  Serial.print("BMP Calibration done: "); Serial.println(calib);

*/







  // --- BNO ---
  WireBNO.begin(22,21); 
  if(!myIMU.begin(BNO08X_ADDR, WireBNO, BNO08X_INT, BNO08X_RST)){
    Serial.println("BNO08x not found!"); while(1);
  }
  setReports();










  // --- Motors ---
  right.setPeriodHertz(50); right.attach(pinright,1000,2000);
  left.setPeriodHertz(50); left.attach(pinleft,1000,2000);
  rightb.setPeriodHertz(50); rightb.attach(pinrightb,1000,2000);
  leftb.setPeriodHertz(50); leftb.attach(pinleftb,1000,2000);






  // ESC calibration
  // also it depends on ESC
  right_speed(2000); left_speed(2000); rightb_speed(2000); leftb_speed(2000);
  delay(2000);
  
  right_speed(1000); left_speed(1000); rightb_speed(1000); leftb_speed(1000);
  delay(4000);

  currentMillis_cal = millis();
}

double time_started = 0.0;

float v1 = 0.0;
float v2 = 0.0;
float v3 = 0.0;
float v4 = 0.0;
float v5 = 0.0;
float rl_alt = 0.0;

// ======== Loop ========
void loop() {
  delay(1);

  // --- Timing ---
  float currentMillis = millis() - currentMillis_cal;

  time_started = currentMillis * 0.001;

  dt = (currentMillis - previousMillis) * 0.001;
  previousMillis = currentMillis;



//harsh stopping
/*
  if (time_started > 15.0){

    while (true) {
      right_speed(1000);
      left_speed(1000);
      rightb_speed(1000);
      leftb_speed(1000);
    }


    


  }
*/


//smooth stopping



  if (time_started > 15.0){

    goal_speed = 1000;

  }




/*

  // --- BMP Altitude ---
  float altitude=0.0, temperature=0.0, pressure=0.0;



  if(bmp.performReading()){

    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - calib;
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();


    v5 = v4;
    v4 = v3;
    v3 = v2;
    v2 = v1;
    v1 = altitude;

    rl_alt = (v1 + v2 + v3 + v4 + v5) / 5; 



    

  }

*/







  // --- BNO Orientation & Accel ---
  float ax=0, ay=0, az=0;
  if(myIMU.wasReset()) setReports();
  if(myIMU.getSensorEvent()){
    if(myIMU.getSensorEventID()==SENSOR_REPORTID_ROTATION_VECTOR){
      roll = myIMU.getRoll()*180.0/PI;
      pitch = myIMU.getPitch()*180.0/PI;
      yaw = myIMU.getYaw()*180.0/PI;
    } else if(myIMU.getSensorEventID()==SENSOR_REPORTID_ACCELEROMETER){
      ax = myIMU.getAccelX();
      ay = myIMU.getAccelY();
      az = myIMU.getAccelZ();
    }
  }









  // --- PID Control ---
  float x_error = x_fin_angle - roll;
  x_integral += x_error * dt;
  float x_derivative = (x_error - x_previousError)/dt;
  float x_output = kp*x_error + ki*x_integral + kd*x_derivative;
  x_previousError = x_error;





  float y_error = y_fin_angle - pitch;
  y_integral += y_error*dt;
  float y_derivative = (y_error - y_previousError)/dt;
  float y_output = kp*y_error + ki*y_integral + kd*y_derivative;
  y_previousError = y_error;





  // --- smooth ---
  if(goal_speed > speed) speed += smth;
  if(goal_speed < speed) speed -= smth;

  // --- Motor Output ---
  int rightPWM = (int)(speed+(-x_output-y_output)/2);
  int leftPWM  = (int)(speed+(+x_output-y_output)/2);
  int rightbPWM= (int)(speed+(-x_output+y_output)/2);
  int leftbPWM = (int)(speed+(+x_output+y_output)/2);

  right_speed(rightPWM);
  left_speed(leftPWM);
  rightb_speed(rightbPWM);
  leftb_speed(leftbPWM);









  
  /*Serial.println("Alt: "); Serial.print(altitude); Serial.print(" m | Avg: "); Serial.print(rl_alt);
  Serial.print(" | Temp: "); Serial.print(temperature); Serial.print(" C | P: "); Serial.print(pressure); Serial.println(" Pa");

  Serial.print("Euler: Roll: "); Serial.print(roll); Serial.print(" Pitch: "); Serial.print(pitch); Serial.print(" Yaw: "); Serial.println(yaw);
  Serial.print("Accel: X: "); Serial.print(ax); Serial.print(" Y: "); Serial.print(ay); Serial.print(" Z: "); Serial.println(az);

  Serial.print("Motors PWM -> R: "); Serial.print(rightPWM);
  Serial.print(" L: "); Serial.print(leftPWM);
  Serial.print(" Rb: "); Serial.print(rightbPWM);
  Serial.print(" Lb: "); Serial.println(leftbPWM);
  Serial.println(time_started);
  */


  

  Serial.println(dt,3);

  //Serial.println("--------------------------------------------------");
}
