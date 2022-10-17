// Pins
#define W_ENCODER_A D12
#define W_ENCODER_B D8
#define Y_ENCODER_A D10
#define Y_ENCODER_B D6
#define F_MOTOR_A A3
#define R_MOTOR_A A6
#define S_MOTOR_A D3
#define F_MOTOR_B A2 
#define R_MOTOR_B D4
#define S_MOTOR_B D2
#define MOTOR_LOGIC A7
#define SCL A5
#define SDA A4

#include "IgorNav.h"
#include "IgorComm.h"
#include "IgorControl.h"
#include "IgorActuate.h"
#include <Arduino.h>


//timers
long time1Hz = 0;
long time5Hz = 0;
static unsigned long prevMillis = 0;

void setupDisplay()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.setTextColor(WHITE);             
  display.setCursor(0,0);   
  display.setTextSize(1);  
  // Clear the buffer
  display.clearDisplay();
}

void setupNav()
{
  
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Positive magnetic declination - MA
  imu.setDeclination(-14.10);
  imu.setFusionAlgorithm(SensorFusion::MAHONY);
  imu.setKp( nKp );
  imu.setKi( nKi );

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 

    //  Paste your calibration bias offset HERE
    imu.loadAccBias(-0.002136, -0.013855, -0.021240);
	imu.loadGyroBias(2.527161, -0.874786, -0.127106);
	imu.loadMagBias(0.110474, 0.153687, -0.231201);

    //  This sketch assumes that the LSM9DS1 is already calibrated, 
    //  If so, start processing IMU data. If not, run the testAndCalibrate 
    //  sketch first.
    imu.start();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void setupControl()
{
    // PID objects
    control->initControl();

}

void setup()
{
  setupDisplay();
  setupNav();
  setupControl();
}

void loop()
{
  // 100Hz Control
  if ( millis() - prevMillis >= 10 )
  {
    dt = millis() - prevMillis;
    
    // State update
    wheelLeft.update(dt);
    wheelRight.update(dt);
    EulerAngles ypr = imu.update();
    SensorData angVel = imu.rawData();
    state.leftWheelSpeed = wheelLeft.getSpeed();
    state.rightWheelSpeed = wheelRight.getSpeed(); 
    state.speed =  ( speed[0] + speed[1] ) / 2;

    state.ypr[0] = ypr.yaw;
    state.ypr[1] = ypr.pitch;
    state.ypr[2] = ypr.roll;
    state.angVel[0] = angVel.gx;
    state.angVel[0] = angVel.gy;
    state.angVel[0] = angVel.gz;

    leftOutput = control->doLeftControl(state);
    rightOutput = control->doRightControl(state);
  }

  if ( !motorInit )
  {
    Countdown(4);
    motorInit = true;
  }

  if ( state.ypr[1] < 215 && state.ypr[1] > 145 )
  {
    Serial.print(leftOutput);
    Serial.print(", ");
    Serial.println(rightOutput);
    motorController.move(leftOutput, rightOutput, MIN_ABS_SPEED);
  }
  else
  {
    motorController.stopMoving();
  }
}
