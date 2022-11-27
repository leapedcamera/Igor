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
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  display.setTextColor(WHITE);             
  display.setCursor(0,0);   
  display.setTextSize(2);  
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
    printLn("LSM9DS1 IMU Connected."); 

    //  Paste your calibration bias offset HERE
    imu.loadAccBias(0.034363, 0.021545, -0.005859);
    imu.loadGyroBias(-0.097198, 0.261688, 0.732727);
    imu.loadMagBias(-0.041992, 0.037720, -0.619751);

    //  This sketch assumes that the LSM9DS1 is already calibrated, 
    //  If so, start processing IMU data. If not, run the testAndCalibrate 
    //  sketch first.
    imu.start();
  } else {
    printLn("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void setupControl()
{
  // PID objects
  control->initControl();
  digitalWrite(MOTOR_LOGIC, HIGH);

}

void setup()
{
  // Set pins
  pinMode(F_MOTOR_A, OUTPUT);
  pinMode(F_MOTOR_B, OUTPUT);
  pinMode(R_MOTOR_A, OUTPUT);
  pinMode(R_MOTOR_B, OUTPUT);
  pinMode(S_MOTOR_A, OUTPUT);
  pinMode(S_MOTOR_B, OUTPUT);
  pinMode(MOTOR_LOGIC, OUTPUT);
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);
  pinMode(Y_ENCODER_A, INPUT);
  pinMode(Y_ENCODER_B, INPUT);
  pinMode(W_ENCODER_A, INPUT);
  pinMode(W_ENCODER_B, INPUT);

  //  Start Serial and wait for connection
  Serial.begin(115200);
  delay(2000);
  setupDisplay();
  setupNav();
  setupControl();
}

void loop()
{
  // High rate nav
  angles = imu.update();

  // 10Hz Control
  dt = millis() - prevMillis;
  if ( dt >= 100 )
  {
    prevMillis = millis();
    // State update
    wheelLeft.update(dt);
    wheelRight.update(dt);
    rates = imu.rawData();
    state.leftWheelSpeed = wheelLeft.getSpeed();
    state.rightWheelSpeed = wheelRight.getSpeed(); 
    state.speed =  ( speed[0] + speed[1] ) / 2;
    state.ypr[0] = angles.yaw;
    state.ypr[1] = angles.pitch;
    state.ypr[2] = angles.roll;
    state.angVel[0] = rates.gx;
    state.angVel[0] = rates.gy;
    state.angVel[0] = rates.gz;
    leftOutput = control->doLeftControl(state);
    rightOutput = control->doRightControl(state);
    control->refresh();
    printStatus(leftOutput, rightOutput, angles.roll, prevMillis);
  }
  if ( !motorInit )
  {
    Countdown(4);
    motorInit = true;
  }


  if ( state.ypr[2] > -130 && state.ypr[2] < -70 )
  {
    motorController.move(leftOutput, rightOutput, MIN_ABS_SPEED);
  }
  else
  {
    motorController.stopMoving();
  }
}
