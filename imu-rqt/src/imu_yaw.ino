#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>


HardwareSerial Serial3(PB11, PB10);

MPU6050 mpu;

ros::NodeHandle nh;

std_msgs::Float32 yaw_msg;
ros::Publisher pub_yaw("Yaw", &yaw_msg);

#define INTERRUPT_PIN PB14  // use pin 2 on Arduino Uno & most boards
#define LED_PIN PC13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
uint8_t fifoBuffer[256]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



void setup() {
  // put your setup code here, to run once:

  pinMode(LED_PIN, OUTPUT);  
  pinMode(INTERRUPT_PIN, INPUT);
  
  Wire.begin();
  Wire.setClock(400000);
  Serial3.begin(115200);
  //Serial.begin(9600);      
  (nh.getHardware())->setPort(&Serial3);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_yaw);

  mpu.initialize();
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  //Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
         
      mpu.setDMPEnabled(true);  // turn on the DMP, now that it's ready

      // enable Arduino interrupt detection
//      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//      Serial.println(F(")..."));
///      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
   

  
 
  // configure LED for output  
}

long long pub_timer = 0;

void loop() {
  if (millis() - pub_timer > 5){
    getMPUdata();
    yaw_msg.data = ypr[0] * 180/M_PI;
    pub_yaw.publish(&yaw_msg);
    pub_timer = millis();
  }
      nh.spinOnce();
 }

void getMPUdata(){
    
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      
        //display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//        Serial.print("ypr\t");
//        Serial.print(ypr[0] * 180/M_PI);
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
  
}
