

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// ================================================================
// ===                ROS                                       ===
// ================================================================
#include <ros.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/Config.h>
#include "WiFi.h"
#include <std_msgs/Empty.h>

float KP, KD, KI; 

const char* ssid     = "W8LAN";
const char* password = "tremco22";
// Set the rosserial socket server IP address
IPAddress server(192,168,0,108);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::Float32 float_msg;
ros::Publisher chatter("control", &float_msg);

void pid_update( const dynamic_reconfigure::Config& pid){
  KP = pid.doubles[0].value * pid.doubles[1].value;
  KI = pid.doubles[2].value * pid.doubles[3].value;
  KD = pid.doubles[4].value * pid.doubles[5].value; 
  digitalWrite(22, HIGH - digitalRead(22));
}

ros::Subscriber<dynamic_reconfigure::Config> sub("/left_wheel_pid/parameter_updates", pid_update );

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 13  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 22 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                     MOTORS CONTROL                       ===
// ================================================================
const int enA = 32;
#define inA1    17
#define inA2    16
const int enB  =   33;
#define inB1    25
#define inB2    26

// setting PWM properties
const int freq = 5000;
const int pwm1_Channel = 0;
const int pwm2_Channel = 0;
const int resolution = 8;

void forward(int vel){
    ledcWrite(pwm1_Channel, vel);  
    ledcWrite(pwm2_Channel, vel);  
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
}

void backward(int vel){
    ledcWrite(pwm1_Channel, vel);  
    ledcWrite(pwm2_Channel, vel);  
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH);
}

void turn_left(int vel){
    ledcWrite(pwm1_Channel, vel);  
    ledcWrite(pwm2_Channel, vel);   
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW); 
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH); 
}

void turn_right(int vel){
    ledcWrite(pwm1_Channel, vel);  
    ledcWrite(pwm2_Channel, vel);  
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
}
 

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);

  
  ledcSetup(pwm1_Channel, freq, resolution);
  ledcSetup(pwm2_Channel, freq, resolution);
  
  ledcAttachPin(enA, pwm1_Channel);
  ledcAttachPin(enB, pwm2_Channel);   

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(23,19);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    // Use ESP8266 serial to monitor the process
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(chatter);
  nh.subscribe(sub);

  
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    //calibration(300);
}


// ================================================================
// ===                    CALIBRATION                           ===
// ================================================================
float yaw_cal = 0.0; 
float roll_cal = 0.0; 
float pitch_cal = 0.0; 
void calibration(int steps){
    int cnt = 0; 
    for (int cnt = 0; cnt < steps; cnt++){
      if (!dmpReady) return;
  
      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
          if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop 
            fifoCount = mpu.getFIFOCount();
          }  
          // other program behavior stuff here
          // .
          // .
          // .
          // if you are really paranoid you can frequently test in between other
          // stuff to see if mpuInterrupt is true, and if so, "break;" from the
          // while() loop to immediately process the MPU data
          // .
          // .
          // .
      }
  
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          fifoCount = mpu.getFIFOCount();
          Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
          
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);                   
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          yaw_cal += ypr[0];
          pitch_cal += ypr[1];
          roll_cal += ypr[2];
      }
    }
    yaw_cal /= steps; 
    pitch_cal /= steps; 
    roll_cal /= steps; 
    Serial.println(yaw_cal);
    Serial.println(pitch_cal);
    Serial.println(roll_cal);
    digitalWrite(LED_PIN, HIGH);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================ 
float yaw = 0; 
float pitch = 0; 
float roll = 0; 

unsigned long int T_sample = 0;
unsigned long int start_time = 0; 
float error = 0; 
float error_der = 0; 
float error_old = 0; 
float error_int = 0;

float control; 
float base = 180; 

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = (ypr[0]- yaw_cal) * 180/M_PI;
        pitch = (ypr[1] - pitch_cal) * 180/M_PI;
        pitch -= 3.2; 
        roll = (ypr[2]- roll_cal) * 180/M_PI;

        T_sample = (millis() - start_time); 
        start_time = millis(); 
        error = pitch; 
        error_der = (error - error_old)/T_sample; 
        error_int += error * T_sample; 
        error_old = error; 

        control = base + abs(error * 2 *  KP + error_der * KD + error_int * KI); 
      
        if (control > 255)
          control = 255; 

        if(error > 0)
          forward(control); 
        else
          backward(control);
          
        float_msg.data = control; 
        if (nh.connected()) {
          chatter.publish(&float_msg);
        }
        nh.spinOnce();
    }
}
