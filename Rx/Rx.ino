//CAMBIOS EN de V3 a V4
//Actual - V5 - 
//Vuelta  a codigo de ejemplo de IMU (MPU6050_DMP6) DMP6.12

// Include Adafruit Graphics & OLED libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4

//IMU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


//Motores
#include <SPI.h>
#include <Servo.h>
//#include <AFMotor.h>
//AF_DCMotor motor(3);

//Radio
#include <nRF24L01.h>
#include <RF24.h>


//motores/servos
Servo ch1;
Servo ch2;
//Servo ch3;
Servo ch4;
//Servo ch5;
Servo ch6;

Servo ESC;


// ================================================================
// ===                           Radio                          ===
// ================================================================
int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;

int ch_width_2_last = 0;
int ch_width_1_last = 0;
int ch_width_2_avg = 0;
int ch_width_1_avg = 0;

int x = 1;
int y = 1;

struct Signal {
byte throttle;      
byte pitch;
byte roll;
byte yaw;
byte aux1;
byte aux2;
byte x;
byte y;
};

Signal data;

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
const uint64_t pipeIn = 0xE9E8F0F0E1LL;

int ackData[2] = {109, -4000}; // the two values to be sent to the master

int radiostate = 0;

// ================================================================
// ===                            IMU                           ===
// ================================================================
  MPU6050 mpu;
  //#define OUTPUT_READABLE_EULER
  #define OUTPUT_READABLE_YAWPITCHROLL
  
  #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
  #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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
  VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  
  // packet structure for InvenSense teapot demo
  uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

  
  //interrupt detection routine
  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {
    mpuInterrupt = true;
  }



long timery = 0;
long timer = 0;
int yaw = 0;
int pitch = 0;

//Extra
int count = 0;
int start = 0;
int check = 0;
bool principiante = false;
unsigned long laststartTime;
unsigned long startTime;


void ResetData()
{
// Define the inicial value of each data input.
// The middle position for Potenciometers. (254/2=127)
data.roll = 127;
data.pitch = 127;
data.throttle = 12; // Motor Stop
data.yaw = 127;
data.aux1 = 127;
data.aux2 = 127;
data.x = 1;
data.y = 1;
}

void setup() {
  
  //Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
  ch1.attach(4);
  ch2.attach(3);
  //ch3.attach(A0);
  ch6.attach(6);
  //ch5.attach(6);
  //ch6.attach(7);
  

  // ================================================================
  // ===                           Radio                          ===
  // ================================================================
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, pipeIn);
  //radio.setAutoAck(false);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Starting radio...");


  // ================================================================
  // ===                            ESC                           ===
  // ================================================================
  Serial.println("ESC calibrating...");
  ESC.attach(5,1000,2000);
  delay(1000);
  Serial.println("Done!");

  ResetData();

  // ================================================================
  // ===                            IMU                           ===
  // ================================================================
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
  
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
    // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
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
  
}


unsigned long lastRecvTime = 0;

void recvData() {
  while ( radio.available() ) {
    
    radio.read(&data, sizeof(Signal));
    radiostate = 1;
    count++;
    lastRecvTime = millis();   // receive the data
    ESC.write(ch_width_3);
}
}

void loop() {


  //Iniciar contadores
  if (check == 0) {
    laststartTime = millis();
    startTime = millis();
    check = 1;
  }


  // ================================================================
  // ===                           Radio                          ===
  // ================================================================
  recvData();
  //unsigned long now = millis();

  //Si pasan mas de 2 segundos sin recivir datos, señal perdida
  if (millis() - lastRecvTime > 2000 ) {
    ResetData(); // Signal lost.. Reset data
    //Serial.println("Signal lost...");
    radiostate = 0;
  }

  //Mandar datos al mando cada segundo
  if (millis() - laststartTime > 1000) {
    ackData[1] = count;
    count = 0;
    radio.writeAckPayload(1, &ackData, sizeof(ackData)); // load the payload for the next time
    laststartTime = millis();
  }

  // ================================================================
  // ===                            IMU                           ===
  // ================================================================
  if(millis() - timer > 50){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pitch = ypr[1] * 180 / M_PI;
    yaw = ypr[2] * 180 / M_PI;
    //Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180 / M_PI);
    //Serial.print("\t");
    Serial.print(pitch);
    //Serial.print("\t");
    //Serial.print(yaw);
    //Serial.println();

#endif
    
    // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    
      
      timer = millis();
  }


  // ================================================================
  // ===                  Convertir y escribir                    ===
  // ================================================================
  //convertir los datos a señal PWM
  ch_width_4 = map(data.yaw,      0, 255, 1000, 2000);     // pin D5 (PWM signal)
  ch_width_2 = map(data.pitch,    0, 255, 0, 180);     // pin D3 (PWM signal)
  ch_width_3 = map(data.throttle, 100, 255, 90, 170);     // pin D4 (PWM signal)
  ch_width_1 = map(data.roll,     0, 255, 0, 180);     // pin D2 (PWM signal)
  ch_width_5 = map(data.aux1,     0, 255, 1000, 2000);     // pin D6 (PWM signal)
  ch_width_6 = map(data.aux2,     0, 255, 1000, 2000);     // pin D7 (PWM signal)
  x = data.x;
  y = data.y;
  //Hacer una media para reducir saltos
  ch_width_2_avg = (ch_width_2_last + ch_width_2)/2;
  ch_width_1_avg = (ch_width_1_last + ch_width_1)/2;  
  ch_width_2_last = ch_width_2;
  ch_width_1_last = ch_width_1;

  if(millis() - timery > 2000){
    //No esta sincronizado con el mando, por lo que seria mejor hacer esto solo en el mando y pasar el valor de principiante a traves del boton
    if (y==0){
      if (principiante == false){
        principiante = true;
      }
      else{
        principiante = false;
      }
    }
    timery = millis();
  }
  
  if (x == 1){
    if (principiante==false){
      //180 - angulo es para revertir el movimiento del servo
      //EL IMU hay que dejarlo quieto unos 10 segundos
      ch1.write(180-(ch_width_1_avg-37)); //ala derecha (sube el negativo baja el flap) LIMITE entre 50 y 150
      ch2.write(180-(ch_width_1_avg-25)); //ala izquierda (baja el negativo sube el flap) LIMITE enrte 50 y 150
      ch6.write(180-(ch_width_2_avg+18));
      //Serial.println(180-(ch_width_2_avg+18));
      ESC.write(ch_width_3);
    }
    else{
      ch1.write(180-(ch_width_1_avg-37+(yaw))); 
      ch2.write(180-(ch_width_1_avg-25+(yaw)));
      ch_width_2 = 180-(ch_width_2_avg+30+(pitch*2));
      ch6.write((ch_width_2<60) ? 60:ch_width_2);
      ESC.write(ch_width_3);
    }
  }
  else{
    ch1.write(180-(ch_width_1_avg-37+(yaw))); 
    ch2.write(180-(ch_width_1_avg-25+(yaw)));
    ch_width_2 = 180-(ch_width_2_avg+30+(pitch*2));
    ch6.write((ch_width_2<60) ? 60:ch_width_2);
    ESC.write(ch_width_3);
  }
  
  }

  //Empieza a partir de 100 aprox
  //max en 166
  

}
