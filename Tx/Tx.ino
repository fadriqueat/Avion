#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Include Adafruit Graphics & OLED libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

const byte address[6] = "00001";
const uint64_t pipeOut = 0xE9E8F0F0E1LL;

RF24 radio(7, 8); // CE, CSN
int ackData[2] = {-1, -1}; // to hold the two values coming from the slave

//btn
int x;
int y;

int check = 0;
int radiostate = 0;
unsigned long laststartTime;
unsigned long startTime;
unsigned long lastAck;
long timery = 0;

bool principiante = false;

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
  void ResetData() 
{
  data.throttle = 12;
  data.pitch = 127; 
  data.roll = 127; 
  data.yaw = 127;    
  data.aux1 = 127;    
  data.aux2 = 127;    
}



void pantalla(){
  display.clearDisplay();
    //Set the color - always use white despite actual display color
  display.setTextColor(WHITE);
  //Set the font size
  display.setTextSize(1);
  //Set the cursor coordinates
  display.setCursor(0,0);
  display.print("Mode:   ");
  if (x == 0){
    display.print("Safe");
    
  }
  else{
    if (principiante == false){
      display.print("Pro");
    }
    else{
      display.print("Ayuda");
    }
    
  }
  display.setCursor(0,10);
  display.print("radioCounter:  ");
  display.print(ackData[1]);
  display.setCursor(0,20);
  display.print("connection:  ");
  if (radiostate == 0){
    display.print("Err");
  }
  else{
    display.print("Stable");
  }

  display.display();
}

void setup() {

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  pantalla();

  radio.begin();
  radio.openWritingPipe(pipeOut);
  //radio.openWritingPipe(address);
  //radio.setAutoAck(false);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();

  //btn
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  //btn derecho pin 2
  //btn izq pin 3
  
  ResetData();
  Serial.begin(9600);
}

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
  val = map(val, lower, middle, 0, 128);
  else
  val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}


void loop() {

  if (check == 0){
    laststartTime = millis();
    startTime = millis();
    lastAck = millis();
    check = 1;
  }

  //btn
  x = digitalRead(2);
  y = digitalRead(3);

  if ( radio.isAckPayloadAvailable() ) {
    
    radio.read(&ackData, sizeof(ackData));
    lastAck = millis();
  }


  if(millis() - timery > 2000){
    //Vibra mientras el boton esta apretado, mirar como solucionar
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

  if (millis() - lastAck > 1000){
    radiostate = 0;
    ackData[1] = 0;
  }
  else{
    radiostate = 1;
  }

  if (millis() - laststartTime > 300){
    pantalla();
    laststartTime = millis();
  }
  
  data.throttle = mapJoystickValues( analogRead(A0), 12, 524, 1020, true ); 
  data.roll = mapJoystickValues( analogRead(A3), 12, 524, 1020, true );
  data.pitch = mapJoystickValues( analogRead(A2), 12, 524, 1020, false );
  data.yaw = mapJoystickValues( analogRead(A1), 12, 524, 1020, false );
  data.aux1 = mapJoystickValues( analogRead(A6), 12, 524, 1020, true );
  data.aux2 = mapJoystickValues( analogRead(A7), 12, 524, 1020, true );
  data.y = y;
  Serial.println(data.pitch);
  radio.write(&data, sizeof(Signal));

}
