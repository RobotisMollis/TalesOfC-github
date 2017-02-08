//---------------------------------------------------------
//---------------------------------------------------------
//---ARDUINO CONTROL PROGRAM FOR SOFT CEPHALOPOD ROBOT-----
//-----------------------ver 3.1---------------------------
//---------------------------------------------------------
//Implemented here: LED ring, servo control, serial communication 

#include <Servo.h>
#define servoPin1 10
#define servoPin2 11
#define servoPin3 12
#define servoPin4 13
#define servoOffset1 0
#define servoOffset2 0
#define servoOffset3 0
#define servoOffset4 0
Servo myservo1; // create servo objects
Servo myservo2;
Servo myservo3;´
Servo myservo4;

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN 3 //LED pin
#define NUMPIXELS 16 // Number of NeoPixels in the ring
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Soft Robotics Control board:
int prescaler = 256; // set this to match whatever prescaler value you set in CS registers
// intialize values for the PWM duty cycle
float potDC1 = 0;
float potDC2 = 0;
float potDC3 = 0;
float potDC4 = 0;
float potPWMfq = 50; //frequency for opening and closing valves with PWM signal

int relay = 4; //pin for relay to start and stop pump
int pos = 0;    // variable to store the servo position
int incomingByte = 0;  //variable to store incoming serial data
int actionVar = 0; //pariable to store next action of robot
long randNumber; //pariable to store randomly generated value



void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  myservo1.attach(servoPin1);  // attaches the servos on the pins
  myservo2.attach(servoPin2);
  myservo3.attach(servoPin3);
  myservo4.attach(servoPin4);
  pixels.begin(); // This initializes the NeoPixel library.
  randomSeed(analogRead(0));//shuffle the random function.

  pinMode(relay, OUTPUT);   //Relay pin for switching pump on/off
  digitalWrite(relay, LOW);//Switch off pump

/*
  // input pins for valve switches
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);
*/

  // output pins for valve PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  int eightOnes = 255;  // this is 11111111 in binary
  TCCR3A &= ~eightOnes;   // this operation (AND plus NOT), set the eight bits in TCCR registers to 0 
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;
  // set waveform generation to frequency and phase correct, non-inverting PWM output
  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);
}


void setLight(int R,int G,int B) {
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(R,G,B)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
}

void servoSweeps(int numSweeps, int backDelayTime, int forwardDelayTime, int maxDeltaAngle) 
{
for (int a = 1; a <= numSweeps; a += 1) { // 
    
  for (pos = 90-maxDeltaAngle; pos <= 90+maxDeltaAngle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos+servoOffset2);              // tell servo to go to position in variable 'pos'
    myservo3.write(pos+servoOffset3);              // tell servo to go to position in variable 'pos'
    myservo4.write(pos+servoOffset4);              // tell servo to go to position in variable 'pos'
    delay(forwardDelayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90+maxDeltaAngle; pos >= 90-maxDeltaAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos+servoOffset2);              // tell servo to go to position in variable 'pos'
    myservo3.write(pos+servoOffset3);              // tell servo to go to position in variable 'pos'
    myservo4.write(pos+servoOffset4);              // tell servo to go to position in variable 'pos'
    delay(backDelayTime);                       // waits 15ms for the servo to reach the position
  }
  }

}


void servoSweeps1(int numSweeps, int backDelayTime, int forwardDelayTime, int maxDeltaAngle) 
{
for (int a = 1; a <= numSweeps; a += 1) { // 
    
  for (pos = 90-maxDeltaAngle; pos <= 90+maxDeltaAngle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(forwardDelayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90+maxDeltaAngle; pos >= 90-maxDeltaAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(backDelayTime);                       // waits 15ms for the servo to reach the position
  }
  }

}

void servoSweeps2(int numSweeps, int backDelayTime, int forwardDelayTime, int maxDeltaAngle) 
{
for (int a = 1; a <= numSweeps; a += 1) { // 
    
  for (pos = 90-maxDeltaAngle; pos <= 90+maxDeltaAngle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(forwardDelayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90+maxDeltaAngle; pos >= 90-maxDeltaAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo2.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(backDelayTime);                       // waits 15ms for the servo to reach the position
  }
  }

}

void servoSweeps3(int numSweeps, int backDelayTime, int forwardDelayTime, int maxDeltaAngle) 
{
for (int a = 1; a <= numSweeps; a += 1) { // 
    
  for (pos = 90-maxDeltaAngle; pos <= 90+maxDeltaAngle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo3.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(forwardDelayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90+maxDeltaAngle; pos >= 90-maxDeltaAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo3.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(backDelayTime);                       // waits 15ms for the servo to reach the position
  }
  }

}

void servoSweeps4(int numSweeps, int backDelayTime, int forwardDelayTime, int maxDeltaAngle) 
{
for (int a = 1; a <= numSweeps; a += 1) { // 
    
  for (pos = 90-maxDeltaAngle; pos <= 90+maxDeltaAngle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo4.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(forwardDelayTime);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90+maxDeltaAngle; pos >= 90-maxDeltaAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo4.write(pos+servoOffset1);              // tell servo to go to position in variable 'pos'
    delay(backDelayTime);                       // waits 15ms for the servo to reach the position
  }
  }

}
void servoHome(){
    myservo1.write(90+servoOffset1);              // tell servo to go to position in variable 'pos'
    myservo2.write(90+servoOffset2);              // tell servo to go to position in variable 'pos'
    myservo3.write(90+servoOffset3);              // tell servo to go to position in variable 'pos'
    myservo4.write(90+servoOffset4);              // tell servo to go to position in variable 'pos'
}


int waitForAndReadSerial(){
  return 0;;
}

void inflateRobot(int amount1, int amount2,int amount3){
    pPWM(potPWMfq,potDC1,potDC2,potDC3,potDC4);
}

void actionA(){ //CEPHALOPOD slow SWEEPS - 2pcs
//*Servos: Sweeps
//*Light: -
//*Inflation: -

setLight(20,20,20);
inflateRobot(0,0,0);
  
//servoSweeps(10,2,2,30); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);
//servoSweeps(10,2,8,30);


servoSweeps(1,20,20,10);
//servoSweeps(2,2,12,40); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);

}

void action0(){ //CEPHALOPOD random SWEEPS - 1-3 pcs
//*Servos: Sweeps
//*Light: -
//*Inflation: -

setLight(20,20,20);
inflateRobot(0,0,0);
  
//servoSweeps(10,2,2,30); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);
//servoSweeps(10,2,8,30);

int randy1 = random(12, 31);
int randy2 = random(12, 31);
int randy3 = random(10, 26);
int randy4 = random(1, 4);

servoSweeps(randy4,randy1,randy2,randy3);
//servoSweeps(2,2,12,40); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);

}

void action1(){ //CEPHALOPOD - Short fast/slow SWEEPS - 2pcs
//*Servos: Sweeps
//*Light: -
//*Inflation: -

setLight(20,20,20);
inflateRobot(0,0,0);
  
//servoSweeps(10,2,2,30); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);
//servoSweeps(10,2,8,30);
servoSweeps(1,8,18,15);
//delay(2000); 
}

void action2(){ //CEPHALOPOD SWEEPS a la orig.
//*Servos: Sweeps
//*Light: -
//*Inflation: -

setLight(20,20,20);
inflateRobot(0,0,0);

int randy1 = random(6, 10);
int randy4 = random(1, 4);
servoSweeps(randy4,randy1,randy1,30);

randy1 = random(6, 10);
int randy2 = random(2, 4);
int randy3 = random(6, 13);
randy4 = random(2, 7);
servoSweeps(randy4,randy2,randy3,30);

//servoSweeps(2,2,12,40); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);

}

void action3(){ //INFLATE ALL CHAMBERS (FOR 300 ms. two times)
//*Servos: -
//*Light: -
//*Inflation: Increasing

//servoHome();
setLight(22,22,24);

digitalWrite(relay, HIGH);//Switch on pump
pPWM(potPWMfq,100,100,100,0);
delay(450);
digitalWrite(relay, LOW);//Switch off pump
delay(150);

digitalWrite(relay, HIGH);//Switch on pump
pPWM(potPWMfq,100,100,100,0);
delay(300);

pPWM(potPWMfq,0,0,0,0);
digitalWrite(relay, LOW);//Switch off pump

//inflateRobot(0,0,0);
setLight(20,20,20);

}

void action4(){ //INFLATE ALL CHAMBERS (FOR 3000 ms.) + LIGHTSWEEPS
//*Servos: -
//*Light: -
//*Inflation: Increasing

setLight(25,20,20);

digitalWrite(relay, HIGH);//Switch on pump
pPWM(potPWMfq,100,100,100,0);
delay(3000);
digitalWrite(relay, LOW);//Switch off pump
pPWM(potPWMfq,0,0,0,0);

//inflateRobot(0,0,0);
setLight(10,10,10);

 //LIGHTSWEEPS
//*Servos: -
//*Light: Sweeps
//*Inflation: -
servoHome();

  int a;
  int amax = 100;
  servoSweeps(1,2,4,40); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);
  //servoSweeps(1,2,1,40); //servoSweeps(numSweeps=10, backDelayTime=2, forwardDelayTime=2, maxDeltaAngle=30);
  for (a = 0; a <= amax; a=a+12) { // goes from 0 degrees to 180 degrees
    setLight(a,a,a);
    delay(300);                       // waits 15ms for the servo to reach the position
  }

}


void action5(){ //INFLATE ONE CHAMBER (FOR random ms.)
//*Servos: -
//*Light: -
//*Inflation: Increasing

//servoHome();
setLight(22,22,24);

randNumber = random(1000, 3501);
int randy1 = random(0, 2);

digitalWrite(relay, HIGH);//Switch on pump
switch (randy1) {
    case 0:
      {
      pPWM(potPWMfq,100,0,0,0);
      }
      break;
    case 1:
      {
      pPWM(potPWMfq,0, 100,0,0);
      }
      break;
    case 2:
      pPWM(potPWMfq,0,0,100,0);
      break;
  }

delay(randNumber);

digitalWrite(relay, LOW);//Switch off pump
pPWM(potPWMfq,0,0,0,0);

setLight(20,20,20);

}


void action6(){ //CEPHALOPOD SWEEPS - ONE arm

int randy1 = random(12, 31);
int randy2 = random(12, 31);
int randy3 = random(10, 26);
int randy4 = random(1, 4);


int randyChoose = random(0, 3);
switch (randyChoose) {
    case 0:
      {
      servoSweeps1(randy4,randy1,randy2,randy3);
      }
      break;
    case 1:
      {
      servoSweeps2(randy4,randy1,randy2,randy3);
      }
      break;
    case 2:
      {
      servoSweeps3(randy4,randy1,randy2,randy3);
      }
      break;
    case 3:
      {
      servoSweeps4(randy4,randy1,randy2,randy3);
      }
      break;
  }

}


void action12(){ //LIGHTSWEEPS
//*Servos: -
//*Light: Sweeps
//*Inflation: -

//inflateRobot(0,0,0);
//servoHome();

  int a;
  int amax = 120;
  for (a = 1; a <= amax; a += 1) { // goes from 0 degrees to 180 degrees
    setLight(a,a,a);
    delay(50);                       // waits 15ms for the servo to reach the position
  }
  for (a = amax; a > 0; a -= 1) { // goes from 0 degrees to 180 degrees
    setLight(a,a,a);
    delay(50);                       // waits 15ms for the servo to reach the position
  }
}

void pPWM(float pwmfreq, float pwmDC1, float pwmDC2, float pwmDC3, float pwmDC4) {
  // set PWM frequency by adjusting ICR (top of triangle waveform)
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  ICR4 = F_CPU / (prescaler * pwmfreq * 2); 
  // set duty cycles
  OCR3A = (ICR4) * (pwmDC1 * 0.01);
  OCR4A = (ICR4) * (pwmDC2 * 0.01);
  OCR4B = (ICR4) * (pwmDC3 * 0.01);
  OCR4C = (ICR4) * (pwmDC4 * 0.01);
}


void moveNow(){

//random number from 0 to 3
//randNumber = random(4);
//random number from 10 to 19
//randNumber = random(10, 20);
  
//actionVar = waitForAndReadSerial(); 
actionVar++; //TESTING!
if (actionVar==5)actionVar=0;

switch (actionVar) {
    case 0:
      {
      action3();
      }
      break;
    case 1:
      {
      action6();
      }
      break;
    case 2:
      {
      action4();
      }
      break;
    case 3:
      {
      action5();
      }
      break;
    case 4:
      {
      action2();
      }
      break;
  }
}

  
void loop() {

  potDC1 = 0; potDC2 = 0; potDC3 = 0; potDC4 = 0;
  pPWM(potPWMfq,potDC1,potDC2,potDC3,potDC4);
    
 if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                incomingByte =  incomingByte -48; //Convert so that numbers are read as correct numbers
                // say what you got:
                //Serial.print("BjarneOst: ");
                Serial.println("*************************");
                //Serial.println(incomingByte, DEC);
                Serial.println(incomingByte, DEC);
                Serial.println("*************************");
                if (incomingByte==1){
                  Serial.print("SUCCESS");
                  moveNow();
                  }
        
       } 
        //random number from 0 to 2
        randNumber = random(3);
        switch (randNumber) {
          case 0:
          {
          action6();
          }
          break;
          case 1:
          {
          action0();
          }
          break;
          case 2:
          {
          delay(1000);
          }
          break;
          case 3:
          {
          action6();
          }
          break;
        }
        
  
}

