//FarkleBot Arduino Code
//Authors: Anna Ailworth (Serial Communication, Dev Commands), Austin Dozier (Motor Control), and Rimonda Aziz (Wiring)
//
#include "math.h"
#include "Arduino.h"
#include <Servo.h>
#include <Stepper.h>
#include <Wire.h>
#define STEPS 3200   

///////////////Timing////////////////
unsigned long thismilli;

//////////////BotControl////////////////
Stepper stepperArm(STEPS, 6, 7);   //Dir=23(brown) Step=22(orange)
Stepper stepperTray(STEPS, 2, 3); //Dir=25(black)  step=24(blue)
Servo syringeServo;  // create servo object to control a servo
Servo cupServo;  // create servo object to control a servo
Servo linearServoExtend;  // create servo object to control a servo
Servo linearServoRetract;  // create servo object to control a servo
int diceCount = 0;
int diceTray = 0;
int diceCup = 0;
int xPos;
int yPos;
int armPos;
int trayPos; 
////////////////Serial Commends///////////////
#define SyringeSuck 0
#define ToCup 20
#define ToTray 21
#define GoToX 22
#define GoToY 23
#define SyringeRelease 100
#define CupTilt 1
#define CupReturn 101
#define LinearExtend 2
#define LinearRetract 102
#define BlinkLED 3
#define StopLED 103
#define ArmExtend 104
#define ArmRetract 105
#define TrayLeft 106
#define TrayRight 107
#define SendCmd 108
#define CamExtend 109
#define CamRetract 9
//#define setOrigin 110
#define Ready 300

/////////////Serial communication constants////////////////////////
int checksum;
int address, data;
int lin;
int up = 3;
int dwn = 5;
int i;
int x;
int y;
int p;
int j;
int k;
int X;
int Y;
int X2;
int Y2;
int XX;
int YY;
String content = "";
char character;
#define ONESECVAL 506250 //One second with of clock cycles with Atmel 2560 chip
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64
#define BaudRate 115200 //Should match any software baud rate
#define CommunicationLength 5 // number of bytes in array coming from PC

/////////////// Bit manipulation shortcuts//////////////////////
#define SETBIT(a,b) ((a) |= (1<<(b))) //Make a bit on a port high (1) (+5V)
#define CLEARBIT(a,b) ((a) &= ~(1<<(b))) //Make a bit on a port low (0) (0V)
#define TOGGLEBIT(a,b) ((a) ^= (1<<(b))) //Flip a bit on a port from high to low or low to high (1->0 or 0->1)
#define CHECKBIT(a,b) ((a) & (1<<(b))) //Interogate bit on port
bool contact = HIGH;
//////////////////Function List/////////////////////////
void ProcessCommand(void);
void ReadCommand(void);
void SendResponse(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);
void SendResponse(String type, int value);

///////////////////////Incoming Serial Command///////////////////////
uint8_t IncomingMessage[CommunicationLength], OutgoingMessage[CommunicationLength];


/////////////////////Initialize/////////////////
void setup() {
  Wire.begin();
  syringeServo.attach(24);             // attaches servo to pin 30 
  cupServo.attach(17);                 // attaches servo to pin
  pinMode(9, OUTPUT);                  //Linear Actuator pin
  pinMode(10, OUTPUT);                 //Linear Actuator pin
  pinMode(38, INPUT);                  //Arm Retracted Limit
  pinMode(39,INPUT);                   //Arm Extended Limit
  pinMode(40,INPUT);                   //Tray Left Limit
  pinMode(41,INPUT);                   //Tray Right Limit
  pinMode(42,OUTPUT);                  // Cam Linear Actuator
  pinMode(43,OUTPUT);                  // Cam Linear Actuator
  pinMode(50,OUTPUT);
  digitalWrite(50,HIGH);
  digitalWrite(10,HIGH); 
  digitalWrite(41,LOW);
  digitalWrite(42,LOW);
  pinMode(LED_BUILTIN, OUTPUT);

  stepperArm.setSpeed(800);           //RPM
  stepperTray.setSpeed(800);          //RPM
  i==0;
  
  cupReturn();
  syringeRelease();
  goHome();

  thismilli = millis();
  Serial.begin(115200);               //Must match Pi Baud
}

void CheckAndCorrect(int Pin, bool Check)
{
  if(Check)
  {digitalWrite(Pin,LOW);}
  else
  {digitalWrite(Pin, HIGH);}
}

/////////////Process Incoming Serial Command////////////////
void ReadCommand(void)
{
  for (int i = 0; i < 5; i++)
  {
    IncomingMessage[i] = 0; //Clear message
    
  }
  uint8_t ind = 0;
  while(/*uart_available()*/Serial.available()>0)
  {
    
    IncomingMessage[ind] = Serial.read();
    delayMicroseconds(1030);
    //uart_putchar(IncomingMessage[ind]);
    ind++;
    if (ind > 5)
    {break;}

  }
  EchoCheckSum();
}

//////////////////Check Incoming Message////////////////////
void EchoCheckSum()
{
  checksum = 0;
  for (int i = 0; i<= 4; i++)
  {
    checksum = checksum + IncomingMessage[i];
  }

  //SendResponse("Echo", checksum);
}

/////////////Sending Response to Pi//////////////////
void SendResponse(String type, float value)
{
  String theStr = "BREAK Response " + type + " " + String(value) + "BREAK";
  Serial.print(theStr);
}
void SendResponse(String type, int value)
{
  String theStr = "BREAK Response " + type + " " + String(value) + "BREAK";
  Serial.print(theStr);
  
}
//Example
//void ResponseWithError()
//{SendResponse("LimitError", ERRORNUM);}

//////////////Reading Commands from the Pi////////////////
void ProcessCommand(void)
{
    ReadCommand();
    switch (IncomingMessage[0])
    {
      case SyringeSuck:
        syringeSuck();
        break;
      case SyringeRelease:
        syringeRelease();
        break;
      case CupTilt:
        cupTilt();
        break;
      case CupReturn:
        cupReturn();
        break;
      case LinearExtend:
        zExtend();
        break;
      case LinearRetract:
        zRetract();
        break;
      case BlinkLED:
         Blinky();
         break;
      case StopLED:
         NoBlinky();
         break;
      case ArmExtend:
         armExtend(IncomingMessage[2]);
         break;
      case ArmRetract:
        armRetract(IncomingMessage[2]);
        break;
      case TrayLeft:
         trayLeft(IncomingMessage[2]);
         break;
      case TrayRight:
         trayRight(IncomingMessage[2]);
         break;
      case SendCmd:
        SendResponse("LimitError", 1);
        break;
      case CamRetract:
        cameraRetract();
        break;
      case CamExtend:
        cameraExtend();
        break;
      case ToCup:
        toCup();
        break;
      case ToTray:
        toTray();
        break;
      case GoToX:
        goToX(IncomingMessage[2]);
        break;
      case GoToY:
        goToY(IncomingMessage[2]);
        break;
      default:
        break;
    }
}

/////////////////Turn on the built in Board LED//////////
void Blinky(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)       
}
////////////////Turn off the built in LED//////////////
void NoBlinky(){
  digitalWrite(LED_BUILTIN, LOW);
}
/////////////////////LINEAR ACTUATOR CONTROL ///////////////////////////////////
void zExtend() {
  digitalWrite(10,LOW);
  analogWrite(9,255);
  delay(3000);     ///Time to fully extend
  analogWrite(9,0);
  lin = dwn;
}
void zRetract(){
  digitalWrite(10,HIGH);
  analogWrite(9,255);
  delay(3000);       ////Time to fully retract
  analogWrite(9,0);
  lin = up;
}
  /////////////////////LINEAR ACTUATOR EXTEND AND RETRACT///////////////////////////////////
  void extend(int del){
  //Extend Linear Actuator
  digitalWrite(10,LOW);
  analogWrite(9,255);
  delay(del); //change if short
  analogWrite(9,0);
  lin = dwn;
  }
  void retract(int del){
  digitalWrite(10,HIGH);
  analogWrite(9,255);
  delay(del); //change if short
  analogWrite(9,0);
  if (del >2500){
  lin = up;
  }
  else{
    lin = dwn;
  }
  }
////////////////////Camera Extend/////////////////
void cameraExtend() {
  digitalWrite(43,HIGH);
  delay(13000);     ///Time to fully extend
  digitalWrite(43,LOW);
}
void cameraRetract(){
  digitalWrite(42,HIGH);
  delay(13000);     ///Time to fully extend
  digitalWrite(42,LOW);
}
/////////////////////SYRINGE SERVO MOTOR CONTROL///////////////////////////////////
void syringeSuck() {                   // Call comand to actuate Syringe Piston
  syringeServo.write(140);                  // sets the servo position according to the scaled value
  delay(15);                           
}
void syringeRelease() {                   // Call comand to actuate Syringe Piston
  syringeServo.write(10);                  
  delay(15);                           
}
/////////////////////CUP SERVO MOTOR CONTROL ///////////////////////////////////
void cupTilt() {     // Call comand to actuate Syringe Piston
    cupServo.write(55);
    delay(200);
    cupServo.write(155);                  // sets the servo position according to the scaled value
  delay(150);
}
void cupCatch() {                   // Call comand to actuate Syringe Piston
  cupServo.write(100);                  // sets the servo position according to the scaled value
  delay(150);
}
  void cupReturn(){
  cupServo.write(70);            // adjust to fit strait up cup possition******************         
  delay(150);    
}
/////////////////////ARM STEPPER MOTOR CONTROL - EXTEND ///////////////////////////////////
void armExtend(int armPos) {
    if (armPos < 0){
    armPos = abs(armPos);
    armRetract(armPos);
  }
  else{
  j = 0;
  while(digitalRead(39) != HIGH and j < armPos){
   stepperArm.step(-500);
   j == j++;
  }
  yPos = yPos + armPos;
  return yPos;
  }
}
/////////////////////ARM STEPPER MOTOR CONTROL - RETRACT ///////////////////////////////////
void armRetract(int armPos) {
  if (armPos < 0){
    armPos = abs(armPos);
    armExtend(armPos);
  }
  else{
  j = 0;
  while(digitalRead(38) != HIGH and j < armPos)  {
  stepperArm.step(500);
  j == j++;
  }
  yPos = yPos - armPos;
  return yPos;
  }
}
/////////////////////TRAY STEPPER MOTOR CONTROL - MOVE RIGHT///////////////////////////////////
void trayRight(int trayPos) {  
   if (trayPos < 0){
    trayPos = abs(trayPos);
    trayLeft(trayPos);
  }
  else{
  j = 0;
  while(digitalRead(41) != HIGH and j < trayPos)  {
  stepperTray.step(-460);
  j == j++;
  }
  xPos = xPos - trayPos;
  return xPos;
}
}
/////////////////////TRAY STEPPER MOTOR CONTROL - MOVE LEFT///////////////////////////////////
void trayLeft(int trayPos) {  
    if (trayPos < 0){
     trayPos = abs(trayPos);
     trayRight(trayPos);
  }
  else{
  j = 0;
  while(digitalRead(40) != HIGH and j < trayPos) { 
  stepperTray.step(460);
  j == j++;
  }  
  xPos = xPos + trayPos;
  return xPos;
  }
}
/////////////////////GoTo Functions///////////////////////////////////
/////////////////////Initilization///////////////////////////////////
void goHome() {
  if (lin == up){
    while (digitalRead(38) != HIGH){
 armRetract(50);
}
  while (digitalRead(41) != HIGH){
 trayRight(50);
}
  }
 else {
  retract(3000);
  goHome();
 }
 syringeRelease();
 cupReturn();
 xPos = 0;
 yPos = 0;
//SendResponse(Ready,200); //Send serial comm to pi Arm is ready and in possition
}
void goHomeCup() {
  if (lin == up){
    while (digitalRead(38) != HIGH){
 armRetract(50);
}
syringeRelease();
  while (digitalRead(41) != HIGH){
 trayRight(50);
}
  }
 else {
  retract(3000);
  goHome();
 }

xPos = 0;
yPos = 0;
//SendResponse(Ready,200); //Send serial comm to pi Arm is ready and in possition
}

/////////////////////Go To X///////////////////////////////////
void goToX(int X){
//  constrain(X,0,483);
//  map(X,180,450,18,118);
  if(xPos==0){
    trayLeft(X);
    XX = X; 
  }
  else{
    X2 = X - XX;
    trayLeft(X2); 
    XX = X;
   
  }   
  }
/////////////////////Go To Y///////////////////////////////////
void goToY(int Y){
//  constrain(Y,0,278);
//  map(Y,0,270,0,100);
    if(yPos==0){
    armExtend(Y);
    YY = Y;   
  }
  else{
    Y2 = Y - YY;
    armExtend(Y2);
    YY = Y;   
  }   
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void goTo(int X, int Y){
  if(xPos==0 and yPos==0){
    armExtend(Y);
    trayLeft(X);
    XX = X;
    YY = Y;   
  }
  else{
    X2 = X - XX;
    Y2 = Y - YY;
    armExtend(Y2);
    trayLeft(X2); 
    XX = X;
    YY = Y;   
  }   
  }

void nextDie(){
 //yPos - armPos;
 //xPos - trayPos;
}
void count(){
diceCount = diceCup + diceTray;
if (diceCount = 6){
  goHome();
}
if (diceCount < 6){
  nextDie();
}
}
void fire(){
  if (digitalRead(38) != HIGH)
   cupServo.write(100);
   extend(2850);
  //Grab die
  if(syringeServo.read() == 10) {
  syringeServo.write(140);
  delay(500);}
    else if (syringeServo.read() < 50){
      syringeServo.write(10);
      delay(500);
      syringeServo.write(140);
      delay(500);
}
}
void toCup(){  
  fire();
  retract(3000);
    cupCatch();
//return to origin above cup
  goHomeCup();
  cupReturn();
  diceCup = diceCup ++;
  }
void toTray(){
  fire();
  retract(300);
//extend to tray corner             //check
  while(digitalRead(39) != HIGH){   //check
     stepperArm.step(-50);          //check
  }                                     //check
  //while(digitalRead(41) != HIGH){   //check
  //   stepperTray.step(50);          //check
 // }                                 //check
syringeServo.write(10);
diceTray = diceTray ++;
}
void doCereal(){
   if (/*uart_available()*/Serial.available() > 0)       //wait for serial command
  {
   ProcessCommand(); 
  }
  thismilli = millis();                                 //Capture time
  analogWrite(9,0);
  contact = digitalRead(22);
  if (digitalRead(39) == HIGH) {
     SendResponse("LimitError", 1);
     delay(2000);
  }
   if (digitalRead(38) == HIGH) {
     SendResponse("LimitError", 1);
     delay(2000);
  }  
}
///////////////////////TESTING////////////////////////////////
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
/////////////////////CALIBRATION//////////////////////////////
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
///////////////////INITIALIZATION/////////////////////////////
void calibrate(){
  calibrateArm();
  calibrateTray();
  goHome();
}
void calibrateArm(){
  y == 1;
while (digitalRead(39) != HIGH){
  armExtend(1);
  y == y++;
}
  Serial.print("Y Ticks: ");
  Serial.println(y);
return y;
}
void calibrateTray(){
  x == 1;
while (digitalRead(40) != HIGH){
  trayLeft(1);
  x == x++;
}
  Serial.print("X Ticks: ");
  Serial.println(x);
return x;

}
void corners(){
   goHome();
  delay(500);
  goTo(0,121);
  updatePos();
  delay(500);
  goTo(100,121);
  updatePos();
  delay(500);
  goTo(100,21);
  updatePos();
  delay(500);
  goTo(0,21);
  updatePos();
  delay(500);
  goHome();
}
void updatePos(){
  Serial.print("X Possition: ");
  Serial.println(xPos);
  Serial.print("Y Possition: ");
  Serial.println(yPos);
}

void loop() {
 if (/*uart_available()*/Serial.available() > 0)       //wait for serial command
  {
   ProcessCommand(); 
  }
  thismilli = millis();                                 //Capture time
  analogWrite(9,0);
  contact = digitalRead(22);
//  if (digitalRead(39) == HIGH) {
//     SendResponse("LimitError", 1);
//     delay(2000);
//  }
//   if (digitalRead(38) == HIGH) {
//     SendResponse("LimitError", 1);
//     delay(2000);
//  }  
}
