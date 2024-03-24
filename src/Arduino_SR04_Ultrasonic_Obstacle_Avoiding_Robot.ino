#include <dummy.h>

/* Pushan Gupta ###########################
 * The Ultrasonic Obstacle Avoider ########
###########################################*/

#include <NewPing.h>    //Include Servo libraries.
#include <Servo.h>

Servo myservo;    //Create Servo Object to control a servo.

#define TRIGGER_PIN  14   //Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     16   //Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400  //Maximum distance we want to ping for (in centimeters) - Maximum sensor distance is rated at 400-500cm.
#define SERVO 2

//DECLARE VARIABLES
int distance;
bool isZero = false; 
int count = 0;
int servoLeftVal;
int servoCenterVal;
int servoRightVal;
int measureCount = 0;

//CREATE PING SENSOR OBJECT
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);   //NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600);
  delay(2000);
  myservo.attach(2, 1000, 2200);    //Attaches the servo on Pin 2 to the Servo Object.
  
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);   //Left Motor
  pinMode(5, OUTPUT);

  pinMode(12, OUTPUT);  //Right Motor
  pinMode(13, OUTPUT);
  delay(50);

  pinMode(0, OUTPUT);   //Left LED (Orange Wires)
  pinMode(15, OUTPUT);  //Right LED (Green Wires) 
  pinMode(1, OUTPUT);   //Avoid Obstacle (Reverse) Blue LED (White Wires)
  pinMode(3, OUTPUT);   //Passive Buzzer Sound (Blue Wires)  

  for (int i = 0; i <= 180; i++)
  {
    myservo.write(i);
    delay(10);
  }

  for (int i = 180; i >= 0; i--)
  {
    myservo.write(i);
    delay(10);
  }

  myservo.write(95);    //Center the servo.
  delay(50);  
}

void loop() {

  distance=(sonar.ping_median(10) / 2) / 74;    //Do multiple pings [10], and return the median in microseconds.
  delay(50);                                    //Convert to a distance, and save in a variable.
  Serial.println(distance);
    if (distance < 10) {    //If distance threshold is met, do something else.
      avoidObstacle();      
    }else{
      goForward();
   }       
}

//CHECK FOR NULL[0] READINGS
void checkZero(){
  for (int x=0; x < 20; x++){
    distance = (sonar.ping_median(10) / 2) / 74;
    
    if (distance == 0){
      isZero=true;
    }else{
      isZero=false;
      count++;
    }
    delay(100);
  }

  if (isZero == true && count > 0){
    ESP.restart();    //If consecutive zeroes for 2 seconds, reboot.
  }
}

void goForward(){
  
  digitalWrite(4, HIGH);
  digitalWrite(12, HIGH);

  digitalWrite(5, LOW);
  digitalWrite(13, LOW);  
}

void turnRight(){
  digitalWrite(4, LOW);   //REVERSE
  digitalWrite(12, LOW);

  digitalWrite(5, HIGH);
  digitalWrite(13, HIGH);

  delay(250);

  for (int i = 0; i <= 3; i++)
  {
    digitalWrite(15, HIGH);  //FLICKER RIGHT LED ON AND OFF x 3 TIMES
    delay(50);
    digitalWrite(15, LOW);
    delay(50);
  }

  digitalWrite(4, HIGH);    //TURN RIGHT
  digitalWrite(12, LOW);

  digitalWrite(5, LOW);
  digitalWrite(13, HIGH);

  delay(250);

  digitalWrite(4, LOW);     //STOP
  digitalWrite(12, LOW);

  digitalWrite(5, LOW);
  digitalWrite(13, LOW);

  delay(1000);
  
  goForward();              //FORWARD
}

void turnLeft(){
  
  digitalWrite(4, LOW);     //REVERSE
  digitalWrite(12, LOW);

  digitalWrite(5, HIGH);
  digitalWrite(13, HIGH);

  delay(250);

  for (int j = 0; j <= 3; j++)
  {
      digitalWrite(0, HIGH);    //FLICKER LEFT LED ON AND OFF x 3 TIMES
      delay(50);
      digitalWrite(0, LOW);
      delay(50);
  }

  digitalWrite(4, LOW);     //TURN LEFT
  digitalWrite(12, HIGH);

  digitalWrite(5, HIGH);
  digitalWrite(13, LOW);

  delay(250);

  digitalWrite(4, LOW);     //STOP
  digitalWrite(12, LOW);

  digitalWrite(5, LOW);
  digitalWrite(13, LOW);

  delay(1000);
  
  goForward();             //FORWARD
}

void goStraight(){

  goForward();  
}

void avoidObstacle(){

  digitalWrite(1, HIGH);      //TURN BLUE LED ON

  tone(3, 500, 3000);         //TURN PASSIVE BUZZER ON FOR 3s

  digitalWrite(4, LOW);       //REVERSE
  digitalWrite(12, LOW);

  digitalWrite(5, HIGH);
  digitalWrite(13, HIGH);

  delay(250);

  digitalWrite(1, LOW);       //TURN BLUE LED OFF 

  digitalWrite(4, LOW);       //STOP
  digitalWrite(12, LOW);

  digitalWrite(5, LOW);
  digitalWrite(13, LOW);
  
  takeMeasurements();

  compareDistance();   
}

//Compares the left/center/right distances to determine the best path to take.
void compareDistance(){

  if (Left_is_biggest(servoLeftVal, servoRightVal, servoCenterVal)){
    turnLeft();
  }else if (Right_is_biggest(servoLeftVal, servoRightVal, servoCenterVal)){
    turnRight();
  }else if (Center_is_biggest(servoLeftVal, servoRightVal, servoCenterVal)){
    goStraight();
  }
}

bool Left_is_biggest(int servoLeftVal, int servoRightVal, int servoCenterVal) {
    return servoLeftVal > servoRightVal && servoLeftVal > servoCenterVal;
}
bool Right_is_biggest(int servoLeftVal, int servoRightVal, int servoCenterVal) {
    return servoRightVal > servoLeftVal && servoRightVal > servoCenterVal;
}
bool Center_is_biggest(int servoLeftVal, int servoRightVal, int servoCenterVal) {
    return servoCenterVal > servoLeftVal && servoCenterVal > servoRightVal;
}

void takeMeasurements(){
  //CHECK LEFT
  myservo.write(0);
  servoLeftVal = (sonar.ping_median(10) / 2) / 74;    //There are 73.746uS in an inch (1130 ft/sec).
  delay(1000);
  //CHECK CENTER
  myservo.write(95);
  servoCenterVal = (sonar.ping_median(10) / 2) / 74;
  delay(1000);
  //CHECK RIGHT
  myservo.write(180);
  servoRightVal = (sonar.ping_median(10) / 2) / 74;
  delay(1000);
  myservo.write(95);
  delay(1000);

  if ((servoLeftVal == servoCenterVal) && (servoCenterVal == servoRightVal)){
    measureCount++;
    takeMeasurements();

    //If measurements are identical for 3 consecutive readings, just go right.
    if (measureCount > 3){
      measureCount = 0;
      turnRight();
    }
  }
}