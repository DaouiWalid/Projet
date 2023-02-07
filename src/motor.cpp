#include "main.h"

//declaration des pines 
// Motor A
int motor1Pin1 = 25; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
// Motor B
int motor2Pin1 = 32; 
int motor2Pin2 = 33; 
int enable2Pin = 17; 


void initPinsMotors(){
    // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
 
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
}

void speed_motor(){
  /*analogWrite(enable1Pin,255);
  analogWrite(enable2Pin,255);*/
  digitalWrite(enable2Pin,HIGH);
  digitalWrite(enable1Pin,HIGH);
}

void goForward(){
    //speed_motor();
    Serial.println("I am in Forward!");
    /** Attendre que l'evenement Forward arrive */
    digitalWrite(2,HIGH); // led light
    Serial.println("Moving Forward");
    // controler le motor1 dana le sense >>
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor1Pin1, LOW);
    // controler le motor2 dana le sense >>
    digitalWrite(motor2Pin2, HIGH); 
    digitalWrite(motor2Pin1, LOW);
}

void stop(){
     // for test: 
    Serial.println("the rover is stoped");
    digitalWrite(2,HIGH);
    
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor1Pin1, LOW); 
    digitalWrite(motor2Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW); 
}


void goLeft(){
  //speed_motor();
    // for test: 
  digitalWrite(2,HIGH);
  Serial.println("Moving Left ");
  // controler le motor1 dana le sense <
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor1Pin1, HIGH); 
  // controler le motor2 dana le sense >
  digitalWrite(motor2Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW); 

  /** after a few second it stop **/
    delay(200);

    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor1Pin1, LOW); 
    digitalWrite(motor2Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    delay(600);
}

void goRight(){
  //speed_motor();
  // for test: 
    digitalWrite(2,HIGH);
    Serial.println("Moving Right");
    // controler le motor1 dana le sense >
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor1Pin1, LOW); 
    // controler le motor2 dana le sense <
    digitalWrite(motor2Pin2, LOW); 
    digitalWrite(motor2Pin1, HIGH);

    /** after a few second it stop **/
    delay(200);

    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor1Pin1, LOW); 
    digitalWrite(motor2Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    delay(600);
}

void goBack(){
  //speed_motor();
    Serial.print("moving back");
    // controler le motor1 dana le sense <<
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor1Pin1, HIGH);
    // controler le motor2 dana le sense <<
    digitalWrite(motor2Pin2, HIGH); 
    digitalWrite(motor2Pin1, LOW);

    delay(650);

    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);

    delay(600);
}