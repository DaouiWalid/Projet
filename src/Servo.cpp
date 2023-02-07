#include <ESP32_Servo.h>
#include "Servo.h"
#include "Radar.h"
#include <Arduino.h>

Servo servo; // objet de Servo

void servoInit(){ 
    servo.attach(4); // relier le servo to a pin
    servo.write(Straight); // initialement regarder vers l'avant
}

double lookAround(int alpha){
    double distance;
    servo.write(alpha);
    delay(400);
    distance = getDistance();
    delay(2000);
    servo.write(Straight);
    return distance;
}