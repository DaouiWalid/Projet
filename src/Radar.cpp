#include <Arduino.h>
const int pinTrig = 5;
const int pinECHO = 18;
const float SoundSpeed = 0.034;

void initPin(){
    pinMode(pinTrig,OUTPUT);
    pinMode(pinECHO,INPUT);
}

double getDistance(){
    int distanceCm;
    int distanceM;
    int durationPulse;

    digitalWrite(pinTrig,LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig,HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig,LOW);

    durationPulse = pulseIn(pinECHO,HIGH);
    distanceM = SoundSpeed * durationPulse/2;
    //distanceCm = distanceM *10^2;
    return distanceM;
}