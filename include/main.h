#ifndef main_h
#define main_h

#include <Arduino.h>
#include "Radar.h"
#include <freertos/semphr.h> // library to use semaphore and mutex 
#include <freertos/queue.h> // library to use Queue
#include "motor.h"
#include "Servo.h"

#define STOP_EVENT 1
#define FRWARD_EVENT 2
#define SERVO_EVENT 4
#define LEFT_EVENT 8
#define RIGHT_EVENT 16
#define BACK_EVENT 32

#endif