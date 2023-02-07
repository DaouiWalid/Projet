#include "main.h"

TaskHandle_t DistanceHandle;
TaskHandle_t EventHandle;
TaskHandle_t ServoHandle;

/******   Declaration de la Queue  *****/
QueueHandle_t DistanceQueue;

/******* Declaration du Groupe Event to handle the actions *******/
EventGroupHandle_t ActionsEvent;

/***  Declaration des semaphores pour la synchronisation de l'ajout et le prelevement de la file ****/
SemaphoreHandle_t semaphore;
SemaphoreHandle_t semaphore_2;
SemaphoreHandle_t mutexControlQueue;
SemaphoreHandle_t mutexF;
SemaphoreHandle_t mutexS;

void handleDistance(void *pvParameters){
  while(1){
    //xSemaphoreTake(mutexControlQueue,portMAX_DELAY);
    Serial.println("I am in handleDistance!");
    double distanceToSet = getDistance();
    Serial.println(distanceToSet);
    // xQueueSend permet d'ajouter un element dans la queue de la file
    // DistanceQueue: la file ,&distanceToSet: pointeur vers la valeur a inserer,portMAX_DELAY: delay d'attent si la file est pleine
    xQueueSend(DistanceQueue,&distanceToSet,portMAX_DELAY);
    xSemaphoreGive(semaphore_2); 
   vTaskDelay(800/portTICK_PERIOD_MS); // save distance in queue every 400 ms
   //xSemaphoreGive(semaphore_2); 
  }
}

void handleEvent(void *pvParameters){
  while(1){
    xSemaphoreTake(semaphore_2,portMAX_DELAY);
    Serial.println("I am in handleEvent!");
    double distanceToGet;
    // recuperer la distance de la queue de la file
    if(xQueueReceive(DistanceQueue,&distanceToGet,portMAX_DELAY)==pdTRUE){
      if (distanceToGet >= 80){
        // set the bit Forward
        xEventGroupSetBits(ActionsEvent,FRWARD_EVENT);
        Serial.print("The forward bit is set ");
      }else{
        Serial.print("The Stop bit is set ");
        xEventGroupSetBits(ActionsEvent,STOP_EVENT);
      }
    }else{
      Serial.print("Erreur l\'ors de la reception de la distance !");
    }
  }
}

void TaskForward(void *pvParameters)
{
  while(1){ 
    xEventGroupWaitBits(ActionsEvent,FRWARD_EVENT,pdTRUE,pdTRUE,portMAX_DELAY);
    goForward();
  }
}
/*
void control(void *pvParameters){
  while(1){
    Serial.print(" hi hi hi hi Moving with speed = ");
    analogWrite(enable1Pin,245);
    analogWrite(enable2Pin,245);
  } 
}
*/

void TaskBack(void *pvParameters){
  while(1){
    xEventGroupWaitBits(ActionsEvent,BACK_EVENT,pdTRUE,pdTRUE,portMAX_DELAY);
    Serial.println("Moving back");
    goBack();
    vTaskResume(DistanceHandle);
    vTaskResume(EventHandle);
  }
}


void Taskstop(void *pvParameters){
  while(1){
    xEventGroupWaitBits(ActionsEvent,STOP_EVENT,pdTRUE,pdTRUE,portMAX_DELAY);
    /** On doit suspendu les deux taches pour donner la main au servo de choisir la direction right or left or back **/
    vTaskSuspend(EventHandle);
    vTaskSuspend(DistanceHandle);

    Serial.println("I am in Stop!");
    stop();
    xSemaphoreGive(semaphore); // donner le tiquer pour le serveau moteur
    delay(1000);
  }
}


void TaskServo(void *pvParameters){
  double distanceRight,distanceLeft;
  while(1){
    xSemaphoreTake(semaphore,portMAX_DELAY);
    Serial.print("I am in serveau task !!!! ");
    distanceRight =lookAround(Degree_90_Right);
    distanceLeft =lookAround(Degree_90_Left);
    Serial.print("distance Gauche = ");
    Serial.println(distanceLeft);
     Serial.print("distance Droite = ");
    Serial.println(distanceRight);
    if((distanceLeft > 80)||(distanceRight > 80)){
      if ( distanceLeft >= distanceRight ){
        xEventGroupSetBits(ActionsEvent,LEFT_EVENT);
      }else{
        xEventGroupSetBits(ActionsEvent,RIGHT_EVENT);
      }
    }else{
      xEventGroupSetBits(ActionsEvent,BACK_EVENT);
    }
  }
}

void TaskLeft(void *pvParameters){
  while(1){
    xEventGroupWaitBits(ActionsEvent,LEFT_EVENT,pdTRUE,pdTRUE,portMAX_DELAY);
   goLeft();
   vTaskResume(DistanceHandle);
   vTaskResume(EventHandle);
  }
}

void TaskRight (void *pcParameters){
  while(1){
    xEventGroupWaitBits(ActionsEvent,RIGHT_EVENT,pdTRUE,pdTRUE,portMAX_DELAY);
    goRight(); 
    vTaskResume(DistanceHandle);
    vTaskResume(EventHandle);
  }
 
}


void setup() {
  Serial.begin(9600);
  // creation de la queue: 
  DistanceQueue=xQueueCreate(10,sizeof(double));   // 10 est la taille de la file, sizeof(double) est la taille de l'element

  // creation de l'event Group:
  ActionsEvent = xEventGroupCreate();

  // creation des semaphore et mutex
  semaphore = xSemaphoreCreateBinary();// syncronize stop with servo
  semaphore_2 = xSemaphoreCreateBinary(); // syncronize handleD with handleE tq : handleD ---> handleE
  mutexF = xSemaphoreCreateMutex();
  mutexS = xSemaphoreCreateMutex();
  mutexControlQueue = xSemaphoreCreateMutex();


  // initialiser les pins du radar 
  initPin();

  //speed control:
  //speed_motor();

  // initialiser les pins des motors
  initPinsMotors();

  // initialiser les pins du servo 
  servoInit();

  // for test:
  pinMode(2, OUTPUT);

  //initialiser une communication en serie
  Serial.println("Testing DC Motor...");

  xTaskCreate(handleDistance,"handle distance",1024,NULL,1,&DistanceHandle);
  xTaskCreate(handleEvent,"handle Event",1024,NULL,1,&EventHandle);
  xTaskCreate(TaskForward,"forward",1024,NULL,1,NULL);
  xTaskCreate(Taskstop,"stop",1024,NULL,1,NULL);
  xTaskCreate(TaskServo,"servo",1024,NULL,1,NULL);
  xTaskCreate(TaskBack,"Back",1024,NULL,1,NULL);
  xTaskCreate(TaskLeft,"Left",1024,NULL,1,NULL);
  xTaskCreate(TaskRight,"Right",1024,NULL,1,NULL);


}

void loop() {
}