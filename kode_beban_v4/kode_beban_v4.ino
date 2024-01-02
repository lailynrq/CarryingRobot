#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#define ENA 9
#define ENB 10
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define TRIG1 3
#define ECHO1 2
#define TRIG2 8
#define ECHO2 11

// Task handles
TaskHandle_t motorATask;
TaskHandle_t motorBTask;
TaskHandle_t ultrasonicTask1;
TaskHandle_t ultrasonicTask2;

// Semaphores for motor control
SemaphoreHandle_t motorASemaphore;
SemaphoreHandle_t motorBSemaphore;
SemaphoreHandle_t ultrasonicMutex;

// Queue for motor speeds
QueueHandle_t motorSpeedQueue;

// Shared variables
long distance1 = 0;
long distance2 = 0;

// Motor Priority
int motorAPriority = 1;
int motorBPriority = 1;

// Function prototypes for task functions
void motorATaskFunction(void* parameter);
void motorBTaskFunction(void* parameter);
void ultrasonicTaskFunction(void* parameter);
void changePriority(int newPriority);

void changePriority(int newPriority) {
  vTaskPrioritySet(motorATask, newPriority);
  vTaskPrioritySet(motorBTask, newPriority);
}

void setup() {
  Serial.begin(9600);

  // Initialize the pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Create semaphores for motor control
  motorASemaphore = xSemaphoreCreateMutex();
  motorBSemaphore = xSemaphoreCreateMutex();
  ultrasonicMutex = xSemaphoreCreateMutex();

  // Create Queue for motor speeds
  motorSpeedQueue = xQueueCreate(1, sizeof(int));

  // Create tasks for each motor
  xTaskCreate(motorATaskFunction, "MotorATask", 100, NULL, 1, &motorATask);
  xTaskCreate(motorBTaskFunction, "MotorBTask", 100, NULL, 1, &motorBTask);

  // Create tasks for ultrasonic sensors
  xTaskCreate(ultrasonicTaskFunction, "UltrasonicTask1", 100, (void*)1, 3, &ultrasonicTask1);
  xTaskCreate(ultrasonicTaskFunction, "UltrasonicTask2", 100, (void*)2, 3, &ultrasonicTask2);

  // Set prioritas awal
  changePriority(motorAPriority);
  changePriority(motorBPriority);
  
  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // This function should be empty when using FreeRTOS.
  // All code execution should be done in tasks.
}

void motorATaskFunction(void* parameter) {
  (void)parameter; // Unused
  int motorSpeed;
  while (1) {
    // Receive motor speed from the queue
    if (xQueueReceive(motorSpeedQueue, &motorSpeed, portMAX_DELAY) == pdTRUE) {
      // Request access to the motor A
      if (xSemaphoreTake(motorASemaphore, portMAX_DELAY) == pdTRUE) {
        analogWrite(ENA, motorSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        // Release the semaphore to allow other tasks to access the motor
        xSemaphoreGive(motorASemaphore);
      }
    }
    // You can add a delay here if needed
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
  }
}

void motorBTaskFunction(void* parameter) {
  (void)parameter; // Unused
  int motorSpeed;
  while (1) {
    // Receive motor speed from the queue
    if (xQueueReceive(motorSpeedQueue, &motorSpeed, portMAX_DELAY) == pdTRUE) {
      // Request access to the motor B
      if (xSemaphoreTake(motorBSemaphore, portMAX_DELAY) == pdTRUE) {
        analogWrite(ENB, motorSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        // Release the semaphore to allow other tasks to access the motor
        xSemaphoreGive(motorBSemaphore);
      }
    }
    // You can add a delay here if needed
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
  }
}

void ultrasonicTaskFunction(void* parameter) {
  int sensorId = (int)parameter;
  int trigPin, echoPin, motorSpeed;

  if (sensorId == 1) {
    trigPin = TRIG1;
    echoPin = ECHO1;
  } else {
    trigPin = TRIG2;
    echoPin = ECHO2;
  }

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  while (1) {
    // Trigger ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the ultrasonic sensor
    long duration = pulseIn(echoPin, HIGH);

    // Acquire mutex before updating the shared variable
    if (xSemaphoreTake(ultrasonicMutex, portMAX_DELAY) == pdTRUE) {
      if (sensorId == 1) {
        distance1 = duration / 58.2; // simplified form
      } else {
        distance2 = duration / 58.2; // simplified form
      }
      xSemaphoreGive(ultrasonicMutex); // Release the mutex
    }

    Serial.print("Distance Sensor ");
    Serial.print(sensorId);
    Serial.print(": ");
    Serial.print(sensorId == 1 ? distance1 : distance2);
    Serial.println(" cm");

    if (sensorId == 1) {
      if (distance1 >= 0 && distance1 >= 40){
        motorSpeed = 255 / 4;
      }
      else {
        motorSpeed = 0;
      }
      //motorSpeed = (distance1 >= 0 && distance1 >= 40) ? 255 / 4 : 0;
    } else {
      if (distance2 >= 0 && distance2 <= 40){
        motorSpeed = 255 / 4;
        if (motorAPriority != 2 && motorBPriority != 2) {
          motorAPriority = 2;
          motorBPriority = 2;
          changePriority(motorAPriority);
          changePriority(motorBPriority);
        }
      }
      else {
        motorSpeed = 0;
        if (motorAPriority != 1 && motorBPriority != 1) {
          motorAPriority = 1;
          motorBPriority = 1;
          changePriority(motorAPriority);
          changePriority(motorBPriority);
        }
      //motorSpeed = (distance2 >= 0 && distance2 <= 40) ? 255 / 4 : 0;
      }
    }
    xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);

    // You can add a delay here if needed
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
  }
}
