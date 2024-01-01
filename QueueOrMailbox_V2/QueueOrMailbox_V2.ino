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
#define trigPinA 3
#define echoPinA 2
#define trigPinB 8    
#define echoPinB 11 

// Task handles
TaskHandle_t motorATask;
TaskHandle_t motorBTask;
TaskHandle_t ultrasonicATask;
TaskHandle_t ultrasonicBTask;
//TaskHandle_t motorControlTask;

// Semaphores for motor control
SemaphoreHandle_t motorASemaphore;
SemaphoreHandle_t motorBSemaphore;
SemaphoreHandle_t ultrasonicAMutex;
SemaphoreHandle_t ultrasonicBMutex;

// Queue for motor speeds
QueueHandle_t motorSpeedQueue;

// Shared variable
long distanceA = 0;
long distanceB = 0;
int motorSpeed = 0;

// Function prototypes for task functions
void motorATaskFunction(void* parameter);
void motorBTaskFunction(void* parameter);
void ultrasonicATaskFunction(void* parameter);
void ultrasonicBTaskFunction(void* parameter);
//void motorControlTaskFunction(void* parameter);

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
  ultrasonicAMutex = xSemaphoreCreateMutex();
  ultrasonicBMutex = xSemaphoreCreateMutex();

  // Create Queue for motor speeds
  motorSpeedQueue = xQueueCreate(1, sizeof(int));

  // Create tasks for each motor
  xTaskCreate(motorATaskFunction, "MotorATask", 100, NULL, 2, &motorATask);
  xTaskCreate(motorBTaskFunction, "MotorBTask", 100, NULL, 2, &motorBTask);

  // Create task for front ultrasonic sensor
  xTaskCreate(ultrasonicATaskFunction, "UltrasonicATask", 100, NULL, 1, &ultrasonicATask);

  // Create task for top ultrasonic sensor
  xTaskCreate(ultrasonicBTaskFunction, "UltrasonicBTask", 100, NULL, 3, &ultrasonicBTask);

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

void ultrasonicATaskFunction(void* parameter) {
  (void)parameter; // Unused
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);

  while (1) {
    // Trigger ultrasonic sensor
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);

    // Read the ultrasonic sensor
    long duration = pulseIn(echoPinA, HIGH);

    // Acquire mutex before updating the shared variable
    if (xSemaphoreTake(ultrasonicAMutex, portMAX_DELAY) == pdTRUE) {
      // Calculate distance in centimeters
      distanceA = duration / 58.2; // simplified form
      xSemaphoreGive(ultrasonicAMutex); // Release the mutex
    }

    Serial.print("Front Distance: ");
    Serial.print(distanceA);
    Serial.println(" cm");

    if (distanceA < 0) {
      distanceA = 0;
    }

    // Check if distance is close enough to stop motors
    else if ((distanceA >= 0) && (distanceA <= 40)) {
      motorSpeed = 0; // Speed for distance between 0 and 40 cm
      xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);
    } else {
      motorSpeed = 255 / 3; // Speed for distance > 40 cm
      xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);
      // Change priority of ultrasonicBTask to 2
      vTaskPrioritySet(ultrasonicATask, 2);
    }

    // You can add a delay here if needed
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
  }
}

void ultrasonicBTaskFunction(void* parameter) {
  (void)parameter; // Unused
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);

  while (1) {
    // Trigger top ultrasonic sensor
    digitalWrite(trigPinB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);

    // Read the top ultrasonic sensor
    long duration = pulseIn(echoPinB, HIGH);

    // Acquire mutex before updating the shared variable
    if (xSemaphoreTake(ultrasonicBMutex, portMAX_DELAY) == pdTRUE) {
      // Calculate distance in centimeters
      distanceB = duration / 58.2; // simplified form
      xSemaphoreGive(ultrasonicBMutex); // Release the mutex
    }

    Serial.print("Top Distance: ");
    Serial.print(distanceB);
    Serial.println(" cm");

    if (distanceB < 0) {
      distanceB = 0;
    }

     // Check if distance is close enough to stop motors
    else if (distanceB == 0) {
      motorSpeed = 255/3; // Speed for distance between 0 and 40 cm
      xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);

      // Change priority of ultrasonicBTask to 1
      vTaskPrioritySet(ultrasonicBTask, 1);
    } else {
      motorSpeed = 0; // Speed for distance > 40 cm
      xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);
    }

    // You can add a delay here if needed
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
  }
}


