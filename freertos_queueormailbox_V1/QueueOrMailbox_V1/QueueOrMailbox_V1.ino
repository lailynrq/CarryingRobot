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
#define trigPin 3
#define echoPin 2

// Task handles
TaskHandle_t motorATask;
TaskHandle_t motorBTask;
TaskHandle_t ultrasonicTask;

// Semaphores for motor control
SemaphoreHandle_t motorASemaphore;
SemaphoreHandle_t motorBSemaphore;
SemaphoreHandle_t ultrasonicMutex;

// Queue for motor speeds
QueueHandle_t motorSpeedQueue;

// Shared variable
long distance = 0;

// Function prototypes for task functions
void motorATaskFunction(void* parameter);
void motorBTaskFunction(void* parameter);
void ultrasonicTaskFunction(void* parameter);

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

  // Create task for ultrasonic sensor
  xTaskCreate(ultrasonicTaskFunction, "UltrasonicTask", 100, NULL, 2, &ultrasonicTask);

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
  (void)parameter; // Unused
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
      // Calculate distance in centimeters
      distance = duration / 58.2; // simplified form
      xSemaphoreGive(ultrasonicMutex); // Release the mutex
    }

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < 0) {
      distance = 0;
    }

    // Check if distance is close enough to stop motors
    else if ((distance >= 0) && (distance <= 40)) {
      int motorSpeed = 0; // Speed for distance between 0 and 40 cm
      xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);
    } else {
      int motorSpeed = 255 / 4; // Speed for distance > 40 cm
      xQueueSendToBack(motorSpeedQueue, &motorSpeed, portMAX_DELAY);
    }

    // You can add a delay here if needed
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
  }
}