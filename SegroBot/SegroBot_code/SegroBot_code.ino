#include <DFRobot_SIM808.h>
#include <sim808.h>

#include <DFRobot_SIM808.h>
#include <sim808.h>

#include <CapacitiveSensor.h>
#include <AFMotor.h>
#include <Servo.h> 
#include <DFRobot_SIM808.h>

//PIR and Capacitive Sensor
int pirPin = 22;    // PIR sensor input pin
int capPin = 9;    // Capacitive sensor output pin
int numReadings = 5; // Number of readings to take before deactivating

//Servo and Stepper Motors
const int STEPS_PER_REVOLUTION = 971; // Steps per revolution of stepper motor
const int STEPPER_SPEED = 100; // Stepper motor speed
Servo servo;	// create servo object to control a servo
int pos = 0;

//Ultrasonic Sensor
const int trigPins[] = {34, 42, 50};  // array of trig pins for each sensor
const int echoPins[] = {35, 43, 51};  // array of echo pins for each sensor
const int DELAY_TIME = 2000;       // time delay between each sensor reading

long duration, distance;           // variables to store the sonar readings


CapacitiveSensor capSensor = CapacitiveSensor(0, capPin); // The first parameter (0) is a dummy value since we are not using a send pin
//Create stepper object
AF_Stepper stepper(200, 1); // (Resolution of steps per revolution of stepper motor, Stepper motor number on L293D shield)


// //sim808 sms
// /**
//  * Besides push-in connection with expansion board, it can also be connected by jump wires
//  * Set DIP switch to 3-Arduino, open the following macro
//  * Connect the main controller to the module with Dupont wires:
//  *  Arduino | module
//  *   PIN_TX |  TX1
//  *   PIN_RX |  RX1
//  * Power the module, which is successful when the power indicator on the module is ON
//  */
// // #define CONNECT_BY_JUMPER   1
// #if CONNECT_BY_JUMPER
//   #define PIN_TX    10
//   #define PIN_RX    11
//   SoftwareSerial mySerial(PIN_TX, PIN_RX);
//   DFRobot_SIM808 sim808(&mySerial);
// /**
//  * Use Leonardo for push-in connection
//  * Set DIP switch to 3-Arduino, and use the Serial1 of Leonardo for communication
//  */
// #elif defined(ARDUINO_AVR_LEONARDO)
//   DFRobot_SIM808 sim808(&Serial1);
// /**
//  * Use UNO & MEGA2560 for push-in connection
//  * Set DIP switch to 3-Arduino, and use the Serial of UNO and MEGA2560 for communication
//  */
// #else
//   DFRobot_SIM808 sim808(&Serial);
// #endif

// //Mobile phone number,need to change
// #define PHONE_NUMBER  "09453729005"

// //The content of messages sent
// // #define MESSAGE  "Bin capacity is now 80%"

void setup() {
  Serial.begin(9600);       // Initialize Serial Monitor
  pinMode(pirPin, INPUT);   // Set PIR sensor pin as input
  pinMode(capPin, OUTPUT);  // Set capacitive sensor pin as output
  for (int i = 0; i < 3; i++) {    // loop through each sensor
    pinMode(trigPins[i], OUTPUT);  // set trig pin as output
    pinMode(echoPins[i], INPUT);   // set echo pin as input
  }
  servo.attach(10); // Attach the servo to pin 10

  //SIM808
  //  #if CONNECT_BY_JUMPER
  //   mySerial.begin(9600);
  // #elif defined(ARDUINO_AVR_LEONARDO)
  //   Serial1.begin(9600);
  // #endif
  // Serial.begin(9600);

  // //******** Initialize sim808 module *************
  // while(!sim808.init()) {
  //     delay(1000);
  //     Serial.print("Sim808 init error\r\n");
  // }  
  // Serial.println("Sim808 init success");
  // Serial.println("Start to send message ...");

}

void loop() {

  servo.write(0);
  delay(15);

  int pirValue = digitalRead(pirPin); // Read PIR sensor value
  
  if (pirValue == HIGH) { // If motion detected
    Serial.println("Motion detected!"); // Print message to Serial Monitor

    long total = 0;

    for (int i = 0; i < numReadings; i++) { // Take numReadings readings
      digitalWrite(capPin, LOW); // Turn on capacitive sensor
      delay(100); // Wait for a short time to allow the sensor to stabilize
      long sensorValue = capSensor.capacitiveSensor(30); // Read the value from the sensor, using a threshold of 30
      Serial.print("Capacitive sensor value: "); // Print message to Serial Monitor
      Serial.println(sensorValue); // Print capacitive sensor value to Serial Monitor
      total += sensorValue; // Add current reading to total
      delay(100); // Wait for a short time before taking the next reading
    }

    digitalWrite(capPin, HIGH); // Turn off capacitive sensor
    int average = total / numReadings; // Calculate average
    Serial.print("Average capacitive sensor value: "); // Print message to Serial Monitor
    Serial.println(average); // Print average to Serial Monitor

    //Stepper and Servo Motor controls
    if (average == -2) {  //Moves compartment to the right for Metals or Liquids
      Serial.println("Metal or Liquid Detected!");
      stepper.setSpeed(STEPPER_SPEED); // Set stepper motor speed
      stepper.step(STEPS_PER_REVOLUTION, FORWARD, DOUBLE); // Rotate stepper motor 1 full revolution (clockwise)
      // opens then closes the servo
      servo.write(35);
      delay(1500);
      servo.write(0);
      delay(1000);
      stepper.step(STEPS_PER_REVOLUTION, BACKWARD, DOUBLE); // Rotate stepper motor 1 full revolution (counter-clockwise)      
    }else if (average >= 0 && average <= 6000) {  //Moves compartment to the left for Plastic
      Serial.println("Plastic Detected");
      stepper.setSpeed(STEPPER_SPEED); // Set stepper motor speed
      stepper.step(STEPS_PER_REVOLUTION, BACKWARD, DOUBLE); // Rotate stepper motor 1 full revolution (counter-clockwise)
      // opens then closes the servo
      servo.write(35);
      delay(1500);
      servo.write(0);
      delay(1000);
      stepper.step(STEPS_PER_REVOLUTION, FORWARD, DOUBLE); // Rotate stepper motor 1 full revolution (clockwise)
    }else if (average > 6000) {
      Serial.println("Paper Detected");
      // opens then closes the servo
      servo.write(35);
      delay(1500);
      servo.write(0);
      delay(1000);
    }

    //Bin fill percentage
    for (int i = 0; i < 3; i++) {    // loop through each sensor
    SonarSensor(trigPins[i], echoPins[i]);  // read distance from the sensor
    int percentage = map(distance, 48, 4, 0, 100);  // map distance to percentage (14 = 0%, 4 = 100%)
    if (percentage < 0) percentage = 0;  // make sure percentage is within 0-100 range
    else if (percentage > 100) percentage = 100;
    Serial.print(i == 0 ? "RightSensor: " : i == 1 ? "MiddleSensor: " : "LeftSensor: ");  // print sensor name based on index
    Serial.print(percentage);         // print percentage value
    Serial.println("%");              // print percentage sign

    //sms if bin is 80%
    if(percentage >= 80){ 
        sendSMS(getGPS());
        // sim808.sendSMS((char *)PHONE_NUMBER, (char *)MESSAGE); 
        // delay(10000); // Wait 10 seconds before sending the next message
      }    
    }

    delay(5000); // Wait for 5 seconds before checking for motion again
  }
}

void SonarSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);            // set trig pin to low for 2 microseconds
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);           // set trig pin to high for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);            // set trig pin back to low
  duration = pulseIn(echoPin, HIGH);     // read duration of sound wave from echo pin
  distance = duration * 0.034 / 2;       // calculate distance in centimeters
}




