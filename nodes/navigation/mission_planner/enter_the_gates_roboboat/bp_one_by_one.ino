#include <Servo.h>

byte servoPin_1 = 3;
byte servoPin_2 = 5;
byte servoPin_3 = 6;
byte servoPin_4 = 9;

Servo servo1; // Define a servo object
Servo servo2;
Servo servo3;
Servo servo4;

int motor_forward = 1550;
int motor_backward = 1450;
int motor_stop = 1500;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Attach the servo to the specified pin
  servo1.attach(servoPin_1);
  servo2.attach(servoPin_2);
  servo3.attach(servoPin_3);
  servo4.attach(servoPin_4);
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);
  delay(1000);

}

void runCommand(char command) {
  Serial.println("Spinning Motor 1 forward")
  servo1.write(motor_forward)
  sleep(2) //keep going for 2s
  Serial.println("Spinning Motor 1 backward")
  servo1.write(motor_backward)
  sleep(2)
  Serial.println("Stopping Motor 1")
  servo1.write(motor_stop)
  sleep(2)
  Serial.println("Spinning Motor 2 forward")
  servo2.write(motor_forward)
  sleep(2) //keep going for 2s
  Serial.println("Spinning Motor 2 backward")
  servo2.write(motor_backward)
  sleep(2)
  Serial.println("Stopping Motor 2")
  servo2.write(motor_stop)
  sleep(2)
  Serial.println("Spinning Motor 3 forward")
  servo3.write(motor_forward)
  sleep(2) //keep going for 2s
  Serial.println("Spinning Motor 3 backward")
  servo3.write(motor_backward)
  sleep(2)
  Serial.println("Stopping Motor 3")
  servo3.write(motor_stop)
  sleep(2)
  Serial.println("Spinning Motor 4 forward")
  servo4.write(motor_forward)
  sleep(2) //keep going for 2s
  Serial.println("Spinning Motor 4 backward")
  servo4.write(motor_backward)
  sleep(2)
  Serial.println("Stopping Motor 4")
  servo4.write(motor_stop)
}

void main() {
  // Check for serial commands and execute corresponding actions
  if (Serial.available()) {
    char command = Serial.read();
    runCommand(command);
  }
}
