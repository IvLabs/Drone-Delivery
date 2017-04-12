#include <Servo.h>


 Servo Throttle;
 Servo Rudder;
 Servo Aileron;
 Servo Elevator;
 Servo Auxiliary;
   
void setup() {
  Throttle.attach(9);
  Rudder.attach(10);
  Aileron.attach(5);
  Elevator.attach(6);
  Auxiliary.attach(11);
}

void loop() {
  
 Throttle.write(0);
  Rudder.write(800);// this is th least value which output a right vlaue for rudder
  delay(7000);
  Rudder.write(2000);// this is th  value which output a right vlaue for rudder
  delay(7000);
  
}
