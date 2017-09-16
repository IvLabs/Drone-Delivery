#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#define TRUE 1

#define TRIG 1 //tx gpio 0.1
#define ECHO 0 //rx gpio 0.0

void setup() {
        wiringPiSetup();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);

        //TRIG pin must start LOW
        digitalWrite(TRIG, LOW);
        delay(30);
}


int main(void) {
        setup();

while(1)
{
 //Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);

        //Wait for echo start
        while(digitalRead(ECHO) == LOW);

        //Wait for echo end
        long startTime = micros();
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;

        //Get distance in cm
        int distance = travelTime / 58;
        printf("Distance: %dcm\n", distance);
}

        return 0;
}
