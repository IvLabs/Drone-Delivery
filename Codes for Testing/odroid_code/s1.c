
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include </home/odroid/wiringPi/wiringPi/wiringPi.h>
#include </home/odroid/wiringPi/wiringPi/softServo.h>
#include </home/odroid/wiringPi/wiringPi/softServo.c>
int main ()
{
  if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "oops: %s\n", strerror (errno)) ;
    return 1 ;
  }

  softServoSetup (0, 1, 2, 3, 4, 5, 6, 7) ;// 1==173 , 0==174
//softServoSetup (1) ;
 //oftServoWrite (0,  -250);
/*
  softServoWrite (1, 1000) ;
  softServoWrite (2, 1100) ;
  softServoWrite (3, 1200) ;
  softServoWrite (4, 1300) ;
  softServoWrite (5, 1400) ;
  softServoWrite (6, 1500) ;
  softServoWrite (7, 2200) ;
*/
  //softServoWrite (1,  2000) ;
   //delay(8000);
 softServoWrite (0,  -400) ;
softServoWrite (1,  -400) ;
   delay(6000);
  softServoWrite (0,  400) ;
 softServoWrite (1,  400) ;
   delay(6000);
//softServoWrite (0,  800) ;
  // delay(5000);
}
