import Adafruit_BBIO.PWM as PWM
throttle="P9_14"
rudder="P9_21"
aileron="P8_13"
elevator="P9_42"
#auxiliary="P8_13"

PWM.start(throttle,1,50)
PWM.start(rudder,1,50)
PWM.start("P8_13",1,50)
PWM.start(elevator,1,50)
#PWM.start(auxiliary,1,50)

PWM.set_duty_cycle(throttle,0)
PWM.set_duty_cycle(rudder,0)
PWM.set_duty_cycle(aileron,0)
PWM.set_duty_cycle(elevator,0)
#PWM.set_duty_cycle(auxiliary,0.02)

while(1):
        {
        PWM.set_duty_cycle(rudder,5)
        }
