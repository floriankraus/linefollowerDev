import time
import RPi.GPIO as GPIO
from Command import COMMAND as cmd
GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin,GPIO.OUT)
class Buzzer:
    def run(self,command):
        if command!="0":
            GPIO.output(Buzzer_Pin,True)
        else:
            GPIO.output(Buzzer_Pin,False)
if __name__=='__main__':
    try:
        B = Buzzer()
        B.run('1')
        time.sleep(3)
        B.run('0')
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        B.run('0')






