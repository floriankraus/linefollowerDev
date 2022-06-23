from control.Motor import *
import RPi.GPIO as GPIO


class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

        self.max_speed = 4000
        self.min_speed = -4000
        self.set_speed = 1500


    def constrain(self, value):
        return min(self.max_speed, max(self.min_speed, value))


    def setspeed(self, adjust):
        print(int(self.constrain(self.set_speed + adjust)), int(self.constrain(self.set_speed - adjust)),
        int(self.constrain(self.set_speed - adjust)), int(self.constrain(self.set_speed - adjust)))

        PWM.setMotorModel(int(self.constrain(self.set_speed + adjust)), int(self.constrain(self.set_speed + adjust)),
                          int(self.constrain(self.set_speed - adjust)), int(self.constrain(self.set_speed - adjust)))


    def getIR(self):
        LMR = 0x00
        if GPIO.input(self.IR01):
            LMR = (LMR | 4)
        if GPIO.input(self.IR02):
            LMR = (LMR | 2)
        if GPIO.input(self.IR03):
            LMR = (LMR | 1)

        if LMR == 0:
            LMR = 0
        elif LMR == 1:
            LMR = 5
        elif LMR == 2:
            LMR = 3
        elif LMR == 3:
            LMR = 4
        elif LMR == 4:
            LMR = 1
        elif LMR == 5:
            LMR = 6
        elif LMR == 6:
            LMR = 2
        elif LMR == 7:
            LMR = 7

        return LMR


    def runPID(self):
        print("Started PID")

        # Initialize PID variables
        P = 0
        I = 0
        D = 0
        lasterror = 0
        adjust = 0

        set_point = 3
        max_error = 2

        Kp = self.max_speed * 2 / max_error
        Ki = 0.5
        Kd = 100

        while True:
            position = self.getIR()

            if position == 0:
                adjust = 0
            elif position == 6:
                adjust = 0
            elif position == 7:
                adjust = adjust
            else:
                # PID calculations
                error = position - set_point
                P = error * Kp
                I = (I + error) * Ki
                D = (error - lasterror) * Kd
                adjust = P + I + D
                lasterror = error

                print(P, I, D, adjust)
                #self.setSpeed(adjust)


    def runPID2(self):
        print("Started PID")

        # Initialize PID variables
        P = 0
        I = 0
        D = 0
        lastError = 0
        adjust = 0

        set_point = 3
        max_error = 2

        Kp = 800
        Kd = 100
        Ki = 2

        while True:
            position = self.getIR()

            if position == 0:
                pass
            elif position == 6:
                pass
            elif position == 7:
                PWM.setMotorModel(0, 0, 0, 0)
            else:
                # PID calculations
                start = time.time()
                end = time.time()
                elapsed_time = (end - start) * 1000000

                error = position - set_point

                P = error * Kp
                I = (I + error * elapsed_time)
                D = (error - lastError) * Kd

                motorSpeed = P + (I * Ki) + D

                lastError = error

                #print(elapsed_time)
                self.setspeed(motorSpeed)

    def run(self):
        print('Started')
        while True:
            self.LMR = 0x00
            if GPIO.input(self.IR01):
                self.LMR = (self.LMR | 4)
            if GPIO.input(self.IR02):
                self.LMR = (self.LMR | 2)
            if GPIO.input(self.IR03):
                self.LMR = (self.LMR | 1)
            if self.LMR == 2:
                PWM.setMotorModel(800, 800, 800, 800)
            elif self.LMR == 4:
                PWM.setMotorModel(-1500, -1500, 2500, 2500)
            elif self.LMR == 6:
                PWM.setMotorModel(-2000, -2000, 4000, 4000)
            elif self.LMR == 1:
                PWM.setMotorModel(2500, 2500, -1500, -1500)
            elif self.LMR == 3:
                PWM.setMotorModel(4000, 4000, -2000, -2000)
            elif self.LMR == 7:
                # pass
                PWM.setMotorModel(0, 0, 0, 0)


infrared = Line_Tracking()
# Main program logic follows:
if __name__ == '__main__':
    try:
        # infrared.run()
        PWM.setMotorModel(0, 0, 0, 0)
        while True:
            infrared.runPID2()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0, 0, 0, 0)
