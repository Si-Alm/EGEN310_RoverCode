import RPi.GPIO as GPIO

class Motor:
    HIGH_VALUE = 75
    MED_VALUE = 50
    LOW_VALUE = 0
    def __init__(self, name, pin_mode, en, in_low, in_high):
        self.name = name
        self.en = en
        self.in_low = in_low
        self.in_high = in_high
        
        # set gpio mode
        GPIO.setmode(GPIO.BCM)

        # setup pins
        GPIO.setup(in_low, GPIO.OUT)
        GPIO.setup(in_high, GPIO.OUT)
        GPIO.setup(en, GPIO.OUT)

        # set everything to low
        GPIO.output(in_low, GPIO.LOW)
        GPIO.output(in_high, GPIO.LOW)

        self.pwm = GPIO.PWM(en, 100)
        self.pwm.start(25)
        self.pwm.ChangeDutyCycle(0)


    def full_stop(self):
        GPIO.output(self.in_low, GPIO.LOW)
        GPIO.output(self.in_high, GPIO.LOW)
        self.pwm.ChangeDutyCycle(self.LOW_VALUE)

    def forward(self):
        GPIO.output(self.in_low, GPIO.HIGH)
        GPIO.output(self.in_high, GPIO.LOW)
        self.pwm.ChangeDutyCycle(self.HIGH_VALUE)

    def backwards(self):
        GPIO.output(self.in_low, GPIO.LOW)
        GPIO.output(self.in_high, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(self.HIGH_VALUE)

