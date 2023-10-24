import RPi.GPIO as gpio
import time
def init():    
    gpio.setmode(gpio.BCM)
    gpio.setup(17, gpio.OUT)
    gpio.setup(22, gpio.OUT)
    # gpio.setup(23, gpio.OUT)
    # gpio.setup(24, gpio.OUT)
def forward(sec):
    init()
    gpio.output(17, False)
    gpio.output(22, True)
    # gpio.output(23, True)
    # gpio.output(24, False)
    time.sleep(sec)
    gpio.cleanup() 

def reverse(sec):
    init()
    gpio.output(17, True)
    gpio.output(22, False)
    # gpio.output(23, True)
    # gpio.output(24, False)
    time.sleep(sec)
    gpio.cleanup() 


init()
while (1):
    forward(10)
    # forward(0.1)
    # reverse(0.5)
    print("end")
    exit()


