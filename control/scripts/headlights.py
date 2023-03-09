import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

GPIO.setup(17,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(27,GPIO.OUT,initial=GPIO.LOW)

fig, ax = plt.subplots()

def on(event):
    print("on")
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(27,GPIO.HIGH)
    return

def off(event):
    print("off")
    GPIO.output(17,GPIO.LOW)
    GPIO.output(27,GPIO.LOW)
    return

def close(event):
    print("close")
    GPIO.cleanup()
    exit()

axon = fig.add_axes([0.5, 0.05, 0.1, 0.075])
axoff = fig.add_axes([0.7, 0.05, 0.1, 0.075])
axclose = fig.add_axes([0.9, 0.05, 0.1, 0.075])
bon = Button(axon, 'on')
bon.on_clicked(on)
boff = Button(axoff, 'off')
boff.on_clicked(off)
bclose = Button(axclose, 'close')
bclose.on_clicked(close)

plt.show()