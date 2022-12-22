# Coded by Tea S.
# 2022
from random import randint
import math
from uerrno import EIO
from machine import Pin, I2C, PWM, freq, reset
import utime
import neopixel
import os
import json
from sys import exit

# Overlock pico to max cpu freq
freq(190000000)

# Hardcoded config file for if it doesnt exist on the pico
config = {"buttons": {"fireBtn": 11, "cooldownBtn": 12},
          "vibrationMotor": {"pin": 9},
          "lights": {
    'top-left': {'length': 4, 'pin': 21, 'flicker': False},
    'top-right': {'length': 4, 'pin': 20, 'flicker': False},
    'heat-left': {'length': 24, 'pin': 27, 'flicker': False},
    'heat-right': {'length': 24, 'pin': 28, 'flicker': False},
    'top-front': {'length': 5, 'pin': 18, 'flicker': False},
    'top-back': {'length': 4, 'pin': 22, 'flicker': False},
    'bottom-front': {'length': 5, 'pin': 5, 'flicker': False},
    'bottom-back': {'length': 4, 'pin': 4, 'flicker': False}},
    "colors": {
    'heating': [[255, 80, 0], [0, 0, 0]],
    'normal': [[0, 255, 150], [255, 255, 255]],
    'hot': [[225, 0, 0], [1, 1, 1]],
    'flash': [[255, 255, 255], [15, 15, 15]]},
    "servos": {"fins": {"invert": True, "start": 45, "speed": 200, "curpos": 45, "end": 125, "pin": 16}},
    "state": "Main"}


# Load config file from system (it holds all the servo settings and current position so the servos dont judder when it turns back on)

if "config" in os.listdir():
    try:
        file = open("config", "r")
        config = json.loads(file.readline())
        file.close()
    except ValueError:
        # Problem reading config file, recreate it using the above hardcoded config file
        print("Unable to read config file...Recreating")
        file = open("config", "wb")
        file.write(json.dumps(config))
        file.close()
else:
    # Create config file if it doesn't exist
    file = open("config", "wb")
    file.write(json.dumps(config))
    file.close()


def saveSettings(wherefrom=""):
    print("save - "+wherefrom)
    print(config)
    file = open("config", "wb")
    file.write(json.dumps(config))
    file.close()


print(config)
# ===========================START OF CLASSES ==================================

# Class for controlling and setting servo settings, pass in a config file with all the settings of the servo


class Servo:
    def __init__(self, config):
        self.speed = config['speed']
        self.posMin = config['start']
        self.posMax = config['end']
        self.pin = config['pin']
        self.invert = config['invert']
        self.config = config
        self.targetAngle = config['curpos']
        self.angle = config['curpos']
        self.pwm = PWM(Pin(self.pin, Pin.OUT))
        self.pwm.freq(50)

    def setPos(self, angle, immediate=False):
        if not self.isMoving():
            if round(angle) != round(self.angle):
                if angle > self.posMax:
                    angle = self.posMax
                elif angle < self.posMin:
                    angle = self.posMin
                if immediate:
                    self.angle = angle
                    self.targetAngle = angle
                    self.moveServo(angle)
                else:
                    self.targetAngle = angle
                # save angle to config file here
                self.saveCurPos()

    def setMin(self, min):
        self.posMin = min
        self.setPos(min)

    def setMax(self, max):
        self.posMax = max
        self.setPos(max)

    def setSpeed(self, speed):
        if speed != self.speed:
            self.speed = speed

    def getSpeed(self):
        return self.speed

    def setInvert(self, invert):
        self.invert = invert

    def update(self):
        if self.angle < self.targetAngle:
            self.angle += (self.speed / 100)
        elif self.angle > self.targetAngle:
            self.angle -= (self.speed / 100)
        self.moveServo(int(self.angle))

    def moveServo(self, degrees):
        # limit degrees between 0 and 180
        if degrees > 180:
            degrees = 180
        if degrees < 0:
            degrees = 0
        if self.invert:
            degrees = (180 + 0) - degrees
        # set max and min duty
        maxDuty = 9000
        minDuty = 1500
        # new duty is between min and max duty in proportion to its value
        newDuty = minDuty + (maxDuty - minDuty) * (degrees / 180)
        # servo PWM value is set
        self.pwm.duty_u16(int(newDuty))

    def isMoving(self):
        if round(self.angle) == round(self.targetAngle):
            return False
        else:
            return True

    def open(self, immediate=False):
        self.setPos(self.posMax, immediate=immediate)

    def close(self, immediate=False):
        self.setPos(self.posMin, immediate=immediate)
        # Need to improve this, only works if the servo was set using open() or close()

    def toggle(self):
        if round(self.angle) == round(self.posMax):
            self.close()
        if round(self.angle) == round(self.posMin):
            self.open()

    def saveCurPos(self):
        self.config['curpos'] = self.targetAngle
        saveSettings("savecurpos")
# Pixel class based on RGB tuples, capable of animation from one colour to another, adjusting brightness as a whole and enabling a cool flicker effect (possibly more in the future)


class Pixel:
    def __init__(self, speed, brightness, color, flicker=True):
        # Normal Flicker
        self.brightness = 0
        self.color = list(color[0])
        self.flashColor = list(color[1])

        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])

        self.randomFlicker = randint(int(1 * speed), int(20 * speed))
        self.speed = speed
        self.randomFlickerCount = 0
        self.randomFlickerOn = 0
        self.brightnessNormal = 0
        self.brightnessFlicker = 0
        self.animationSpeed = 1
        self.isFlicker = flicker
        self.animating = False
        self.setBrightness(brightness)

    def getPixelState(self):
        self.randomFlickerCount += 1
        self.randomFlickerOn -= 1
        if self.randomFlickerCount >= self.randomFlicker:
            self.randomFlicker = randint(
                int(1 * self.speed), int(20 * self.speed))
            self.randomFlickerOn = 1
            self.randomFlickerCount = 0
        if self.randomFlickerOn > 0 and self.isFlicker:
            return tuple([int(x * self.brightness / 100) for x in self.flashColor])
        else:
            return tuple([int(x * self.brightness / 100) for x in self.color])

    def update(self):
        # Handle colour animation
        if self.color != self.targetColor:
            for i in range(len(self.color)):
                if self.color[i] > self.targetColor[i]:
                    self.color[i] = self.color[i] - self.animationSpeed
                if self.color[i] < self.targetColor[i]:
                    self.color[i] = self.color[i] + self.animationSpeed
                if self.flashColor[i] > self.targetFlashColor[i]:
                    self.flashColor[i] = self.flashColor[i] - \
                        self.animationSpeed
                if self.flashColor[i] < self.targetFlashColor[i]:
                    self.flashColor[i] = self.flashColor[i] + \
                        self.animationSpeed

        # Handle random flickering effect
        return self.getPixelState()
    # Get rgb tuple for writing to neopixels

    def getRgb(self):
        return self.rgbCurrent

    def isAnimating(self):
        return self.animatng
    # Set the color to animate to

    def animateColor(self, color, speed=0):
        if speed != 0:
            self.animationSpeed = speed
        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])

    def setColor(self, color):
        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])
        self.color = list(color[0])
        self.flashColor = list(color[1])

    def setAnimationSpeed(self, speed):
        self.animationSpeed = speed

    def setBrightness(self, brightness):
        if self.brightness != brightness:
            if brightness > 100:
                brightness = 100
            elif brightness <= 0:
                brightness = 0
            self.brightness = brightness

    def setFlicker(self, flicker):
        self.isFlicker = flicker
# Class for handling neolight lighting (just holds individual pixels and updates them all the logic happens in the pixel class above)


class NeoLight:
    def __init__(self, config):
        self.config = config
        self.brightness = 0
        self.pin = config['pin']
        self.cooldown = False
        self.speed = 1
        self.length = config['length']
        self.np = neopixel.NeoPixel(Pin(config['pin']), config['length'])
        self.pixels = []
        self.color = [[225, 0, 0], [15, 15, 15]]
        self.cooldown = False
        self.flicker = config['flicker']
        for i in range(len(self.np)):
            pixel = Pixel(self.speed, self.brightness,
                          self.color, self.flicker)
            self.pixels.append(pixel)

    def update(self):
        for index, pixel in enumerate(self.pixels):
            self.np[index] = pixel.update()
        self.np.write()

    def animateColor(self, color, speed=0):
        if color != self.color:
            for pixel in self.pixels:
                if speed > 0:
                    pixel.animateColor(color, speed)
                else:
                    pixel.animateColor(color)
            self.color = color

    def setColor(self, color):
        for index, pixel in enumerate(self.pixels):
            if index < self.length:
                pixel.setColor(color)
                self.color = color

    def setColorRange(self, color, start, end):
        for index, pixel in enumerate(self.pixels):
            if index >= start and index < end:
                pixel.setColor(color)
                self.color = color

    def setPixelColor(self, color, pixel):
        self.pixels[pixel].setColor(color)

    def setBrightness(self, brightness):
        if self.brightness != brightness:
            if brightness > 100:
                brightness = 100
            elif brightness < 0:
                brightness = 0
            for index, pixel in enumerate(self.pixels):
                if index < self.length:
                    pixel.setBrightness(brightness)
            self.brightness = brightness

    def setFlicker(self, flicker):
        self.flicker = flicker
        for index, pixel in enumerate(self.pixels):
            pixel.setFlicker(flicker)

    def setLength(self, length):
        self.length = length
# Class for handling input buttons, you can set if it should fire off once or press and hold to continually output true after a predefined timeout.


class Button:
    def __init__(self, pin, single=False):
        self.single = single
        self.pin = pin
        self.lock = False
        self.button = Pin(pin, Pin.IN, Pin.PULL_DOWN)
        self.timeout = 5
        self.interval = 5
        self.count = 0
        self.held = False

    def getState(self, single=True):
        if self.button.value():
            self.held = True
            if not self.lock:
                self.lock = True
                return True
            else:
                if not single:
                    if self.count >= self.timeout:
                        self.lock = False
                        pass
                    self.count = self.count + 1
                return False
        else:
            self.lock = False
            self.count = 0
            self.held = False
            return False

    def getHeld(self):
        return self.held

    def getCount(self):
        return self.count

    def reset(self):
        self.lock = True
# Basic timer class to count down things and call a method when it reaches its goal


class Timer:
    def __init__(self, start, end, interval, functions=[]):
        self.end = end
        self.start = start
        self.current = start
        self.interval = interval
        self.functions = functions

    def update(self, do):
        if do:
            if self.interval > 0:
                if self.current <= self.end:
                    self.current = self.current + self.interval
            else:
                if self.current >= self.start:
                    self.current = self.current + self.interval
        if self.current >= self.end:
            for function in self.functions:
                if callable(function):
                    function()
            return True
        else:
            return False

    def reset(self):
        self.current = self.start

    def getState(self):
        return self.current

    def setStartEnd(self, start, end):
        self.start = start
        self.end = end


# Initialize all the buttons

fireBtn = Button(config['buttons']['fireBtn'], True)
cooldownBtn = Button(config['buttons']['cooldownBtn'], True)

vibrationMotor = Pin(config['vibrationMotor']['pin'], Pin.OUT)


# METHODS ==================================
def set_pixel_line_gradient(pixel1, pixel2, left_rgb, right_rgb, light):

    if pixel2 - pixel1 == 0:
        return
    right_pixel = max(pixel1, pixel2)
    left_pixel = min(pixel1, pixel2)

    r_diff = right_rgb[0] - left_rgb[0]
    g_diff = right_rgb[1] - left_rgb[1]
    b_diff = right_rgb[2] - left_rgb[2]
    for i in range(right_pixel - left_pixel):
        fraction = i / (right_pixel - left_pixel)
        red = round(r_diff * fraction + left_rgb[0])
        green = round(g_diff * fraction + left_rgb[1])
        blue = round(b_diff * fraction + left_rgb[2])
        light.setPixelColor([[red, green, blue], [0, 0, 0]], left_pixel + i)


def paintLines(frame):
    if frame > 0:
        if frame < 20:
            set_pixel_line_gradient(0, max(frame, 0), [255, 80-(frame*2), 0], [
                                    0, 255, 150], lights['heat-left'])
            set_pixel_line_gradient(0, max(frame, 0), [255, 80-(frame*2), 0], [
                                    0, 255, 150], lights['heat-right'])
        else:
            set_pixel_line_gradient(
                max(frame-20, 0), 24, [255, 80-(frame*2), 0], [0, 255, 150], lights['heat-left'])
            set_pixel_line_gradient(
                max(frame-20, 0), 24, [255, 80-(frame*2), 0], [0, 255, 150], lights['heat-right'])
    else:
        lights['heat-left'].setColor(config['colors']['normal'])
        lights['heat-right'].setColor(config['colors']['normal'])


def changeState(toChange):
    global state
    state = toChange


finServo = Servo(config['servos']['fins'])
lights = {}


for light in config['lights']:
    lights[light] = NeoLight(config['lights'][light])

for light in lights:
    lights[light].setBrightness(15)
    lights[light].setColor(config['colors']['normal'])


# Update servos once before continuing (sets them to the same position as in the config file)
finServo.update()


class State():
    def __init__(self, saveState=False):
        self.saveState = saveState

    def enter(self, sm):
        pass

    def update(self, sm):
        pass

    def exit(self, sm):
        pass

# ========= States ===============


class Intro2(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self, sm):
        for light in lights:
            lights[light].animateColor(config['colors']['hot'], 3)
        self.introTimer = Timer(0, 50, 1)

    def exit(self, sm):
        for light in lights:
            lights[light].setColor(config['colors']['normal'])
            lights[light].setBrightness(25)

    def update(self, sm):
        if self.introTimer.update(True):
            sm.changeState(Main())
        pass


class Intro(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self, sm):
        for light in lights:
            lights[light].animateColor(config['colors']['hot'], 3)
        self.introTimer = Timer(0, 50, 1)

    def update(self, sm):
        if self.introTimer.update(True):
            sm.changeState(Intro2())
        pass


class Firing(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self, sm):
        self.fireLength = Timer(0, 2, 1)
        self.offCooldown = Timer(0, 5, 1)
        lights['top-front'].setColor(config['colors']['flash'])
        lights['bottom-front'].setColor(config['colors']['flash'])
        lights['top-front'].setBrightness(255)
        lights['bottom-front'].setBrightness(255)
        lights['top-back'].animateColor(config['colors']['hot'], 4)

    def exit(self, sm):
        lights['top-front'].setColor(config['colors']['normal'])
        lights['bottom-front'].setColor(config['colors']['normal'])
        lights['top-front'].setBrightness(sm.normalBrightness)
        lights['bottom-front'].setBrightness(sm.normalBrightness)
        lights['top-back'].animateColor(config['colors']['normal'], 7)
        sm.heat = sm.heat + 2
        if sm.frame < 20:
            sm.frame += 4
        else:
            sm.frame += 1

    def update(self, sm):
        vibrationMotor.high()
        if self.fireLength.update(True):
            lights['top-front'].setColor(config['colors']['normal'])
            lights['bottom-front'].setColor(config['colors']['normal'])
            lights['top-front'].setBrightness(sm.normalBrightness)
            lights['bottom-front'].setBrightness(sm.normalBrightness)
            if self.offCooldown.update(True):
                sm.changeState(Main())


class Exploded(State):
    def __init__(self):
        super().__init__(saveState=True)

    def enter(self, sm):
        super().enter(sm)
        pass

    def update(self, sm):
        if cooldownBtn.getState(True):
            sm.changeState(Main())

    def exit(self, sm):
        finServo.close(immediate=True)


class Main(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        paintLines(sm.frame)
        self.cooldownCounter = Timer(0, 10, 1)
        vibrationMotor.low()

    def update(self, sm):
        # Main Loop
        if self.cooldownCounter.update(True):
            if sm.frame > 0:
                sm.frame -= 1
                paintLines(sm.frame)
        if fireBtn.getState(False):
            sm.changeState(Firing())
        if sm.frame >= 44:
            sm.changeState(OverHeated())

        # Enter display mode with fins open
        cooldownBtn.getState(False)
        if cooldownBtn.getCount() > 30:
            finServo.open(immediate=True)
            cooldownBtn.reset()
            sm.changeState(Exploded())


class OverHeated(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        for light in lights:
            lights[light].animateColor(config['colors']['hot'], 6)
            lights[light].setFlicker(True)

    def update(self, sm):
        if cooldownBtn.getState(True):
            sm.changeState(Cooldown())
        pass


class Cooldown(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        finServo.open(immediate=True)
        sm.frame = sm.frame - 1
        sm.heat = 0
        for light in lights:
            lights[light].animateColor(config['colors']['normal'], 4)
            lights[light].setFlicker(False)
        self.timer = Timer(0, 70, 1)
        self.servoCloseTimer = Timer(0, 50, 1)
        self.cooldowntimer = Timer(0, 10, 1)
        lights['heat-left'].animateColor(config['colors']['heating'], 6)
        lights['heat-right'].animateColor(config['colors']['heating'], 6)
        paintLines(sm.frame)

    def update(self, sm):
        if self.cooldowntimer.update(True):
            if sm.frame > 0:
                sm.frame -= 1
                paintLines(sm.frame)
        if self.timer.update(True):
            sm.changeState(Main())
        if self.servoCloseTimer.update(True):
            finServo.close(immediate=True)


class StateMachine():
    def __init__(self, starting):
        self.currentState = None
        # Settings
        self.normalBrightness = 15
        self.heat = 0
        self.maxHeat = 58
        self.length = 0
        self.frame = 0
        self.cb = Timer(0, 1, 1)
        self.changeState(starting)

    def changeState(self, state):
        if self.currentState:
            self.currentState.exit(self)
            # Save new state on exiting old state
            if self.currentState.saveState:
                config['state'] = type(state).__name__
                saveSettings()

        self.currentState = state

        if self.currentState:
            self.currentState.enter(self)
            if self.currentState.saveState:
                config['state'] = type(state).__name__
                saveSettings()

    def getCurrentState(self):
        return self.currentState

    def update(self):
        self.currentState.update(self)
        if finServo.isMoving():
            finServo.update()
        for light in lights:
            lights[light].update()


# Load into previously saved state:
# Only if one of the "safe" states
if config['state'] == 'Main' or config['state'] == 'Exploded':
    state = config['state']
else:
    state = 'Main'
constructor = globals()[state]

mainLogic = StateMachine(constructor())

# MAIN LOOP
loopstart = utime.time()


while True:
    loopstart = utime.ticks_cpu()
    mainLogic.update()
    delta = utime.ticks_cpu() - loopstart
