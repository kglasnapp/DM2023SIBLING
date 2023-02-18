# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# You must add a gamepad HID device inside your boot.py file and reboot the macropad
# in order to use this example.
# See this Learn Guide for details:
# https://learn.adafruit.com/customizing-usb-devices-in-circuitpython/hid-devices#custom-hid-devices-3096614-9


# Add I2C Scan -- https://learn.adafruit.com/adafruit-macropad-rp2040?view=all and serach for I2C

import board
import digitalio
import analogio
from microcontroller import reset
import usb_hid
import time
import os
import displayio
import terminalio

from adafruit_display_shapes.rect import Rect
from adafruit_display_text import label

from adafruit_macropad import MacroPad
import supervisor

from hid_gamepad import Gamepad

from rainbowio import colorwheel
from adafruit_seesaw import seesaw, neopixel, rotaryio, digitalio


# Equivalent of Arduino's map() function.
RED = 0xFF0000
GREEN = 0x00FF00
BLUE = 0x0000FF
WHITE = 0xFFFFFF
YELLOW = 0xFFFF00
PURPLE = 0x9500ff
NOLIGHT = 0x000000

lastPos = 0
print("Start Robot Adjuster")
group = displayio.Group()
pad = MacroPad()
pad.display.auto_refresh = False
pad.pixels.auto_write = False
count = 0
accum = "0"
increment = 1.0
lastEncoderSwitch = 0

#seesaw = seesaw.Seesaw(board.I2C(), 0x36)

#encoder = rotaryio.IncrementalEncoder(seesaw)
#seesaw.pin_mode(24, seesaw.INPUT_PULLUP)

#switch = digitalio.DigitalIO(seesaw, 24)

#pixel = neopixel.NeoPixel(seesaw, 6, 1)
#pixel.brightness = 0.5
button_held = False
color = 0


def range_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def displayOld(header, buttons):
    textLines = pad.display_text(title=header)
    for i in range(4):
        textLines[i].text = "%7s %7s %7s" % ("Keith", "Buddy", "Susan")
    textLines.show()


def setupDisplay():
    # Set up displayio group with all the labels
    for key_index in range(12):
        x = key_index % 3
        y = key_index // 3
        group.append(label.Label(terminalio.FONT, text='', color=0xFFFFFF,
                                 anchored_position=(
                                     (pad.display.width - 1) * x / 2, pad.display.height - 1 - (3 - y) * 12),
                                 anchor_point=(x / 2, 1.0)))
    group.append(Rect(0, 0, pad.display.width, 12, fill=0xFFFFFF))
    group.append(label.Label(terminalio.FONT, text='', color=0x000000,
                             anchored_position=(pad.display.width//2, -2), anchor_point=(0.5, 0.0)))
    pad.display.show(group)


def display(header, buttons, colors):
    group[13].text = header   # Application name
    for i in range(12):
        group[i].text = buttons[i]
        pad.pixels[i] = colors[i]
    pad.stop_tone()
    pad.pixels.show()
    pad.display.refresh()


def keyPress(event):
    global accum
    keyNumber = event.key_number
    if event.pressed:
        pad.pixels[keyNumber] = WHITE
        key = mode["text"][keyNumber]
        accum = str(accum) + str(key)
        try:
            float(accum)
        except:
            accum = accum[:-1]
        group[13].text = str(float(accum))
    else:
        pad.pixels[keyNumber] = testColors[keyNumber]
    pad.pixels.show()
    pad.display.refresh()


def enterPress(event):
    global accum
 
    keyNumber = event.key_number
    if event.pressed:
        pad.pixels[keyNumber] = WHITE
        send("enter", "", accum)
        group[13].text = mode["text"][keyNumber] + " " + str(float(str(accum)))
        accum = 0
    else:
        pad.pixels[keyNumber] = testColors[keyNumber]
    pad.pixels.show()
    pad.display.refresh()


def updateGamePad(event):
    keyNumber = event.key_number
    if event.pressed:
        gp.press_buttons(keyNumber+1)
        pad.pixels[keyNumber] = WHITE
        group[13].text = "Last:" + mode["text"][keyNumber]
        send("gamePad", mode["text"][keyNumber], "")
    else:
        gp.release_buttons(keyNumber+1)
        pad.pixels[keyNumber] = testColors[keyNumber]
    pad.pixels.show()
    pad.display.refresh()


def send(cmd, value, number):
    s = '{"command":"%s","value":"%s","number":"%s"}' % (cmd, value, number)
    print(s)


def handlePID(event):
    global increment, accum
    keyNumber = event.key_number
    text = mode["text"][keyNumber]
    cmd = mode["commands"][keyNumber]
    subCmd = mode["subCommands"][keyNumber]
    if accum == '':
        accum = 0
    if event.pressed:
        pad.pixels[keyNumber] = WHITE
        if cmd == "put":
            send(cmd, subCmd, accum)
            group[13].text = "%s%s %f" % (cmd,subCmd,float(accum))
        if cmd == "get":
            send(cmd, subCmd, accum)
            group[13].text = "%s%s %f" % (cmd,subCmd,float(accum))
        if cmd == "act":
            send(cmd, subCmd, accum)
            if subCmd == '<':
                increment *= 10
            if subCmd == '>':
                increment *= .1
            if subCmd == 'clear':
                increment = 1.0
            group[13].text = "%f %f" % (float(accum), float(increment))
    else:
        pad.pixels[keyNumber] = testColors[keyNumber]
    pad.pixels.show()
    pad.display.refresh()

def handleI2C(event):
    keyNumber = event.key_number
    text = mode["text"][keyNumber]
    cmd = mode["commands"][keyNumber]


def updateAccum(position):
    global accum, increment
    accum = str(float(accum) + float(increment))
    group[13].text = "%f %f" % (float(accum), float(increment))
    pad.display.refresh()


def updateI2C(evevt):
    result = scanI2C()
    for i in range(12):
        if i != 0: 
            group[i].text = ""
            mode["text"][i] = ""
    for i in range(len(result)):
        print(result[i])
        mode["commands"][i+1] = result[i]
        if result[i] == "0x36":
            result[i] = "Encoder"
        if result[i] == "0x29":
            result[i] = "TOF"
        group[i+1].text = result[i]        
    pad.display.refresh()

    
def scanI2C():
    i2c = board.I2C()
    result = []
    while not i2c.try_lock():
        pass
    try:
        for address in i2c.scan():
            #print("I2C addresses found:", hex(address))
            result.append(hex(address))
    finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
        i2c.unlock()
        return result
last_position =  0


def showEncoder():
    global button_held, last_position, color
    print("Start Secondary Encoder")
    while True:
        # if not switch.value and not button_held:
        #     button_held = True
        #     print("Button pressed")
        # if switch.value and button_held:
        #     button_held = False
        #     print("Button released")

        # negate the position to make clockwise rotation positive
        event = pad.keys.events.get()
        if event:
            if event.key_number == 0:
                pixel.brightness = 0
                pixel.fill(colorwheel(0))
                group[13].text = "I2C"
                pad.display.refresh() 
                break
        position = -encoder.position

        if position != last_position:
            print(position)
            group[13].text = "Enc:" + str(position)
            pad.display.refresh() 


        if switch.value:
            # Change the LED color.
            if position > last_position:  # Advance forward through the colorwheel.
                color += 1
            else:
                color -= 1  # Advance backward through the colorwheel.
            color = (color + 256) % 256  # wrap around to 0-256
            pixel.fill(colorwheel(color))
            if button_held:
                print("Released", color)
                button_held = False
            

        else:  # If the button is pressed...
            # ...change the brightness.
            if position > last_position:  # Increase the brightness.
                pixel.brightness = min(1.0, pixel.brightness + 0.1)
            else:  # Decrease the brightness.
                pixel.brightness = max(0, pixel.brightness - 0.1)
            if not button_held:
                print("Pressed")
                button_held = True
        pixel.brightness = 0.5
        pixel.fill(colorwheel(color))
        last_position = position
   
def robotModeEvent(event):
    print(event) 
    
    keyNumber = event.key_number
    if event.pressed:
        gp.press_buttons(keyNumber+1)
        pad.pixels[keyNumber] = WHITE
    else:
        gp.release_buttons(keyNumber+1)
        pad.pixels[keyNumber] = robotColors[keyNumber]
    pad.pixels.show()
    pad.display.refresh()

    if event.key_number == 9:
        if event.pressed:
            group[13].text = "left grid"
            pad.pixels[10] = BLUE
            pad.pixels[11] = BLUE

    if event.key_number == 10:
        if event.pressed:
            group[13].text = "middle grid"
            pad.pixels[9] = GREEN
            pad.pixels[11] = GREEN
    if event.key_number == 11:
        if event.pressed:
            group[13].text = "right grid"
            
            pad.pixels[9] = RED
            pad.pixels[10] = RED
        
            
    if (event.key_number >=0 and event.key_number <= 8):
        print("betwen 0-8")
        if event.pressed:
            group[13].text = "Robot!"
            pad.pixels[9] = BLUE
            pad.pixels[10] = GREEN
            pad.pixels[11] = RED
    pad.pixels.show()
    pad.display.refresh()



def doI2CTask(event):
    keyNumber = event.key_number
    dev =  mode['commands'][keyNumber]
    print(keyNumber,dev)
    if dev == "0x36":
        print("Encoder")
        showEncoder()
    if dev == "0x29":
        print("TOF")

def updateJoy():
      lastPos = position
      if lastPos > 127:
         lastPos = 127
      if lastPos < -127:
         lastPos = -127
      gp.move_joysticks(lastPos, lastPos, lastPos, lastPos)

testColors = [RED, GREEN, BLUE, RED, GREEN,
              BLUE, RED, GREEN, BLUE, RED, GREEN, BLUE]

robotColors = [YELLOW, PURPLE, YELLOW,
             YELLOW, PURPLE, YELLOW, 
             NOLIGHT, NOLIGHT, NOLIGHT, 
             BLUE, GREEN, RED]
             





keyPadMode = {
    "name": "KeyPad",
    "text": ['7', '8', '9', '4', '5', '6', '1', '2', '3', '.', '0', 'Enter'],
    "commands": ["kp", "kp", "kp", "kp", "kp", "kp", "kp", "kp", "kp", "kp", "kp", "enter"],
    "colors": testColors
}
robotMode ={
    "name": "Robot!",
    "text": ["top ", "middle ", "right ", "", "", "", "", "", "", "", "", ""],
    "commands": ["rm", "rm", "rm", "rm", "rm", "rm", "rm", "rm", "rm", "rm", "rm", "rm"],
    "colors": robotColors
}

autoMode ={
    "name": "AUTO",
    "text": ["1 ", "2 ", "3 ", "", "", "", "", "", "", "", "", ""],
    "commands": ["am ", "am", "am", "am", "am", "am", "am", "am", "am", "am", "am", "am"],
    "colors": testColors
}

pidMode = {
    "name": "PID",
    "text": ["Put P", "Put I", "Put D", "Get P", "Get I", "Get D", "<", ">", "Clear", "", "", ""],
    "commands": ["put", "put", "put", "get", "get", "get", "act", "act", "act", "", "", ""],
    "subCommands": ["p", "i", "d", "p", "i", "d", "<", ">", "clear", "", "", ""],
    "colors": testColors,
}

gamePadMode = {
    "name": "Game PAD",
    "text": ["Tel UP", "Tel DN", "", "In", "Out", "", "Sp:1", "Sp:2", "Sp:3", "", "", ""],
    "commands": ["gp", "gp", "gp", "gp", "gp", "gp", "gp", "gp", "gp", "gp", "gp", "gp"],
    "colors": testColors
}

I2CMode = {
    "name": "I2C",
    "text": ["scan", "", "", "", "", "", "", "", "", "", "", ""],
    "commands": ["scan", "", "", "", "", "", "", "", "", "", "", ""],
    "subCommands": ["p", "i", "d", "p", "i", "d", "<", ">", "clear", "", "", ""],
    "colors": testColors,
}

currentMode = 0
modes = [robotMode, autoMode, pidMode, gamePadMode, I2CMode, keyPadMode]
gp = Gamepad(usb_hid.devices)
setupDisplay()
mode = modes[currentMode]
display("Robot!", mode["text"], robotColors)
scanI2C()
while True:
    # Update the value of the accum based upon the encoder
    position = pad.encoder
    if position != lastPos:
        if mode['name'] == "PID":
            updateAccum(position - last_position)
        last_Position = position

    # If encoder press switch modes
    pad.encoder_switch_debounced.update()
    encoderSwitch = pad.encoder_switch_debounced.pressed
    if encoderSwitch:
        currentMode += 1
        if(currentMode >= len(modes)):
            currentMode = 0
        mode = modes[currentMode]
        display(mode["name"], mode["text"], mode["colors"])
    # Handle key press events
    event = pad.keys.events.get()
    if event:
        keyCmd = mode["commands"][event.key_number]
        if(keyCmd == "kp"):
            keyPress(event)
        if(keyCmd == "enter"):
            enterPress(event)
        if (keyCmd == "rm"):
            robotModeEvent(event)
        if(keyCmd == "gp"):
            updateGamePad(event)
        if(keyCmd == "get" or keyCmd == "put" or keyCmd == "act"):
            handlePID(event)
        if keyCmd == "scan" and event.pressed:
            updateI2C(event)
        if keyCmd == "show" and event.pressed:
            handleI2C(event)
        if keyCmd[0:2] == "0x"and event.pressed:
            doI2CTask(event)

    # Process any serial data
    if supervisor.runtime.serial_bytes_available:
        s = input()
        print("Serial Date:", s)
    time.sleep(.05)
    count += 1
