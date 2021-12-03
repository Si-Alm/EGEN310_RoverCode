import motor
import RPi.GPIO as GPIO
from time import sleep
import os, struct, array
from fcntl import ioctl
import _thread
import atexit

def exit_handler():
    print('My application is ending!')

atexit.register(exit_handler)

############# FUNCTION DEFINITIONS ###############
def right_monitor():
    while axis_states['right'] > -1:
        if check_left_stick() or check_right_stick():
            while(check_left_stick() or check_right_stick()):
                sleep(0.01)

            motor1.forward()
            motor2.forward()
        print("forward")

def left_monitor():
    while axis_states['brake'] > -1:
        if check_left_stick() or check_right_stick():
            while(check_left_stick() or check_right_stick()):
                sleep(0.1)

            motor1.backwards()
            motor2.backwards()
        print("backwards")

def right_stick_monitor():
    while check_right_stick():
        five = 0
        #print("right wheel")

def left_stick_monitor():
    while check_left_stick():
        five = 0
        #print("left wheel")

def check_right_stick():
    return(axis_states['hat0x'] > 0.05)

def check_left_stick():
    return(axis_states['hat0x'] < -0.05)

############# SET UP CONTROLLER BINDINGS ###############

# store controller input states in these guys
axis_states = {}
button_states = {}

# Bindings/names for controllers - the hex constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'right',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0



############# SET UP MOTORS ###############
ena = 14
in1 = 15
in2 = 18

enb = 11
in3 = 9
in4 = 10

motor1 = motor.Motor("Motor1", GPIO.BCM, ena, in1, in2)
motor2 = motor.Motor("Motor2", GPIO.BCM, enb, in3, in4)

############# main looping functionality ###############

# input/controller state trackers
right_down = False
left_down = False
stick_right_active = False
stick_left_active = False
halt_for_turn = False

# TODO: turning - one wheel back one forward
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)

        if type & 0x01:
            button = button_map[number]
            print(button + " clicked")
            button_states[button] = value
            if value:
                if not down:
                    print("%s pressed" % (button))
                    down = True
            else:
                print("%s released" % (button))
                down = False

        if type & 0x02:
            axis = axis_map[number]
            if axis:
                cur_axis_value = value / 32767.0
                axis_states[axis] = cur_axis_value
                # forward/backwards controlls - bound to triggers
                print(axis)
                if(axis == 'hat0x'):
                    print("val: " + str(cur_axis_value))
                if axis == 'right':
                    if right_down == False and axis_states['right'] > -1:
                        right_down = True
                        _thread.start_new_thread(right_monitor, ())
                        motor1.forward()
                        motor2.forward()
                    elif right_down == True and axis_states['right'] <= -1:
                        right_down = False
                        motor1.full_stop()
                        motor2.full_stop()
                        print("right stopped")
                if axis == 'brake':
                    if left_down == False and axis_states['brake'] > -1:
                        left_down = True
                        _thread.start_new_thread(left_monitor, ())
                        motor1.backwards()
                        motor2.backwards()
                    elif left_down == True and axis_states['brake'] <= -1:
                        left_down = False
                        motor1.full_stop()
                        motor2.full_stop()
                        print("left stopped")
                # individual wheels - bound to joysticks
                if axis == 'hat0x':
                    if check_right_stick(): #stick_right_active == False and:
                        stick_right_active = True
                        _thread.start_new_thread(right_stick_monitor, ())
                        motor1.forward()
                        motor2.backwards()
                    elif check_left_stick(): #stick_left_active == False and 
                        stick_left_active = True
                        _thread.start_new_thread(left_stick_monitor, ())
                        motor2.forward()
                        motor1.backwards()
                    elif False == check_right_stick(): #stick_right_active == True and 
                        stick_right_active = False
                        motor1.full_stop()
                        motor2.full_stop()
                        print("right stopped")
                    elif False == check_left_stick(): #stick_left_active == True and 
                        stick_left_active = False
                        motor2.full_stop()
                        motor1.full_stop()
                        print("left stopped")
                    
                    
                    """   
                        if stick_right_active == False and check_right_stick():
                            stick_right_active = True
                            _thread.start_new_thread(right_stick_monitor, ())
                            motor1.forward()
                            motor2.backwards()
                        elif stick_right_active == True and False == check_right_stick():
                            stick_right_active = False
                            motor1.full_stop()
                            motor2.full_stop()
                            print("right stopped")
                    if axis == 'x' or axis == 'y':
                        if stick_left_active == False and check_left_stick():
                            stick_left_active = True
                            _thread.start_new_thread(left_stick_monitor, ())
                            motor2.forward()
                            motor1.backwards()
                        elif stick_left_active == True and False == check_left_stick():
                            stick_left_active = False
                            motor2.full_stop()
                            motor1.full_stop()
                            print("left stopped")
                        """