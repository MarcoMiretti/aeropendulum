import time
import serial
import matplotlib.pyplot as plt
import numpy as np

"""
writes in_str to bluetooth
"""
def bt_write( in_str ):
    for i in range(32-len(in_str)):
        in_str += '\0'
    ser.write(in_str)
    ser.flushOutput()

"""
reads variable value from bluetooth
variable = string
returns = float
"""
def bt_get( variable ):
    rval = '0.0'
    bt_write('p '+variable)
    rval = ser.read(8)
    rval = rval.replace('\x00','')
    if rval == '':
        rval = '0.0'
    return float(rval)
    
"""
sets a variable value through bluetooth
variable = string
value = float, int
"""
def bt_set( variable, value ):
    bt_write('s '+variable+' '+str(float(value)))
    print 's '+variable+' '+str(float(value))

print 'port number:'
input_port=raw_input()
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/rfcomm'+input_port,
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.2
    )
ser.isOpen()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

input=1
while 1 :
    # get keyboard input
    input = raw_input(">> ")
    if input == 'exit':
        ser.close()
        exit()
    if input == 'caracterize':
        
        motorPowers = []
        angles = []
        
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(angles, motorPowers, 'b*')
        plt.xlim(0.3, 1.6)
        plt.ylim(2.15, 2.3)
        
        bt_set('mode',0)
        print 'PC mode selected---'
        time.sleep(0.5)
        bt_set('onOff',1)
        print 'Aero ON---'
        time.sleep(2)
        motorPower = 2.15
        i = 0
        last_angle = 0
        while(last_angle<3.14159265/2):
            new_angle = bt_get('angle')
            while (new_angle < last_angle*0.95) or new_angle > last_angle*1.05:
                last_angle = new_angle
                print 'waiting for steady state'
                time.sleep(1)
            motorPowers.append(bt_get('motorPower'))
            angles.append(new_angle)
            motorPower += 0.001
            bt_set('motorPower',motorPower)
            last_angle = angles[-1]
            i += 1
            print 'Iteration '+str(i)
            line1.set_xdata(angles)
            line1.set_ydata(motorPowers)
            plt.draw()
            plt.pause(1e-17)
            time.sleep(1)
    if input == 'read':
        out = ''
        out += ser.read(ser.in_waiting)
        if out != '':
            print ">>" + out
    if input[0] == 'p':
        bt_write('p motorPower')
        while ser.in_waiting < 8:
            time.sleep(0.01)
        out = ''
        out += ser.read(ser.in_waiting)
        ser.flushInput()
        if out != '':
            print '>>'+out
    else:
        bt_write(input)
