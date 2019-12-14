import time
import serial
import matplotlib.pyplot as plt
import numpy as np
import datetime
import csv


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

in_data = [1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1]

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

input=1
while 1 :
    u0 = 2.14711467;
    mK = 0.11679756;
    # get keyboard input
    input = raw_input(">> ")

    
    if input == 'exit':
        ser.close()
        exit()


    if input == 'caracterize':
        
        motorPowers = []
        sinAngles = []

        # Prepare plot
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(sinAngles, motorPowers, '.')
        plt.xlim(0.2, 1.2)
        plt.ylim(2.15, 2.3)
        plt.xlabel('sin(Angle)')
        plt.ylabel('Motor Power [PWM Duty%]')
        
        # Initialize aeropendulum as PC mode
        bt_set('mode',0)
        print 'PC mode selected---'
        time.sleep(1)
        bt_set('onOff',1)
        print 'Aero ON---'
        time.sleep(2)
        motorPower = 2.15
        bt_set('motorPower',motorPower)
        time.sleep(1)

        # Gather caracterization data
        i = 0
        last_angle = 0
        last_sinAngle = 0
        increment = 0.001
        while(last_angle<np.pi/2):
            new_angle = bt_get('angle')
            new_sinAngle = np.sin(new_angle)
            while (new_sinAngle < last_sinAngle*0.95) or new_sinAngle > last_sinAngle*1.05:
                last_sinAngle = new_sinAngle
                last_angle = new_angle
                print 'waiting for steady state'
                time.sleep(1)
            motorPowers.append(bt_get('motorPower'))
            sinAngles.append(new_sinAngle)
            motorPower += increment
            bt_set('motorPower',motorPower)
            last_sinAngle = sinAngles[-1]
            last_angle = new_angle 
            i += 1
            print 'Iteration '+str(i)
            line1.set_xdata(sinAngles)
            line1.set_ydata(motorPowers)
            plt.draw()
            plt.pause(1e-17)
            time.sleep(1)
        bt_set('motorPower',2.15)
        plt.savefig('caracterization_data.pdf')
        sinAngles_est = []
        motorP_est = []
        for i in range(len(sinAngles)):
            if (sinAngles[i] > 0.376 and sinAngles[i] < 1 and motorPowers[i] != 0):
                sinAngles_est.append(sinAngles[i])
                motorP_est.append(motorPowers[i])

        # Linearization coeficients calculation
        res = np.polyfit(sinAngles_est, motorP_est, 1)

        # Recreate the line on the plot
        xp = np.linspace(0.37, 1, 100)
        plt.plot(sinAngles_est, motorP_est, '.', xp, res[1]+res[0]*xp)
        plt.show()

        # Save caracterization data to logs
        with open('logs.txt', 'a') as f:
            f.write('Caracterization date:'+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+'\n')
            f.write('angles='+str(sinAngles_est)+'\n')
            f.write('motorPower='+str(motorP_est)+'\n')
        
        # Save the coeficients
        u0 = res[1]
        mK = res[0]
        print 'Caracterization result: '+str(res)








    if input == 'identify':

        ser.flushInput()
        bt_set('onOff',0)
        time.sleep(1)
        bt_set('mode', 2.0)
        time.sleep(1)
        id_angles = []
        linear_input = []
        bt_set('onOff',1)
        time.sleep(2)

        while ser.in_waiting<7:
            time.sleep(0.1)

        for i in range(4*630):
            rval = '0.0'
            rval = ser.read_until('\x00')
            rval = rval.replace('\x00','')
            id_angles.append(float(rval))

        with open('output.csv', 'wb') as myfile:
            wr = csv.writer(myfile)
            wr.writerow(id_angles)



        for i in in_data:
            for j in range(10):
                linear_input.append(-0.015*in_data[i])

        with open('lin_input.csv', 'wb') as myfile:
            wr = csv.writer(myfile)
            wr.writerow(linear_input)












    if input == 'track':
        print 'Angle tracking, press ctrl-C to stop'
        sampling_time = 0.1
        n_of_samples = 100
        bt_set('onOff',0)
        time.sleep(0.5)
        bt_set('tracking',1)
        time.sleep(0.5)
        tracked_angles = []
        seconds = []

        # Prepare plot
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        seconds = np.linspace(-sampling_time*n_of_samples*1.8, 0, n_of_samples)
        all_tracked = []
        tracked_angles = np.full(100, 0.359)
        line1, = ax.plot(seconds, tracked_angles, '.')
        plt.xlim(-10, 0)
        plt.ylim(0.35, 2.5)
        plt.xlabel('time[s]')
        plt.ylabel('angle[rad]')
        
        bt_set('onOff',1)
        time.sleep(2)
        try:
            while(1):
                rval = '0.0'
                rval = ser.read_until('\x00')
                rval = rval.replace('\x00','')
                if rval != '' and rval != 'unknown':
                    rval = float(rval)
                    tracked_angles = np.roll(tracked_angles,-1)
                    tracked_angles[-1] = rval
                    all_tracked.append(rval)
                    line1.set_ydata(tracked_angles)
                    plt.draw()
                    plt.pause(1e-17)
        except KeyboardInterrupt:
            bt_set('tracking',0)
            time.sleep(1)
            pass
        
        # Save caracterization data to logs
        with open('track_logs.txt', 'a') as f:
            f.write('Caracterization date:'+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+'\n')
            f.write('angles='+str(all_tracked)+'\n')
    
    if input == 'on':
        bt_set('onOff',1)
        time.sleep(2)

    if input == 'off':
        bt_set('onOff',0)
        time.sleep(1)

    if input == 'read':
        out = ''
        out += ser.read(ser.in_waiting)
        if out != '':
            print ">>" + out
    if input[0] == 'p':
        bt_write(input)
        while ser.in_waiting < 7:
            time.sleep(0.01)
        out = ''
        out += ser.read(ser.in_waiting)
        ser.flushInput()
        if out != '':
            print '>>'+out

    if input[0] == 's':
        bt_write(input)

    ser.flushInput()
