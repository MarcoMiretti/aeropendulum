import time
import serial
import matplotlib.pyplot as plt
import numpy as np
import datetime
import csv

class aero:
    """
    init aeropendulum class with port number
    param port (int) port number
    """
    def __init__(self, port):
        self.port = port

    """
    connect to bluetooth com port
    """
    def bt_connect(self):
        self.ser = serial.Serial(
            port = '/dev/rfcomm'+str(self.port),
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2
            )
        self.ser.isOpen()

    """
    writes in_str to bluetooth
    """
    def bt_write(self, in_str ):
        for i in range(32-len(in_str)):
            in_str += '\0'
        self.ser.write(in_str)
        self.ser.flushOutput()

    """
    reads variable value from bluetooth
    variable = string
    returns = float
    """
    def bt_get(self, variable ):
        rval = '0.0'
        self.bt_write('p '+variable)
        rval = self.ser.read(8)
        rval = rval.replace('\x00','')
        if rval == '':
            rval = '0.0'
        return float(rval)
        
    """
    sets a variable value through bluetooth
    variable = string
    value = float, int
    """
    def bt_set(self, variable, value ):
        self.bt_write('s '+variable+' '+str(float(value)))
    
    """
    disconnects from serial port
    """
    def bt_disconnect(self):
        self.ser.close()
        exit()

    """
    Obtains feedback values for the inner control loop
    """
    def caracterize(self):
        motorPowers = []
        sinAngles = []
        # Discard existen serial port data
        self.ser.flushInput()

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
        self.bt_set('mode',0)
        print 'PC mode selected---'
        time.sleep(1)
        self.bt_set('onOff',1)
        print 'Aero ON---'
        time.sleep(2)
        motorPower = 2.15
        self.bt_set('motorPower',motorPower)
        time.sleep(1)

        # Gather caracterization data
        i = 0
        last_angle = 0
        last_sinAngle = 0
        increment = 0.001
        while(last_angle<np.pi/2):
            new_angle = self.bt_get('angle')
            new_sinAngle = np.sin(new_angle)
            while (new_sinAngle < last_sinAngle*0.95) or new_sinAngle > last_sinAngle*1.05:
                last_sinAngle = new_sinAngle
                last_angle = new_angle
                print 'Sistem is not in steady state... wait.'
                time.sleep(1)
            motorPowers.append(self.bt_get('motorPower'))
            sinAngles.append(new_sinAngle)
            motorPower += increment
            self.bt_set('motorPower',motorPower)
            last_sinAngle = sinAngles[-1]
            last_angle = new_angle 
            i += 1
            print 'Iteration '+str(i)
            line1.set_xdata(sinAngles)
            line1.set_ydata(motorPowers)
            plt.draw()
            plt.pause(1e-17)
            time.sleep(1)
        self.bt_set('motorPower',2.15)
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
        print "Caracterization data saved to logs.txt"

        # Save the coeficients
        self.u0 = res[1]
        self.mK = res[0]

        print 'Caracterization result: '+str(res)

    def identify(self):
        
        self.ser.flushInput()

        print "Turn AERO OFF..."
        self.bt_set('onOff',0)
        time.sleep(1)

        print "Select mode 'identification'..."
        self.bt_set('mode', 2.0)
        time.sleep(1)
        id_angles = []
        linear_input = []
        print "Turn motors ON..."
        self.bt_set('onOff',1)
        time.sleep(2)


        print "To start the identification lightly push the pendulum to 90 degrees"
        # wait until first datum is ready
        while self.ser.in_waiting<7:
            time.sleep(0.1)

        print "Reading output data..."
        # read angle output values from aeropendulum
        for i in range(4*630):
            rval = '0.0'
            rval = self.ser.read_until('\x00')
            rval = rval.replace('\x00','')
            id_angles.append(float(rval))

        print "Writing angles to output.csv..."
        # write output
        with open('output.csv', 'wb') as myfile:
            wr = csv.writer(myfile)
            wr.writerow(id_angles)

        print "Gather input PRBS..."
        with open('rawPRBS.csv', 'rb') as f:
            reader = csv.reader(f)
            in_data = list(reader)[0]

        for i in in_data:
            for j in range(10):
                linear_input.append(-0.015*int(i))

        print "Writing input prbs to lin_input.csv..."
        with open('lin_input.csv', 'wb') as myfile:
            wr = csv.writer(myfile)
            wr.writerow(linear_input)

        print "Turn motors OFF"
        self.bt_set('onOff',0)

    def track(self):
        self.ser.flushInput()
        print 'Angle tracking, press ctrl-C to stop'
        sampling_time = 0.1
        n_of_samples = 100
        tracked_angles = []
        seconds = []
         
        self.bt_set('onOff',0)
        time.sleep(0.5)
        self.bt_set('tracking',1)
        time.sleep(0.5)
        
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
        
        self.bt_set('onOff',1)
        time.sleep(2)
        try:
            while(1):
                rval = '0.0'
                rval = self.ser.read_until('\x00')
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
            self.bt_set('tracking',0)
            time.sleep(1)
            pass
        
        # Save caracterization data to logs
        with open('track_logs.txt', 'a') as f:
            f.write('Caracterization date:'+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+'\n')
            f.write('angles='+str(all_tracked)+'\n')


print 'Insert port number:'
input_port=input()
aero1 = aero(int(input_port))
aero1.bt_connect()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

input=1
while 1 :
    # get keyboard input
    input = raw_input(">> ")

    if input == 'exit':
        aero1.bt_disconnect()

    if input == 'caracterize':
        aero1.caracterize()

    if input == 'identify':
        aero1.identify()

    if input == 'track':
        aero1.track()

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

    aero1.ser.flushInput()
