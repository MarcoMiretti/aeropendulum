import time
import serial
import codecs
import struct

print 'port number:'
input_port=raw_input()
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/rfcomm'+input_port,
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

input=1
while 1 :
    # get keyboard input
    input = raw_input(">> ")
    for i in range(32-len(input)):
        input += '\0'
    if input == 'exit':
        ser.close()
        exit()
    else:
        # send the character to the device
        ser.write(input)
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(0.5)
        while ser.inWaiting() > 0:
            out += ser.read(1)

        if out != '':
            print ">>" + out
