import numpy
import math
file = open("gdb.txt", "r") 
count = 0
angleValues = []
pwmValues = []
for line in file:
    count += 1
    if count > 3:
        flag = 0
        number = ""
        for letter in line:
            if flag == 2:
                number += letter
            if flag == 1:
                flag = 2
            if letter == "=":
                flag = 1
        flag = 0
        if count == 4:
            angleValues.append(math.sin(float(number)))
        else:
            pwmValues.append(float(number))
            count = 0

res = numpy.polyfit(angleValues, pwmValues, 1)
print res
