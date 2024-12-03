import matplotlib.pyplot as plt
from serial import Serial
import time
import random
import statistics
import re

s = Serial("/dev/cu.usbserial-0001", 9600)

smooth = 1
k = []
x = []
y = []

plt.ion()
while True:
    try:
        q = s.read(5)
        if q:
            n = str(q)[3:-1]
            if not all(i.isdigit() for i in n):
                continue

            k.append(int(n))
            
            a = [k[i:i+smooth] for i in range(0, len(k), smooth)]
            a = [statistics.mean(i) for i in a]
            
            x = list(range(len(a)))
            y = a[:]

        plt.plot(y)
        plt.draw()
        plt.pause(0.001)
        plt.clf()
    except Exception as e:
        print(e)
        time.sleep(1)
        
