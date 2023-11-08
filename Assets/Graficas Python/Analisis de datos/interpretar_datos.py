import csv
import math
import numpy as np

FILE = "data.csv"
reader= None
data = []
timestamp = []
imLen = []
delays = []
imgDelays = []
with open(FILE) as csvfile:
    data = csvfile.readlines()
for line in data:
    try:
        spl = line.split(",")
        ts = spl[2]
        im = spl[24][:-1]
        imLen.append(int(im))
        timestamp.append (float(ts))
    except:
        pass

for i in range (1,len(timestamp)):
    try:
        d = timestamp[i] - timestamp[i-1]
        if (imLen[i-1]>0):#imagen
            imgDelays.append(d)
        if (d>0):
            delays.append(d)
    except:
        pass
for i in range (len(timestamp),0,-2):
    try:
        d = timestamp[i] - timestamp[i-1]
        if (d>0):
            delays.append(d)
    except:
        pass


Average = np.average(np.array(delays))
AverageImg = np.average(np.array(imgDelays))
print (f"{Average} secs in average for packet, {AverageImg} secs for image packets")
print (timestamp[2])