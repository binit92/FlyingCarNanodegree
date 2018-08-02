import numpy as np
import os 

dir = os.path.dirname(os.path.abspath(__file__))
parent = os.path.dirname(dir)
#print(parent)

gps_file = parent + '\\config\\log\\Graph1.txt'
acc_file = parent + '\\config\\log\\Graph2.txt'

#print (gps_file)
#print (acc_file)

def Calculate(filename):
	xlist = []
	with open(filename, 'rt') as f:
		next(f)
		for line in f:
			_,x = line.strip().split(',')
			#print(x)
			xlist.append(float(x))

	xarray = np.array(xlist,dtype=float)
	#print(xarray)
	mean = "xmean = {}".format(xarray.mean())
	std = "xstd = {}".format(xarray.std())		
	print(mean);
	print(std);
	return std

if (os.path.exists(gps_file)):
	print("Standard Deviation for GPS", Calculate(gps_file));
	
if (os.path.exists(acc_file)):
	print("Standard Deviation for Accelerometer", Calculate(acc_file));
	