import csv
import numpy as np
import matplotlib.pyplot as plt

fichiers=["ESC06 - Foglio1.csv","ESC07 - Foglio1.csv","ESC13 - Foglio1.csv","ESC81 - Foglio1.csv"]

f0=open(fichiers[0], newline='')
f1=open(fichiers[1], newline='')
f2=open(fichiers[2], newline='')
f3=open(fichiers[3], newline='')


reader0= csv.DictReader(f0, delimiter=',')
reader1= csv.DictReader(f1, delimiter=',')
reader2= csv.DictReader(f2, delimiter=',')
reader3= csv.DictReader(f3, delimiter=',')

#---------MOTOR 1--------------

ESCSignal=[]
Thrust=[]
Voltage=[]
Current=[]

MotPowMin=1000
MotPowMax=2000
MotExp=0.3
SpinMin=1150
SpinMax=1950
CommandIdle=1150
CommandMax=2000

for row in reader0:
	ESCSignal.append(float(row['ESC']))
	Thrust.append(float(row['Thrust']))
	#Voltage.append(float(row['Voltage (V)']))
	#Current.append(float(row['Current (A)']))

Corr=[]
New=[]
for i in range(len(ESCSignal)) :
	c=max(0,(ESCSignal[i]-CommandIdle)/(CommandMax-CommandIdle))
	p=max(0,(1-MotExp)*c+MotExp*c*c)
	value=SpinMin+p*(2000-SpinMin)
	Corr.append(value)
	a=2000
	res=0
	for j in range(len(Thrust)) :
		if abs(value-ESCSignal[j])<a :
			res=Thrust[j]
			a=abs(value-ESCSignal[j])
	New.append(res)
	
	
#---------MOTOR 2--------------
ESCSignal1=[]
Thrust1=[]
Voltage1=[]
Current1=[]

MotExp1=0.3
SpinMin1=1150
SpinMax1=1950
ThrustSpinMin1=0.271
ThrustSpinMax1=1.881383512


for row in reader1:
	ESCSignal1.append(float(row['ESC']))
	Thrust1.append(float(row['Thrust']))
	#Voltage1.append(float(row['Voltage (V)']))
	#Current1.append(float(row['Current (A)']))

Corr1=[]
New1=[]
for i in range(len(ESCSignal1)) :
	c=max(0,(ESCSignal1[i]-CommandIdle)/(CommandMax-CommandIdle))
	p=max(0,(1-MotExp1)*c+MotExp1*c*c)
	value=SpinMin1+p*(2000-SpinMin1)
	Corr1.append(value)
	a=2000
	res=0
	for j in range(len(Thrust1)) :
		if abs(value-ESCSignal1[j])<a :
			res=Thrust1[j]
			a=abs(value-ESCSignal1[j])
	New1.append(res)
	
	
#---------MOTOR 3--------------
ESCSignal2=[]
Thrust2=[]
Voltage2=[]
Current2=[]

MotExp2=0.3
SpinMin2=1150
SpinMax2=1950
ThrustSpinMin2=0.271
ThrustSpinMax2=1.881383512


for row in reader2:
	ESCSignal2.append(float(row['ESC']))
	Thrust2.append(float(row['Thrust']))
	#Voltage2.append(float(row['Voltage (V)']))
	#Current2.append(float(row['Current (A)']))

Corr2=[]
New2=[]
for i in range(len(ESCSignal2)) :
	c=max(0,(ESCSignal2[i]-CommandIdle)/(CommandMax-CommandIdle))
	p=max(0,(1-MotExp2)*c+MotExp2*c*c)
	value=SpinMin2+p*(2000-SpinMin2)
	Corr2.append(value)
	a=2000
	res=0
	for j in range(len(Thrust2)) :
		if abs(value-ESCSignal2[j])<a :
			res=Thrust2[j]
			a=abs(value-ESCSignal2[j])
	New2.append(res)
	
#---------MOTOR 4--------------
ESCSignal3=[]
Thrust3=[]
Voltage3=[]
Current3=[]

MotExp3=0.3
SpinMin3=1150
SpinMax3=1950
ThrustSpinMin3=0.271
ThrustSpinMax3=1.881383512


for row in reader3:
	ESCSignal3.append(float(row['ESC']))
	Thrust3.append(float(row['Thrust']))
	#Voltage3.append(float(row['Voltage (V)']))
	#Current3.append(float(row['Current (A)']))

Corr3=[]
New3=[]
for i in range(len(ESCSignal3)) :
	c=max(0,(ESCSignal3[i]-CommandIdle)/(CommandMax-CommandIdle))
	p=max(0,(1-MotExp3)*c+MotExp3*c*c)
	value=SpinMin3+p*(2000-SpinMin3)
	Corr3.append(value)
	a=2000
	res=0
	for j in range(len(Thrust3)) :
		if abs(value-ESCSignal3[j])<a :
			res=Thrust3[j]
			a=abs(value-ESCSignal3[j])
	New3.append(res)

if __name__=="__main__":

	plt.plot(ESCSignal,New)
	plt.plot(ESCSignal1,New1)
	plt.plot(ESCSignal2,New2)
	plt.plot(ESCSignal3,New3)
	plt.show()

	"""
	fig, axs = plt.subplots(4)
	fig.subplots_adjust(hspace=0.5)

	axs[0].plot(ESCSignal, New)
	axs[0].plot(ESCSignal, Thrust)

	axs[1].plot(ESCSignal1, New1)
	axs[1].plot(ESCSignal1, Thrust1)

	axs[2].plot(ESCSignal2, New2)
	axs[2].plot(ESCSignal2, Thrust2)

	axs[3].plot(ESCSignal3, New3)
	axs[3].plot(ESCSignal3, Thrust3)

	plt.show()
	"""
	
	
