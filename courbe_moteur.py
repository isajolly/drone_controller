import csv
import numpy as np
import matplotlib.pyplot as plt

fichiers=["ESC06 - Foglio1.csv","ESC07 - Foglio1.csv","ESC13 - Foglio1.csv","ESC81 - Foglio1.csv"]
#fichiers=["motor-2-16.csv","motor-2-2.csv","motor-2-14.csv","motor-2-13.csv"]

f0=open(fichiers[0], newline='')
f1=open(fichiers[1], newline='')
f2=open(fichiers[2], newline='')
f3=open(fichiers[3], newline='')


reader0= csv.DictReader(f0, delimiter=',')
reader1= csv.DictReader(f1, delimiter=',')
reader2= csv.DictReader(f2, delimiter=',')
reader3= csv.DictReader(f3, delimiter=',')

MotPowMin=1000
MotPowMax=2000
MotExp=0.
SpinMin=1234
SpinMax=1950
CommandIdle=1150
CommandMax=2000
MotExp1=0.
SpinMin1=1150
SpinMax1=1950
ThrustSpinMin1=0.271
ThrustSpinMax1=1.881383512
MotExp2=0.
SpinMin2=1217
SpinMax2=1950
ThrustSpinMin2=0.271
ThrustSpinMax2=1.881383512
MotExp3=0.
SpinMin3=1234
SpinMax3=1950
ThrustSpinMin3=0.271
ThrustSpinMax3=1.881383512

color0='hotpink'
color1='violet'
color2='palegreen'
color3='lightskyblue'

#---------MOTOR 1--------------

ESCSignal=[]
Thrust=[]
Voltage=[]
Current=[]


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
	value= 1249.5+(ESCSignal[i]-1000)*0.6105 #drone orange
	value= 1065+(ESCSignal[i]-1000)*0.73 #drone noir
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
	value= 1152+(ESCSignal1[i]-1000)*0.827 #drone orange
	value= 1125+(ESCSignal1[i]-1000)*0.69 #drone noir
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
	value= 1190+(ESCSignal2[i]-1000)*0.689 #drone orange
	value= 1056+(ESCSignal2[i]-1000)*0.86 #drone noir
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
	value=1104+0.53*(ESCSignal3[i]-1000) #drone orange
	value=1063+0.73*(ESCSignal3[i]-1000) #drone noir
	Corr3.append(value)
	a=2000
	res=0
	for j in range(len(Thrust3)) :
		if abs(value-ESCSignal3[j])<a :
			res=Thrust3[j]
			a=abs(value-ESCSignal3[j])
	New3.append(res)
	

if __name__=="__main__":

	plt.plot(ESCSignal,Thrust, color=color0)
	plt.plot(ESCSignal1,Thrust1, color=color1)
	plt.plot(ESCSignal2,Thrust2, color=color2)
	plt.plot(ESCSignal3,Thrust3, color=color3)
	plt.plot([1050+10*i for i in range(85)],[620 for i in range(85)], color= 'red') #drone noir
	plt.plot([1050+10*i for i in range(85)],[34 for i in range(85)], color= 'red') #drone noir
	#plt.plot([1100+10*i for i in range(85)],[432 for i in range(85)], color= 'red') #drone orange
	
	#plt.plot([1100+10*i for i in range(85)],[60 for i in range(85)], color= 'red') #drone orange
	plt.show()
	
	plt.plot(ESCSignal,New, color=color0)
	plt.plot(ESCSignal1,New1, color=color1)
	plt.plot(ESCSignal2,New2, color=color2)
	plt.plot(ESCSignal3,New3, color=color3)
	#plt.plot([1150+10*i for i in range(85)],[8*i for i in range(50)], color= 'red') #drone noir
	#plt.plot([1050+10*i for i in range(90)],[50+4*i for i in range(90)], color= 'red') #drone orange
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
	
	
