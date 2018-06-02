# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 23:06:50 2017

@author: Pushkar
"""

import tkinter
import os
from firebase import firebase



def takeoffCallBack():
   print( "Takeoff " +E1.get())
   os.system("rosrun mavros mavcmd takeoffcur 0 0 "+E1.get())
   
def landCallBack():
   print( "Takeoff")
   os.system("rosrun mavros mavcmd landcur 0 0")
   
def modeCallBack():
   os.system( "rosrun mavros mavsys mode -c MODE")
   os.system("")
   
def armCallBack():
   os.system( "rosrun mavros mavsafety arm")
   
def disarmCallBack():
   os.system( "rosrun mavros mavsafety disarm")

def clearCallBack():
   os.system( "rosrun mavros mavwp clear")

def showCallBack():
   os.system( "rosrun mavros mavwp show")

def loadCallBack():
   os.system( "rosrun mavros mavwp load filename.txt")

#def helloCallBack():
    #os.system("roscore")
    #os.system("sleep(5)")
    #os.system("rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0:57600")

def getCallBack():
    fire = firebase.FirebaseApplication("https://firebegin-57ea2.firebaseio.com/", None)
    global lat
    lat= fire.get('/lat', None)
    global long 
    long= fire.get('/long', None)
    print ("latitude is", lat)
    print ("longitude is", long)
     
def makeCallBack():
    file = open("latlong.txt","w") 
    file.write(lat)
    file.write(long)
    file.close()
    print("Done!")

    
top = tkinter.Tk()
var = tkinter.StringVar()
label = tkinter.Label(top, textvariable=var)
B = tkinter.Button(text ="Takeoff", command = takeoffCallBack,width="6")
A = tkinter.Button( text ="Land", command = landCallBack,width="6")
C = tkinter.Button(text ="Mode", command = modeCallBack,width="6")
D = tkinter.Button( text ="Arm", command = armCallBack,width="6")
e = tkinter.Button(text ="Disarm", command = disarmCallBack,width="6")
f = tkinter.Button( text ="Clear", command = clearCallBack,width="6")
g = tkinter.Button(text ="Show", command = showCallBack,width="6")
h = tkinter.Button( text ="Load", command = loadCallBack,width="6")
i = tkinter.Button( text ="Get Mission", command = getCallBack,width="10")
j = tkinter.Button( text ="Make Mission", command = makeCallBack,width="10")
var.set("BLOWHORN\n Drone Delivery")
L1 = tkinter.Label(top, text="Height")
E1 = tkinter.Entry(top, bd =5)
#E2 = tkinter.Spinbox(values=("AUTO.MISSION","AUTO.LOITER","AUTO.RTL","STABILIZE"))
E2 = tkinter.Spinbox(values=("MANUAL","ALTCTL","POSCTL","AUTO.MISSION","AUTO.LOITER","AUTO.LAND","AUTO.TAKEOFF","STABILIZED"))
L2 = tkinter.Label(top, text="Height")
label.pack()
L1.pack()
E1.pack()
B.pack()
A.pack()
L2.pack()
E2.pack()
C.pack()
D.pack()
e.pack()
f.pack()
g.pack()
h.pack()
i.pack()
j.pack()
#helloCallBack()
top.mainloop()
