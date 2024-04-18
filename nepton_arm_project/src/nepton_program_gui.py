#!/usr/bin/env /home/inbarm/catkin_ws/src/HRI_Project/remote_gesture_recognition/src/py3venv/bin/python3
import rospy
import tkinter as tk
import matplotlib as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.figure import Figure
# import sys
import numpy as np
from PIL import ImageTk, Image
import os
from scipy.spatial.transform import Rotation as R

from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

font = {'family': 'serif',
        'color':  'k',
        'weight': 'normal',
        # 'size': 16,
        }

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Nepton Project')
        # self.geometry('700x550')
        # self.maxsize(700,  550) 

        self.config(background= "white")
        self.grid_rowconfigure(0, weight=1)

        left_frame  =  tk.Frame(self,  width=400,  height=  600,  bg='white')
        left_frame.grid(row=1,  column=0,  padx=0,  pady=5)
        up_frame  =  tk.Frame(self,  width=500,  height=600,  bg='white')
        up_frame.grid(row=0,  column=0,  padx=0,  pady=5)

        image = Image.open("/home/inbarm/catkin_ws/src/Nepton_Project/nepton_arm_project/src/logo-writing.png")
        width, height = image.size
        resize_image = image.resize((int(width/5), int(height/5)))
        img = ImageTk.PhotoImage(resize_image)
        label = tk.Label(up_frame,image = img,compound='center')
        label.image = img
        label.grid(row=0,column=2)
        self.entry1= tk.Entry(left_frame,width= 5)
        self.entry1.focus_set()
        self.entry1.grid(row=7,column=3)
        set_amplitude = tk.Button(left_frame,text='set amplitude:',font=('calbiri',12),command= self.set_amplitude,bg='#5496fa')
        set_amplitude.grid(row=7,column=2)

        self.entry2= tk.Entry(left_frame,width= 5)
        self.entry2.focus_set()
        self.entry2.grid(row=8,column=3)
        set_cycle_time = tk.Button(left_frame,text='set cycle time:',font=('calbiri',12),command= self.set_cycle_time,bg='#5496fa')
        set_cycle_time.grid(row=8,column=2)

        self.entry3= tk.Entry(left_frame,width= 5)
        self.entry3.focus_set()
        self.entry3.grid(row=9,column=3)
        set_movment_time = tk.Button(left_frame,text='set movment time in seconds:',font=('calbiri',12),command= self.set_movment_time,bg='#5496fa')
        set_movment_time.grid(row=9,column=2)

        # self.ani = animation.FuncAnimation(fig, self.animate, interval=5)


    def set_amplitude(self):
        self.amplitude= float(self.entry1.get())
        rospy.set_param('amplitude', self.amplitude)

    def set_cycle_time(self):
        self.cycle_time = float(self.entry2.get())
        rospy.set_param('cycle_time', self.cycle_time)

    def set_movment_time(self):
        self.movment_time = float(self.entry3.get())
        rospy.set_param('movment_time', self.movment_time)
        
        
def startApp():
    App().mainloop()

if __name__ == "__main__":
    App().mainloop() 
    
    