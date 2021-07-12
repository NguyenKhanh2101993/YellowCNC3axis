from tkinter import *
from tkinter.ttk import Combobox
from tkinter import filedialog, messagebox, ttk
import tkinter.font as font
#from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as rtu

import pickle
import serial
import time
import threading
import queue
import math
import webbrowser
#from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from matplotlib import cm
import numpy as np
from math import cos, radians, sin

rx = lambda v, angle : (v[0],(cos(radians(angle))*v[1]) + ((-sin(radians(angle)))*v[2]),
                             (sin(radians(angle))*v[1]) + ((cos(radians(angle)))*v[2]))

ry = lambda v, angle : (((cos(radians(angle)))*v[0]) + ((-sin(radians(angle)))*v[1]), v[2],
                        ((sin(radians(angle)))*v[0]) + ((cos(radians(angle)))*v[1]))

rz = lambda v, angle : (((cos(radians(angle)))*v[0]) + ((-sin(radians(angle)))*v[1]),
                        ((sin(radians(angle)))*v[0]) + ((cos(radians(angle)))*v[1]), v[2])
class shape:
    def __init__(self, file, canvas):
        self.location, self.scale = [0, 0, 0], 1
        self.rotation = [0, 0, 0]
        self.canvas = canvas
        self.EdgeThickness = 1
        self.color="#ffffff"
        self.width = int(self.canvas["width"])
        self.height = int(self.canvas["height"])
        def load_model(file):
            from functools import reduce
            l=open(file, 'r').readlines()
            verts, lines =[], []
            for i in l:
                if (i[:1]=='v') and (i[:2]!='vn'):
                    xv, yv, zv = (i[2:].split(' '))
                    verts.append((float(xv), float(yv), float(zv)))
                elif (i[:1]=='f'):
                    face = reduce(lambda x, y:x+y, map(lambda x:x.split(' '), i.split('//')))[-6:]
                    lines.extend(((verts[int(face[0])-1], verts[int(face[2])-1]),
                                  (verts[int(face[2])-1], verts[int(face[4])-1]),
                                  (verts[int(face[4])-1], verts[int(face[0])-1])))
            return tuple(map(lambda line:
                             (tuple(map(lambda n:n*self.scale, line[0])), tuple(map(lambda n:n*self.scale, line[1]))),
                        lines))

        self.lines = load_model(file)
    def render(self):
        u=int(self.width/16)
        fl=0.15
        def xcor(x, y):
            try:
                if (x<0): return (self.width/2)-(x/(y*fl))*(-1*u)
                else: return (self.width/2)+(x/(y*fl))*u
            except(ZeroDivisionError):return 0
        def ycor(z, y):
            try:
                if (z<0): return (self.height/2)-(z/(y*fl))*u
                else: return (self.height/2)+(z/(y*fl))*(-1*u)
            except(ZeroDivisionError):return 0

        add = lambda x, y:tuple(map(lambda a, b:a+b, x, y))
        vr = list(map(lambda v:(add(self.location, rz(ry(rx(v[0], self.rotation[0]), self.rotation[1]), self.rotation[2])),
                                add(self.location, rz(ry(rx(v[1], self.rotation[0]), self.rotation[1]), self.rotation[2]))),
                      self.lines))
        for l in vr: self.canvas.create_line(xcor(l[0][0], l[0][1]), ycor(l[0][2], l[0][1]),
                                             xcor(l[1][0], l[1][1]), ycor(l[1][2], l[1][1]), fill=self.color, width=self.EdgeThickness)
