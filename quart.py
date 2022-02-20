from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *  
 
import serial
import serial.tools.list_ports
import sys
import time
import math

from pathlib import Path
from stl import mesh

import math
import numpy as np

#from Open_GL import PyQtOpenGL #Importovanie OpenGl widgetu !

L1 = 130 #Length of shoulder 1 in mm
L2 = 115 #Length of shoulder 2 in mm
  

#Find the joint angle for x,y

baudrates = [9600, 14400, 28800, 57600, 115200]
ser = ""

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setAcceptDrops(True)

        #Load the UI Page
        uic.loadUi('quart-ui.ui', self)
        
        #open_gl = PyQtOpenGL(parent=self.opengl)
        #open_gl.setMinimumSize(481, 451)

        self.currentSTL = None
        self.lastDir = None
        
        self.droppedFilename = None
   
        self.viewer = gl.GLViewWidget(parent=self.opengl)
        self.viewer.setMinimumSize(840, 700)
        self.viewer.setBackgroundColor(QColor(240, 240, 240))

        self.viewer.setCameraPosition(distance=2000)
        
        g = gl.GLGridItem()
        g.setSize(1000, 1000)
        g.setSpacing(50, 50)
        g.setColor(QColor(33, 50, 94))
        self.viewer.addItem(g)

        
        ports = serial.tools.list_ports.comports(include_links=False)
        for port in ports:
            self.COM_slot.addItem(port.device)

        for j in baudrates:
            self.BR_slot.addItem(str(j))

        self.connButton.clicked.connect(self.Connect_COM)
        self.moveButton.clicked.connect(self.calculateAngles)
        
        self.X_slider.valueChanged.connect(self.updateX)
        self.Y_slider.valueChanged.connect(self.updateY)
                
        self.Console.setReadOnly(True)

        self.console("Welcome back user !")

    def Connect_COM(self):
        PORT = self.COM_slot.currentText()
        BAUDRATE = self.BR_slot.currentText()
        ser = serial.Serial(PORT,BAUDRATE,timeout=2)
        time.sleep(2)                  
        ser.write('0'.encode("utf-8"))
        self.console("Connected")
    
    def updateX(self, event):
        self.X_lcd.display(event)
        self.x = event
    def updateY(self, event):
        self.Y_lcd.display(event)
        self.y = event
        
    def sendData(self):
        data = str(self.Theta2) \
        +","+str(self.Theta1) 
        print(data)
        ser.write(data.encode("utf-8"))
        
    def calculateAngles(self):
        if ser != "":       
            self.Theta2 = math.acos((self.x*self.x + self.y*self.y - L1^2 - L2^2)/(2 * L1 * L2))
            print(round(np.degrees(self.Theta2)))

            self.Theta1 = math.atan(self.x/self.y) - math.atan((L2 * math.sin(self.Theta2))/(L1 + L2 * math.cos(self.Theta2)))
            print(round(np.degrees(self.Theta1)))
            self.sendData()
        else:
            self.console("Nie ste pripojený.")
    
    def console(self, text):
        self.Console.append(text)

    def showSTL(self, filename):
        if self.currentSTL:
            self.viewer.removeItem(self.currentSTL)

        points, faces = self.loadSTL(filename)
        meshdata = gl.MeshData(vertexes=points, faces=faces)
        mesh = gl.GLMeshItem(meshdata=meshdata, smooth=True, drawFaces=True, drawEdges=True, edgeColor=(0, 1, 0, 1))
        self.viewer.addItem(mesh)
        
        self.currentSTL = mesh
        
    def loadSTL(self, filename):
        m = mesh.Mesh.from_file(filename)
        shape = m.points.shape
        points = m.points.reshape(-1, 3)
        faces = np.arange(points.shape[0]).reshape(-1, 3)
        return points, faces

    def dragEnterEvent(self, e):
        print("enter")
        mimeData = e.mimeData()
        mimeList = mimeData.formats()
        filename = None
        
        if "text/uri-list" in mimeList:
            filename = mimeData.data("text/uri-list")
            filename = str(filename, encoding="utf-8")
            filename = filename.replace("file:///", "").replace("\r\n", "").replace("%20", " ")
            filename = Path(filename)
            
        if filename.exists() and filename.suffix == ".stl":
            e.accept()
            self.droppedFilename = filename
        else:
            e.ignore()
            self.droppedFilename = None
        
    def dropEvent(self, e):
        if self.droppedFilename:
            self.showSTL(self.droppedFilename)
        
def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
