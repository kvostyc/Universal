from PyQt5 import QtWidgets, uic
import serial
import serial.tools.list_ports
import sys
import time
import math

import math
import numpy as np

from Open_GL import PyQtOpenGL #Importovanie OpenGl widgetu !

L1 = 130 #Length of shoulder 1 in mm
L2 = 115 #Length of shoulder 2 in mm
  

#Find the joint angle for x,y

baudrates = [9600, 14400, 28800, 57600, 115200]
ser = ""

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        #Load the UI Page
        uic.loadUi('quart-ui.ui', self)
        
        open_gl = PyQtOpenGL(parent=self.opengl)
        open_gl.setMinimumSize(481, 451)
        
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

    def Connect_COM(self):
        PORT = self.COM_slot.currentText()
        BAUDRATE = self.BR_slot.currentText()
        ser = serial.Serial(PORT,BAUDRATE,timeout=2)
        time.sleep(2)                  
        ser.write('0'.encode("utf-8"))
        self.Console.append("Connected")
    
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
            self.Console.append("Nie ste pripojený.")
        
def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
