import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os
import datetime


pg.setConfigOption('background', (7, 0.00, 23))
pg.setConfigOption('foreground', 'w')
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('AARC Telem')
fName = './logs/firstLog.txt'
dataArray = []
timeArray = []
y1Array = []
y2Array = []

TS_MULT_us = 1e6

class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super(TimeAxisItem, self).__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [int2dt(value).strftime("%H:%M:%S") for value in values]


def int2dt(ts, ts_mult=TS_MULT_us): #makes string from a value for the tick mark
    return(datetime.datetime.fromtimestamp(float(ts)/ts_mult)) #ts is posix time in seconds times 1e6


# 1) Simplest approach -- update data in the array such that plot appears to scroll. In these examples, the array size is fixed.
pg.setConfigOptions(antialias=True)
tai1 = TimeAxisItem(orientation='bottom')
p1 = win.addPlot(row=0, col=0, labels={'left': "WOB (N)"}, axisItems={
    'bottom': tai1}, title="WOB Plot (last 10 seconds)")
tai1.enableAutoSIPrefix(enable=False)
p1.setMouseEnabled(y=False)
curve1 = p1.plot()
ptr1 = 0
def update1():
    global ptr1
    readLastLine()
    ptr1 += 1
    curve1.setData(x=timeArray[-200:], y=y1Array[-200:])
    curve1.setPos(ptr1, 0)

# 2) Allow data to accumulate. In these examples, the array doubles in length whenever it is full.
tai2 = TimeAxisItem(orientation='bottom')
tai2.enableAutoSIPrefix(enable=False)
p2 = win.addPlot(row=0, col=1, labels={'left': "WOB (N)"}, axisItems={
    'bottom': tai2}, title="WOB Plot (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
p2.setDownsampling(mode='peak')
p2.setClipToView(True)
p2.setMouseEnabled(y=False)
curve2 = p2.plot()
def update2():
    curve2.setData(x=timeArray, y=y1Array)


# 2) Allow data to accumulate. In these examples, the array doubles in length whenever it is full.
tai3 = TimeAxisItem(orientation='bottom')
tai3.enableAutoSIPrefix(enable=False)
p3 = win.addPlot(row=1, col=0, labels={'left': "Angle (Degrees)"}, axisItems={
    'bottom': tai3}, title="Mirage Angle Plot (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
p3.setDownsampling(mode='peak')
p3.setClipToView(True)
p3.setMouseEnabled(y=False)
curve3 = p3.plot()
def update3():
    curve3.setData(x=timeArray, y=y2Array)


def readLastLine():  # actually gets second to last line b/c last line might not be finished from matlab
    with open(fName, "rb") as file:  # binary mode, must do this to start at end of file
        lineNumber = 0  # from bottom
        file.seek(-2, os.SEEK_END)
        while (not(lineNumber == 2)):
            currChar = file.read(1)
            if (currChar == b'\n'):
                lineNumber = lineNumber + 1
                if (lineNumber == 2):
                    break
            file.seek(-2, os.SEEK_CUR)
        good, trash = file.readline().decode().split("\r\n")
        dataArray = good.split(" ")
        timeArray.append(int(float(dataArray[0]) * TS_MULT_us))
        y1Array.append(float(dataArray[1]))
        y2Array.append(float(dataArray[2]))


def update():
    update1()
    update2()
    update3()


def getLineCount():
    count = 0
    with open(fName, "r") as file:
        for line in file:
            count = count + 1
    return count

def loadAllPreviousValues():  # returns an array of all data in file
    ptr99 = 0
    data99 = np.empty(100)
    count = getLineCount()
    with open(fName, "r") as file:
        for line in file:
            if (ptr99 >= (count - 3)):  # avoid reading from very end of file which may not be done from matlab
                break
            # resizing array to accomodate new values
            if ptr99 >= data99.shape[0]:
                tmp99 = data99
                data99 = np.empty(data99.shape[0] * 2)
                data99[:tmp99.shape[0]] = tmp99
            gen2 = file.readline()
            # print(currLine)
            x, y = gen2.split(" ")
            ynew, throwaway = y.split("\n")
            data99[ptr99] = float(ynew)
            ptr99 = ptr99 + 1
    return data99, ptr99

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # ms

# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



