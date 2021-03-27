import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os
import datetime


pg.setConfigOption('background', (13, 0.00, 50))
pg.setConfigOption('foreground', 'w')
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('AARC Telem')
fName = './logs/lineSize.txt'
dataArray = []
timeArray = []
y1Array = []
y2Array = []
y3Array = []
y4Array = []
y5Array = []
y6Array = []

TS_MULT_us = 1e6

class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super(TimeAxisItem, self).__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [int2dt(value).strftime("%H:%M:%S") for value in values]


def int2dt(ts, ts_mult=TS_MULT_us): #makes string from a value for the tick mark
    return(datetime.datetime.fromtimestamp(float(ts)/ts_mult)) #ts is posix time in seconds times 1e6


# ---------------------------------------------------------------------------------------------------------------
pg.setConfigOptions(antialias=True)
tai_WOB = TimeAxisItem(orientation='bottom')
plot_WOB = win.addPlot(row=0, col=0, labels={'left': "WOB (N)"}, axisItems={
    'bottom': tai_WOB}, title="WOB Plot (last 10 seconds)")
tai_WOB.enableAutoSIPrefix(enable=False)
plot_WOB.setMouseEnabled(y=False)
curve_WOB = plot_WOB.plot(pen=pg.mkPen('w', width=3))
ptr_WOB = 0
def update_WOB1():
    global ptr_WOB
    readLastLine()
    ptr_WOB += 1
    curve_WOB.setData(x=timeArray[-200:], y=y1Array[-200:])
    curve_WOB.setPos(ptr_WOB, 0)

# ---------------------------------------------------------------------------------------------------------------
tai_WOB2 = TimeAxisItem(orientation='bottom')
tai_WOB2.enableAutoSIPrefix(enable=False)
plot_WOB2 = win.addPlot(row=0, col=1, labels={'left': "WOB (N)"}, axisItems={
    'bottom': tai_WOB2}, title="WOB Plot (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
plot_WOB2.setDownsampling(mode='peak')
plot_WOB2.setClipToView(True)
plot_WOB2.setMouseEnabled(y=False)
curve_WOB2 = plot_WOB2.plot(pen=pg.mkPen('w', width=3))
def update_WOB2():
    curve_WOB2.setData(x=timeArray, y=y1Array)
# ---------------------------------------------------------------------------------------------------------------
tai_DrillRPM = TimeAxisItem(orientation='bottom')
tai_DrillRPM.enableAutoSIPrefix(enable=False)
plot_DrillRPM = win.addPlot(row=1, col=0, labels={'left': "Drill RPM"}, axisItems={
    'bottom': tai_DrillRPM}, title="Drill RPM (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
plot_DrillRPM.setDownsampling(mode='peak')
plot_DrillRPM.setClipToView(True)
plot_DrillRPM.setMouseEnabled(y=False)
curve_DrillRPM = plot_DrillRPM.plot(pen=pg.mkPen('w', width=3))


def update_DrillRPM():
    curve_DrillRPM.setData(x=timeArray, y=y2Array)
# ---------------------------------------------------------------------------------------------------------------
tai_DrillXpos = TimeAxisItem(orientation='bottom')
tai_DrillXpos.enableAutoSIPrefix(enable=False)
plot_DrillXpos = win.addPlot(row=2, col=0, labels={'left': "Drill X-Position From Limit Switch (mm)"}, axisItems={
    'bottom': tai_DrillXpos}, title="Drill Z-Position (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
plot_DrillXpos.setDownsampling(mode='peak')
plot_DrillXpos.setClipToView(True)
plot_DrillXpos.setMouseEnabled(y=False)
curve_DrillXpos = plot_DrillXpos.plot(pen=pg.mkPen('w', width=3))


def update_DrillXpos():
    curve_DrillXpos.setData(x=timeArray, y=y4Array)
# ---------------------------------------------------------------------------------------------------------------
tai_MirageAngle = TimeAxisItem(orientation='bottom')
tai_MirageAngle.enableAutoSIPrefix(enable=False)
plot_MirageAngle = win.addPlot(row=2, col=1, labels={'left': "Mirage Angle (deg)"}, axisItems={
    'bottom': tai_MirageAngle}, title="Mirage Angle From Start Position (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
plot_MirageAngle.setDownsampling(mode='peak')
plot_MirageAngle.setClipToView(True)
plot_MirageAngle.setMouseEnabled(y=False)
curve_MirageAngle = plot_MirageAngle.plot(pen=pg.mkPen('w', width=3))


def update_MirageAngle():
    curve_MirageAngle.setData(x=timeArray, y=y5Array)
# ---------------------------------------------------------------------------------------------------------------
tai_LimSwitch = TimeAxisItem(orientation='bottom')
tai_LimSwitch.enableAutoSIPrefix(enable=False)
plot_LimSwitch = win.addPlot(row=3, col=0, labels={'left': "Limit Switch Active (bool)"}, axisItems={
    'bottom': tai_LimSwitch}, title="Limit Switch Activity (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
plot_LimSwitch.setDownsampling(mode='peak')
plot_LimSwitch.setClipToView(True)
plot_LimSwitch.setMouseEnabled(y=False)
curve_LimSwitch = plot_LimSwitch.plot(pen=pg.mkPen('w', width=3))
def update_LimSwitch():
    curve_LimSwitch.setData(x=timeArray, y=y6Array)


# ---------------------------------------------------------------------------------------------------------------
tai_DrillCurrent = TimeAxisItem(orientation='bottom')
tai_DrillCurrent.enableAutoSIPrefix(enable=False)
plot_DrillCurrent = win.addPlot(row=1, col=1, labels={'left': "Drill Current (Amps) "}, axisItems={
    'bottom': tai_DrillCurrent}, title="Drill Current (enitre history)")
# Use automatic downsampling and clipping to reduce the drawing load
plot_DrillCurrent.setDownsampling(mode='peak')
plot_DrillCurrent.setClipToView(True)
plot_DrillCurrent.setMouseEnabled(y=False)
curve_DrillCurrent = plot_DrillCurrent.plot(pen=pg.mkPen('w', width=3))


def update_DrillCurrent():
    curve_DrillCurrent.setData(x=timeArray, y=y3Array)
# ---------------------------------------------------------------------------------------------------------------

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
        y3Array.append(float(dataArray[3]))
        y4Array.append(float(dataArray[4]))
        y5Array.append(float(dataArray[5]))
        y6Array.append(float(dataArray[6]))




def update():
    update_WOB1()
    update_WOB2()
    update_DrillRPM()
    update_DrillXpos()
    update_MirageAngle()
    update_LimSwitch()
    update_DrillCurrent()

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
timer.start(1000)  # ms

# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



