import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os
import datetime
import collections
#deque
pg.setConfigOption('background', (13, 0.00, 50))
pg.setConfigOption('foreground', 'w')
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('AARC Telem')
# fName = './logs/abcd.txt'
fName = 'C:/Users/willr/Desktop/OASIS/logs/Run_3.txt'
# fName = 'C:/Users/Will/Desktop/OASIS/logs/logTest1.txt'
# LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined
#LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined ----- ACTIVE
numSeconds = 20  # sec
timerTime = 100  # ms
numDataPoints = int(numSeconds / (timerTime / 1000))

dataArray = []
timeArray = []
y1Array = []
y2Array = []
y3Array = []
y4Array = []
y5Array = []
y6Array = []
y7Array = []
y8Array = []
y9Array = []
y10Array = []
TS_MULT_us = 1e6



class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super(TimeAxisItem, self).__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [int2dt(value).strftime("%H:%M:%S") for value in values]


def int2dt(ts, ts_mult=TS_MULT_us): #makes string from a value for the tick mark
    return(datetime.datetime.fromtimestamp(float(ts)/ts_mult)) #ts is posix time in seconds times 1e6

#LoadCellLeftValue, LoadCellRightValue, DrillCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos
# ---------------------------------------------------------------------------------------------------------------
pg.setConfigOptions(antialias=True)
tai_WOB = TimeAxisItem(orientation='bottom')
plot_WOB = win.addPlot(row=0, col=0, labels={'left': "WOB (N)"}, axisItems={
    'bottom': tai_WOB}, title="WOB Plot (last " + str(numSeconds) + " seconds)")
tai_WOB.enableAutoSIPrefix(enable=False)
plot_WOB.setMouseEnabled(y=False)
curve_WOB = plot_WOB.plot(pen=pg.mkPen('w', width=3))
ptr_WOB = 0
def update_WOB1():
    global ptr_WOB
    ptr_WOB += 1
    curve_WOB.setData(x=timeArray[-numDataPoints:], y=y9Array[-numDataPoints:])
    curve_WOB.setPos(ptr_WOB, 0)

# ---------------------------------------------------------------------------------------------------------------
tai_HeaterTemp = TimeAxisItem(orientation='bottom')
tai_HeaterTemp.enableAutoSIPrefix(enable=False)
plot_HeaterTemp = win.addPlot(row=0, col=1, labels={'left': "Heater Temp (F)"}, axisItems={
    'bottom': tai_HeaterTemp}, title="Heater Temp Plot")
# Use automatic downsampling and clipping to reduce the drawing load
plot_HeaterTemp.setDownsampling(mode='peak')
plot_HeaterTemp.setClipToView(True)
plot_HeaterTemp.setMouseEnabled(y=False)
curve_HeaterTemp = plot_HeaterTemp.plot(pen=pg.mkPen('w', width=3))
ptr_HeaterTemp = 0
def update_HeaterTemp():
    global ptr_HeaterTemp
    ptr_HeaterTemp += 1
    curve_HeaterTemp.setData(x=timeArray[-numDataPoints:],
                       y=y5Array[-numDataPoints:])
    curve_HeaterTemp.setPos(ptr_HeaterTemp, 0)
# ---------------------------------------------------------------------------------------------------------------
tai_HeaterPower = TimeAxisItem(orientation='bottom')
tai_HeaterPower.enableAutoSIPrefix(enable=False)
plot_HeaterPower = win.addPlot(row=1, col=0, labels={'left': "HeaterPower (%)"}, axisItems={
    'bottom': tai_HeaterPower}, title="HeaterPower")
# Use automatic downsampling and clipping to reduce the drawing load
plot_HeaterPower.setDownsampling(mode='peak')
plot_HeaterPower.setClipToView(True)
plot_HeaterPower.setMouseEnabled(y=False)
curve_HeaterPower = plot_HeaterPower.plot(pen=pg.mkPen('w', width=3))

ptr_HeaterPower = 0
def update_HeaterPower():
    global ptr_HeaterPower
    ptr_HeaterPower += 1
    curve_HeaterPower.setData(x=timeArray[-numDataPoints:],
                       y=y4Array[-numDataPoints:])
    curve_HeaterPower.setPos(ptr_HeaterPower, 0)
# ---------------------------------------------------------------------------------------------------------------
tai_DrillZPos = TimeAxisItem(orientation='bottom')
tai_DrillZPos.enableAutoSIPrefix(enable=False)
plot_DrillZPos = win.addPlot(row=2, col=0, labels={'left': "Drill Z-Position (mm)"}, axisItems={
    'bottom': tai_DrillZPos}, title="Drill Z-Position")
# Use automatic downsampling and clipping to reduce the drawing load
plot_DrillZPos.setDownsampling(mode='peak')
plot_DrillZPos.setClipToView(True)
plot_DrillZPos.setMouseEnabled(y=False)
curve_DrillZPos = plot_DrillZPos.plot(pen=pg.mkPen('w', width=3))

ptr_Drill = 0
def update_DrillZpos():
    global ptr_Drill
    ptr_Drill += 1
    curve_DrillZPos.setData(x=timeArray[-numDataPoints:],
                       y=y6Array[-numDataPoints:])
    curve_DrillZPos.setPos(ptr_Drill, 0)
# ---------------------------------------------------------------------------------------------------------------
tai_MirageAngle = TimeAxisItem(orientation='bottom')
tai_MirageAngle.enableAutoSIPrefix(enable=False)
plot_MirageAngle = win.addPlot(row=2, col=1, labels={'left': "Mirage Angle (steps)"}, axisItems={
    'bottom': tai_MirageAngle}, title="Mirage Angle")
# Use automatic downsampling and clipping to reduce the drawing load
plot_MirageAngle.setDownsampling(mode='peak')
plot_MirageAngle.setClipToView(True)
plot_MirageAngle.setMouseEnabled(y=False)
curve_MirageAngle = plot_MirageAngle.plot(pen=pg.mkPen('w', width=3))

ptr_Mirage = 0
def update_MirageAngle():
    global ptr_Mirage
    ptr_Mirage += 1
    curve_MirageAngle.setData(x=timeArray[-numDataPoints:],
                       y=y8Array[-numDataPoints:])
    curve_MirageAngle.setPos(ptr_Mirage, 0)
# ---------------------------------------------------------------------------------------------------------------
tai_ExtraZPos = TimeAxisItem(orientation='bottom')
tai_ExtraZPos.enableAutoSIPrefix(enable=False)
plot_ExtraZPos = win.addPlot(row=3, col=0, labels={'left': "Extraction Z-Position (mm)"}, axisItems={
    'bottom': tai_ExtraZPos}, title="Extraction Z-Position")
# Use automatic downsampling and clipping to reduce the drawing load
plot_ExtraZPos.setDownsampling(mode='peak')
plot_ExtraZPos.setClipToView(True)
plot_ExtraZPos.setMouseEnabled(y=False)
curve_ExtraZPos = plot_ExtraZPos.plot(pen=pg.mkPen('w', width=3))
ptr_Extra = 0
def update_ExtraZPos():
    global ptr_Extra
    ptr_Extra += 1
    curve_ExtraZPos.setData(x=timeArray[-numDataPoints:],
                       y=y7Array[-numDataPoints:])
    curve_ExtraZPos.setPos(ptr_Extra, 0)


# ---------------------------------------------------------------------------------------------------------------
tai_TotalCurrent = TimeAxisItem(orientation='bottom')
tai_TotalCurrent.enableAutoSIPrefix(enable=False)
plot_TotalCurrent = win.addPlot(row=1, col=1, labels={'left': "Total Current (Amps) "}, axisItems={
    'bottom': tai_TotalCurrent}, title="Total Current")
# Use automatic downsampling and clipping to reduce the drawing load
plot_TotalCurrent.setDownsampling(mode='peak')
plot_TotalCurrent.setClipToView(True)
plot_TotalCurrent.setMouseEnabled(y=False)
curve_TotalCurrent = plot_TotalCurrent.plot(pen=pg.mkPen('w', width=3))
ptr_Current = 0
def update_TotalCurrent():
    global ptr_Current
    ptr_Current += 1
    curve_TotalCurrent.setData(x=timeArray[-numDataPoints:],
                       y=y3Array[-numDataPoints:])
    curve_TotalCurrent.setPos(ptr_Current, 0)


# ---------------------------------------------------------------------------------------------------------------
tai_MSE = TimeAxisItem(orientation='bottom')
tai_MSE.enableAutoSIPrefix(enable=False)
plot_MSE = win.addPlot(row=3, col=1, labels={'left': "MSE (pa) "}, axisItems={
    'bottom': tai_MSE}, title="MSE")
# Use automatic downsampling and clipping to reduce the drawing load
plot_MSE.setDownsampling(mode='peak')
plot_MSE.setClipToView(True)
plot_MSE.setMouseEnabled(y=False)
curve_MSE = plot_MSE.plot(pen=pg.mkPen('w', width=3))
ptr_MSE = 0
def update_MSE():
    global ptr_MSE
    ptr_MSE += 1
    curve_MSE.setData(x=timeArray[-numDataPoints:],
                            y=y10Array[-numDataPoints:])
    curve_MSE.setPos(ptr_MSE, 0)
# ---------------------------------------------------------------------------------------------------------------

# LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined


#LoadCellLeftValue,LoadCellRightValue,TotalCurrent,HeaterPower,HeaterTemp,
#DrillPos, ExtrPos, MiragePos, LoadCellCombined, drillTorque, mse_wob, mse_torque, MSE
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
        y3Array.append(float(dataArray[3]))  # totalSystemCurrent !
        y4Array.append(float(dataArray[4]))  # HeaterPower
        y5Array.append(float(dataArray[5]))  # HeaterTemp
        y6Array.append(float(dataArray[6]))  # DrillPos
        y7Array.append(float(dataArray[7]))  # ExtractionPos
        y8Array.append(float(dataArray[8]))  # MiragePos
        y9Array.append(float(dataArray[9]))  # WOB
        y10Array.append(float(dataArray[13]))  # MSE






def update():
    readLastLine()
    update_WOB1()
    update_HeaterTemp()
    update_HeaterPower()
    update_DrillZpos()
    update_MirageAngle()
    update_ExtraZPos()
    update_TotalCurrent()
    update_MSE()

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
timer.start(timerTime)  # ms

# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



