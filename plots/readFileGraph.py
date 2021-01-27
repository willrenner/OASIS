import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('AARC Telem')
# fName = './logs/readTest.txt'
fName = './logs/abcd.txt'

# 1) Simplest approach -- update data in the array such that plot appears to scroll. In these examples, the array size is fixed.
pg.setConfigOptions(antialias=True)
p1 = win.addPlot(row=0, col=0, labels={'left': "ROP", 'bottom': "Time"})
p1.setMouseEnabled(y=False)
data1x = np.zeros(300)
data1y = np.zeros(300)
curve1 = p1.plot()
ptr1 = 0


def update1():
    global data1x, ptr1
    data1x[:-1] = data1x[1:]  # shift data in the array one sample left
    data1y[:-1] = data1y[1:]
    gen = "".join(readLastLine())  # now a string
    x, y = gen.split(" ")
    # data1x[-1] = float(x)
    data1y[-1] = float(y)
    ptr1 += 1
    curve1.setData(data1y)
    curve1.setPos(ptr1, 0)


# 2) Allow data to accumulate. In these examples, the array doubles in length whenever it is full.
p2 = win.addPlot(row=0, col=1, labels={'left': "ROP", 'bottom': "Time"})
# Use automatic downsampling and clipping to reduce the drawing load
p2.setDownsampling(mode='peak')
p2.setClipToView(True)
p2.setMouseEnabled(y=False)
curve2 = p2.plot()
data2 = np.empty(100)
ptr2 = 0


def update2():
    global data2, ptr2
    gen = "".join(readLastLine())  # now a string
    x, y = gen.split(" ")
    data2[ptr2] = float(y)
    ptr2 += 1
    if ptr2 >= data2.shape[0]:
        tmp = data2
        data2 = np.empty(data2.shape[0] * 2)
        data2[:tmp.shape[0]] = tmp
    curve2.setData(data2[:ptr2])


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


p3 = win.addPlot(row=1, col=0, colspan=2, labels={
                 'left': "ROP", 'bottom': "Time"})


# Use automatic downsampling and clipping to reduce the drawing load
p3.setDownsampling(mode='peak')
p3.setClipToView(True)
p3.setMouseEnabled(y=False)
curve3 = p3.plot()
ptr3 = 0

data3, ptr3 = loadAllPreviousValues()
curve3.setData(data3)


def update3():
    global data3, ptr3
    gen = "".join(readLastLine())  # now a string
    x, y = gen.split(" ")
    data3[ptr3] = float(y)
    ptr3 += 1
    if ptr3 >= data3.shape[0]:
        tmp = data3
        data3 = np.empty(data3.shape[0] * 2)
        data3[:tmp.shape[0]] = tmp
    curve3.setData(data3[:ptr3])


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
        yield file.readline().decode()

    # update all plots


def update():
    update1()
    update2()
    update3()


timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # ms

# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
