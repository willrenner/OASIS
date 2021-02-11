import pytz
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os
import pytz
import datetime
from collections import deque

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('AARC Telem')
fName = './logs/abcdefg.txt'

UNIX_EPOCH_naive = datetime.datetime(1970, 1, 1, 0, 0)  # offset-naive datetime
UNIX_EPOCH_offset_aware = datetime.datetime(
    1970, 1, 1, 0, 0, tzinfo=pytz.timezone('US/Central'))  # offset-aware datetime
UNIX_EPOCH = UNIX_EPOCH_offset_aware
TS_MULT_us = 1e6


class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        #  super().__init__(*args, **kwargs)
        super(TimeAxisItem, self).__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        # PySide's QTime() initialiser fails miserably and dismisses args/kwargs
        # return [QTime().addMSecs(value).toString('mm:ss') for value in values]
        return [int2dt(value).strftime("%H:%M:%S.%f") for value in values]


def now_timestamp(ts_mult=TS_MULT_us, epoch=UNIX_EPOCH):
    return(int((datetime.datetime.now(datetime.timezone.utc) - epoch).total_seconds()*ts_mult))


def int2dt(ts, ts_mult=TS_MULT_us):
    return(datetime.datetime.utcfromtimestamp(float(ts)/ts_mult))


def dt2int(dt, ts_mult=TS_MULT_us, epoch=UNIX_EPOCH):
    delta = dt - epoch
    return(int(delta.total_seconds()*ts_mult))


def td2int(td, ts_mult=TS_MULT_us):
    return(int(td.total_seconds()*ts_mult))


def int2td(ts, ts_mult=TS_MULT_us):
    return(datetime.timedelta(seconds=float(ts)/ts_mult))


# 1) Simplest approach -- update data in the array such that plot appears to scroll. In these examples, the array size is fixed.
pg.setConfigOptions(antialias=True)
tai = TimeAxisItem(orientation='bottom')
p1 = win.addPlot(row=0, col=0, labels={'left': "ROP", 'bottom': "Time"}, axisItems={
    'bottom': TimeAxisItem(orientation='bottom')})
p1.setMouseEnabled(y=False)
maxlen = 100
data_x = deque(maxlen=maxlen)
data_y = deque(maxlen=maxlen)
curve1 = p1.plot()
ptr1 = 0


def update1():
    global ptr1
    gen = "".join(readLastLine())  # now a string
    x, y = gen.split(" ")
    ptr1 += 1
    data_y.append(float(y))
    data_x.append(now_timestamp())
    curve1.setData(x=list(data_x), y=list(data_y))
    curve1.setPos(ptr1, 0)


# 2) Allow data to accumulate. In these examples, the array doubles in length whenever it is full.
p2 = win.addPlot(row=0, col=1, labels={'left': "ROP", 'bottom': "Time"}, axisItems={
    'bottom': TimeAxisItem(orientation='bottom')})
# Use automatic downsampling and clipping to reduce the drawing load
p2.setDownsampling(mode='peak')
p2.setClipToView(True)
p2.setMouseEnabled(y=False)
curve2 = p2.plot()
data_x2 = list()
data_y2 = list()

ptr2 = 0


def update2():
    global ptr2
    gen = "".join(readLastLine())  # now a string
    x, y = gen.split(" ")
    data_y2.append(float(y))
    data_x2.append(now_timestamp())
    ptr2 += 1
    # if ptr2 >= data2.shape[0]:
    #     tmp = data2
    #     data2 = np.empty(data2.shape[0] * 2)
    #     data2[:tmp.shape[0]] = tmp
    curve2.setData(x=list(data_x2), y=list(data_y2))


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


# p3 = win.addPlot(row=1, col=0, colspan=2, labels={
#                  'left': "ROP", 'bottom': "Time"})

# Use automatic downsampling and clipping to reduce the drawing load
# p3.setDownsampling(mode='peak')
# p3.setClipToView(True)
# p3.setMouseEnabled(y=False)
# curve3 = p3.plot()
# ptr3 = 0

# data3, ptr3 = loadAllPreviousValues()
# curve3.setData(data3)

# def update3():
#     global data3, ptr3
#     gen = "".join(readLastLine())  # now a string
#     x, y = gen.split(" ")
#     data3[ptr3] = float(y)
#     ptr3 += 1
#     if ptr3 >= data3.shape[0]:
#         tmp = data3
#         data3 = np.empty(data3.shape[0] * 2)
#         data3[:tmp.shape[0]] = tmp
#     curve3.setData(data3[:ptr3])


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
    # update3()


timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)  # ms

# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
