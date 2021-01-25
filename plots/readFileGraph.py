import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import os

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('AARC Telem')
# fName = './logs/readTest.txt'
fName = './logs/abc.txt'

# 1) Simplest approach -- update data in the array such that plot appears to scroll. In these examples, the array size is fixed.
pg.setConfigOptions(antialias=True)
p1 = win.addPlot(labels = {'left': "ROP", 'bottom': "Time"})
p1.setMouseEnabled(y=False)
data1x = np.zeros(300)
data1y = np.zeros(300)
curve1 = p1.plot()
ptr1 = 0

def update1():
    global data1x, ptr1
    data1x[:-1] = data1x[1:]  # shift data in the array one sample left    
    data1y[:-1] = data1y[1:]    
    gen = "".join(readLastLine()) # now a string
    x, y = gen.split(" ")
    # data1x[-1] = float(x)
    data1y[-1] = float(y)
    ptr1 += 1
    curve1.setData(data1y)
    curve1.setPos(ptr1, 0)

# 2) Allow data to accumulate. In these examples, the array doubles in length whenever it is full. 
p2 = win.addPlot(labels = {'left': "ROP", 'bottom': "Time"})
p2.setDownsampling(mode='peak')# Use automatic downsampling and clipping to reduce the drawing load
p2.setClipToView(True)
p2.setMouseEnabled(y=False)
curve2 = p2.plot()
data2 = np.empty(100)
ptr2 = 0


def update2():
    global data2, ptr2
    gen = "".join(readLastLine()) #now a string
    x, y = gen.split(" ") 
    data2[ptr2] = float(y)
    ptr2 += 1
    if ptr2 >= data2.shape[0]:
        tmp = data2
        data2 = np.empty(data2.shape[0] * 2)
        data2[:tmp.shape[0]] = tmp
    curve2.setData(data2[:ptr2])


# # 3) Copy
# p5 = win.addPlot()

# p5.setDownsampling(mode='peak')
# p5.setClipToView(True)
# curve5 = p5.plot()

# data5, ptr5 = populateFromFile()
# ptr5 = 0

# def update3():
#     global data3, ptr3
#     gen = "".join(readLastLine()) #now a string
#     x, y = gen.split(" ") 
#     data3[ptr3] = float(y)
#     ptr3 += 1
#     if ptr3 >= data3.shape[0]:
#         tmp = data3
#         data3 = np.empty(data3.shape[0] * 2)
#         data3[:tmp.shape[0]] = tmp
#     curve4.setData(data3[:ptr3])


def readLastLine(): #actually gets second to last line b/c last line might not be finished from matlab
    with open(fName, "rb") as file:
        lineNumber = 0 #from bottom
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
timer.start(50) #ms

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()



