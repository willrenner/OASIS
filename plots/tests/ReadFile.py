import time
import os

fName = './logs/readTest.txt'

def getValFromFile():
    currLine = readlastline()
    currTime, val = currLine.split(" ")
    yield currTime, val

def readlastline():
    with open('./logs/abc.txt') as f:
        for line in f:
            pass
    yield line

while(1):
    # x, y = getValFromFile(f)
    # print(x)
    # print(y)
    # l = ''.join(readlastline()) # join iterates over last line object to create a full string
    # p, o = l.split(" ")
    # print(p)
    # print(o)
    count = 0   
    with open(fName, "rb") as file:
        lineNumber = 0
        file.seek(-2, os.SEEK_END)
        while (not(lineNumber == 2)):
            currChar = file.read(1)
            if (currChar == b'\n'): 
                lineNumber = lineNumber + 1
                if (lineNumber == 2):
                    break
            file.seek(-2, os.SEEK_CUR)
        print(file.readline().decode())

    time.sleep(0.1)