import re
from matplotlib import pyplot as plt
import numpy as np
import sys, os

#sys.path.append('../src/interfaces')
sys.path.append(os.path.join(os.path.dirname(sys.path[0]),'src', 'interfaces'))

from timing import SYNCHRONIZATION_PERIOD, NB_NODES, SCHEDULING_SLOT_DURATION, NB_SCHEDULING_CYCLES, TASK_SLOT_DURATION, NB_TASK_SLOTS, NB_FULL_CYCLES

TASK_START_TIME = (SYNCHRONIZATION_PERIOD + SCHEDULING_SLOT_DURATION * NB_NODES * NB_SCHEDULING_CYCLES / 2)/1000
FRAME_DURATION = (TASK_SLOT_DURATION * NB_TASK_SLOTS)/1000
FULL_CYCLE_DURATION = TASK_START_TIME + FRAME_DURATION * NB_FULL_CYCLES

tagDic = {8193:1, 8200:2, 8201:3, 8214:4, 8225:5, 8197:6, 8228:7, 8212:8 , 8224:9, 8208:10}
color = ['b','g','r','c','m','y','k', '#00FF00','#e76c36','#eeec36','#2ECC71' ]

def extractData(fileName, time, tagLiveList, tagLiveSlot, planedSlotList):
    numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
    rx = re.compile(numeric_const_pattern, re.VERBOSE)
    with open(file) as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    for ele in content:
        dis = rx.findall(ele)
        idx = tagDic[int(dis[1])]
        time.append(float(dis[0]))
        tagLiveSlot.append(int((dis[-1])))
        tagLiveList.append(idx)
        temp = []
        for i in range(2,len(dis)):
            if int(dis[i])!= 255:
                temp.append(int(dis[i]))
            else: break
        if not planedSlotList[idx] or planedSlotList[idx][-1] != temp:
            planedSlotList[idx].append(temp)


def shiftTime(time, tagLiveSlot):
    startTime = time[0] - tagLiveSlot[0] * TASK_SLOT_DURATION / 1000
    for i in range(len(time)):
        time[i] -= startTime


def draw(planedSlotList, tagLiveList, time ):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    ax.set_xlabel("time/slot")
    xTime = [0+x*TASK_SLOT_DURATION/1000 for x in range(NB_TASK_SLOTS)]
    ax.xaxis.set_ticks(xTime)
    ax.grid(linewidth=0.2)
    plt.xticks(rotation=75)
    plt.grid(True)

    keys = list(tagDic.keys())
    dicValues = list(tagDic.values())

    for i in range(1,len(planedSlotList)):
        temptime = []
        tempy = []
        if planedSlotList[i]:
            for ele in planedSlotList[i][0]:
                temptime.append(ele*TASK_SLOT_DURATION/1000.+0.01)
                tempy.append(-2.)
            plt.scatter(temptime,tempy,marker='s',c=color[i],s=30,label=keys[dicValues.index(i)])
    for i in range(len(time)):
        tempx = time[i]%FRAME_DURATION
        tempy = time[i]/FRAME_DURATION
        plt.scatter([tempx],[tempy],marker=9,c=color[tagLiveList[i]],s=20)

    ax.legend(loc='center left',prop={'size': 6})
    plt.ylabel("Time base (seconds)")
    plt.savefig("./testing/result.pdf")

if __name__ == '__main__':

    file = './data.txt'
    time = []
    tagLiveList = []
    tagLiveSlot = []
    planedSlotList = [[] for i in range(len(color))] # The scheduling of tags, [[],]

    extractData(file,time,tagLiveList,tagLiveSlot,planedSlotList)
    shiftTime(time,tagLiveSlot)
    draw(planedSlotList, tagLiveList, time)
