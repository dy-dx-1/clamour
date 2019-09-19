from __future__ import print_function
import os
import pygame
import fnmatch
import math
import queue
import multiprocessing as mp
from time import sleep
from random import randint
import json
import numpy as np

# variable needed to find the right sphere and to play them accordingly to the user position
PLACEHOLDER = "xxx_yyy_zzz"  # file name gabarit exemple: xxx_yyy_zzz_HH_FF_TTTTT

PATH = "/home/david/chambord_flacs/" #ExportsFactices/" #Flac/"  # ExportsFactices/" #Flac/"
EXTENSION = ".flac"

class Zone(object):
    def __init__(self, x_min, y_min, x_max, y_max):
        self.x_min = x_min * 1000.0
        self.x_max = x_max * 1000.0
        self.y_min = y_min * 1000.0
        self.y_max = y_max * 1000.0

    def is_in(self, x, y) -> bool:

        if self.x_min <= x and x < self.x_max:
            if self.y_min <= y and y < self.y_max:
                return True

        return False


class PyGameManager(object):
    def __init__(self, nb_channels=20):
        pygame.mixer.init(frequency=44100, size=16, channels=2, buffer=4096)
        # create multiple channel for simultanous playback
        pygame.mixer.set_num_channels(nb_channels)

        self.nb_channels = nb_channels
        self.channels = []
        for index in range(nb_channels):
            self.channels.append([pygame.mixer.Channel(index), None, -1])

        self.incr = 0
        self.lastSound = ""
        self.lastvolume = 100
        self.position_str = PLACEHOLDER

        self.patternChord = "*" + PLACEHOLDER + "*." + EXTENSION

        self.make_dict(PATH)

        self.position_old = None
        self.lastSound = ""

        self.position_x = 0.0
        self.position_y = 0.0

        self.zones = [Zone(0.0, 0.0, 5.0, 6.0), Zone(0.0, 6.0, 6.0, 12.0), Zone(0.0, 12.0, 6.0, 18.0),
                      Zone(0.0, 18.0, 6.0, 24.0), Zone(0.0, 24.0, 6.0, 30.0), Zone(5.0, 0.0, 9.0, 6.0),
                      Zone(5.0, 6.0, 9.0, 12.0), Zone(5.0, 12.0, 9.0, 18.0), Zone(5.0, 18.0, 9.0, 24.0),
                      Zone(5.0, 24.0, 9.0, 27.0), Zone(5.0, 27.0, 9.0, 30.0), Zone(9.0, 0.0, 11.0, 6.0),
                      Zone(9.0, 6.0, 11.0, 12.0), Zone(9.0, 12.0, 11.0, 18.0), Zone(9.0, 18.0, 11.0, 24.0),
                      Zone(9.0, 24.0, 11.0, 30.0), Zone(11.0, 0.0, 15.0, 6.0), Zone(11.0, 6.0, 15.0, 12.0),
                      Zone(11.0, 12.0, 15.0, 18.0), Zone(11.0, 18.0, 15.0, 24.0), Zone(11.0, 24.0, 15.0, 30.0),
                      Zone(15.0, 0.0, 20.0, 6.0), Zone(15.0, 6.0, 20.0, 12.0), Zone(15.0, 12.0, 20.0, 18.0),
                      Zone(15.0, 18.0, 20.0, 24.0), Zone(15.0, 24.0, 20.0, 30.0),]

    def make_dict(self,path)->None:
        # files = os.listdir(path)
        # files_dict = {}
        # for filename in files:
        #     files_dict[filename] = True
        with open('../sound/files.json', 'r') as fp:
            self.files_dict = json.load(fp)

        # arr = np.array([files_dict[key] for key in files_dict])
        # print(np.amax(arr, axis=0))
        # print(np.amin(arr, axis=0))

    def buildFileName(self,coor)->str:
        posX = math.floor(coor.x)
        posY = math.floor(coor.y)
        posZ = math.floor(coor.z)

        # print('Cell grid: ' + str(posX) + '(' + str(coor.x) + ') ' + str(posY) + '(' + str(coor.y) + ') ' + str(
        #     posZ) + '(' + str(coor.z) + ')')
        for fn, bounds in self.files_dict.items():
            if (posX > bounds[0] and posX < bounds[1]):
                if (posY > bounds[2] and posY < bounds[3]):
                    if (posZ > bounds[4] and posZ < bounds[5]):
                        return fn
        return ""

    def soundPlayer(self, track):

        # loop trought the channel to find an available one and use it to play the track
        if track != self.lastSound:
            self.lastSound = track
            for index, channel_tuple in enumerate(self.channels):
                # print('inside for', index)
                # side_select = (randint(0, 1) == 0) # Kept for stereo effect
                offset_time = randint(80, 200) * 0.001

                channel = channel_tuple[0]

                # print(theChannel_right, theChannel_left)
                if channel.get_busy() != 1:
                    print("In Zone {0}".format(self.find_zone()))

                    sleep(offset_time)
                    if (index % 2) == 0:
                        channel.set_volume(1,0)
                    else:
                        channel.set_volume(0, 1)

                    thePath = PATH + track
                    print("Play {0}({1}%) on channel {2}".format(thePath, index))
                    channel_tuple[1] = pygame.mixer.Sound(thePath)
                    try:
                        channel.play(channel_tuple[1])
                    except:
                        pass
                    break

    def buildPlayList(self):
        # this function switch the different stats and call the sound that must be played

        # wait for the last instance to finish
        while pygame.mixer.get_busy == 1:
            print('waiting for channel to be ready')

    def cyclic_call(self, position):
        self.position_x = position.x
        self.position_y = position.y
        self.position_str = self.buildFileName(position)
        self.patternChord = self.position_str + EXTENSION

#        print("X:",position.x," Y:",position.y," Z:",position.z)
        if self.position_str != PLACEHOLDER and (position.x != 0 or position.y != 0 or position.z != 0):
            self.buildPlayList()
        else:
            print("no position receive yet")


    def play(self, path):
        self.patternChord = path
        self.buildPlayList()

    def find_zone(self) -> int:

        for index, zone in enumerate(self.zones):
            if zone.is_in(self.position_x, self.position_y):
                return index

        return -1


if __name__ == "__main__":
    p = PyGameManager()
    class Position():
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    for index_x in range(0, 30000, 400):
        for index_y in range(0, 20000, 400):
            p.cyclic_call(Position(index_x, index_y, 400))
            sleep(0.1)
#    p.play("083_033_000.flac")
#    sleep(5)
