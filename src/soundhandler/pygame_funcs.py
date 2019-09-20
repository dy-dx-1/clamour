from __future__ import print_function
import os
import pygame
import fnmatch
import math
import queue
import multiprocessing as mp
from time import sleep
from random import randint, choice
import json
import numpy as np

# variable needed to find the right sphere and to play them accordingly to the user position
PLACEHOLDER = "xxx_yyy_zzz"  # file name gabarit exemple: xxx_yyy_zzz_HH_FF_TTTTT
SOUND_XYZ_FILES_JSON = '../sound/xyz_files.json'
PATH = "../../chambord_flacs/" #ExportsFactices/" #Flac/"  # ExportsFactices/" #Flac/"
EXTENSION = ".flac"
ORIGIN = [28, 4, 1959]

def convert_xyz_to_indexes(x, y, z):
    return "{}_{}_{}".format(
        int(round((x - ORIGIN[0]) / 30))
        , int(round((y - ORIGIN[1]) / 30))
        , int(round((z - ORIGIN[2]) / 30))
    )

class PyGameManager(object):
    def __init__(self, nb_channels=10):
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096)
        # create multiple channel for simultanous playback
        pygame.mixer.set_num_channels(nb_channels)

        self.nb_channels = nb_channels
        self.channels = []
        for index in range(nb_channels):
            self.channels.append([pygame.mixer.Channel(index), None, -1])

        self.incr = 0
        self.lastSound = ""

        self.patternChord = "*" + PLACEHOLDER + "*." + EXTENSION

        self.position_old = None
        self.lastSound = ""
        with open(SOUND_XYZ_FILES_JSON, 'r') as fp:
            self.xyz_files = json.load(fp)

        print(len(self.xyz_files))

    def buildFileName(self, coor) -> str:
        indexes = convert_xyz_to_indexes(coor.x, coor.y, coor.z)
        return self.xyz_files.get(indexes, "")

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

                    sleep(offset_time)
#                    if (index % 2) == 0:
#                        channel.set_volume(1,0)
#                    else:
#                        channel.set_volume(0, 1)

                    thePath = PATH + track
                    print("Play ", thePath, index, " on channel ", (index % 2))
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

        self.soundPlayer(self.patternChord)

    def cyclic_call(self, position):
        self.patternChord = self.buildFileName(position)

        if (position.x != 0 or position.y != 0 or position.z != 0) and self.patternChord:
            self.buildPlayList()
        else:
            print("No file found on position: X",position.x," Y:",position.y," Z:",position.z, "(",self.patternChord,")")


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

    # p.cyclic_call(Position(28, 124, 778))

    # for index_x in range(-3150, 3101, 24):
    #     for index_y in range(-3206, 3206, 24):
    for index_x in range(-1650, 650, 21):
        for index_y in range(-143, 912, 21):
            p.cyclic_call(Position(int(index_x), int(index_y), 2155))
            sleep(0.1)
