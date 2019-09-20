import json
import pygame
import numpy as np
from pypozyx import Coordinates
from random import randint
from time import sleep

from contextManagedQueue import ContextManagedQueue
from messages import SoundMessage

# variable needed to find the right sphere and to play them accordingly to the user position
PLACEHOLDER = "xxx_yyy_zzz"  # file name template example: xxx_yyy_zzz_HH_FF_TTTTT
PATH = "../../chambord_flacs/"
EXTENSION = ".flac"


class SoundManager(object):
    def __init__(self, sound_queue: ContextManagedQueue, nb_channels=10):
        pygame.mixer.init(frequency=44100)
        pygame.mixer.set_num_channels(nb_channels)  # create multiple channel for simultaneous playback

        self.sound_queue = sound_queue
        self.channels = []
        for index in range(nb_channels):
            self.channels.append([pygame.mixer.Channel(index), None, -1])

        self.incr = 0
        self.patternChord = "*" + PLACEHOLDER + "*." + EXTENSION
        self.lastSound = ""

        self.files_dict = None
        self.make_dict()

    def make_dict(self) -> None:
        with open('../sound/files.json') as fp:
            self.files_dict = json.load(fp)

        print(len(self.files_dict))
        arr = np.array([self.files_dict[key] for key in self.files_dict])
        print(np.amax(arr, axis=0))
        print(np.amin(arr, axis=0))

    def buildFileName(self, coordinates: Coordinates) -> str:
        for fn, bounds in self.files_dict.items():
            if (coordinates.x >= bounds[0]) and (coordinates.x <= bounds[1]):
                if (coordinates.y >= bounds[2]) and (coordinates.y <= bounds[3]):
                    if (coordinates.z >= bounds[4]) and (coordinates.z <= bounds[5]):
                        print(fn, bounds)
                        return fn
        return ""

    def soundPlayer(self, track):
        # loop trough the channel to find an available one and use it to play the track
        if track != self.lastSound:
            self.lastSound = track
            for index, channel_tuple in enumerate(self.channels):
                offset_time = randint(80, 200) * 0.001

                channel = channel_tuple[0]

                if channel.get_busy() != 1:
                    sleep(offset_time)

                    thePath = PATH + track
                    print("Play ", thePath, index, " on channel ", (index % 2))
                    channel_tuple[1] = pygame.mixer.Sound(thePath)
                    try:
                        channel.play(channel_tuple[1])
                    except:
                        pass
                    break

    def buildPlayList(self):
        """This function switches the different stats and call the sound that must be played."""

        while pygame.mixer.get_busy == 1:  # wait for the last instance to finish
            print('waiting for channel to be ready')

        self.soundPlayer(self.patternChord)

    def cyclic_call(self, position: Coordinates):
        self.patternChord = self.buildFileName(position)

        if (position.x != 0 or position.y != 0 or position.z != 0) and self.patternChord:
            self.buildPlayList()
        else:
            print("No file found on position: X", position.x, " Y:", position.y, "Z:", position.z, "(", self.patternChord, ")")

    def play(self, path):
        self.patternChord = path
        self.buildPlayList()

    def run(self) -> None:
        while True:
            if not self.sound_queue.empty():
                message = SoundMessage.load(*self.sound_queue.get_nowait())
                scaled_position = Coordinates(message.position.x / 10, message.position.y / 10, message.position.z / 10)
                self.cyclic_call(scaled_position)
