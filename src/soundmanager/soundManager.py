import json
import pygame
from pypozyx import Coordinates
from random import randint
from time import sleep
import sys, os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from contextManagedQueue import ContextManagedQueue
from contextManagedProcess import ContextManagedProcess
from messages import SoundMessage

# variable needed to find the right sphere and to play them accordingly to the user position
PLACEHOLDER = "xxx_yyy_zzz"  # file name template example: xxx_yyy_zzz_HH_FF_TTTTT
PATH = "../../chambord_flacs/"
SOUND_XYZ_FILES_JSON = '../sound/xyz_files.json'
EXTENSION = ".flac"
ORIGIN = [28, 4, 1959]


class SoundManager(object):
    def __init__(self, sound_queue: ContextManagedQueue, nb_channels=10):
        pygame.mixer.init(frequency=44100)
        pygame.mixer.set_num_channels(nb_channels)  # create multiple channel for simultaneous playback

        self.sound_queue = sound_queue
        self.channels = []
        for index in range(nb_channels):
            self.channels.append([pygame.mixer.Channel(index), None, -1])

        self.incr = 0
        self.pattern_chord = "*" + PLACEHOLDER + "*." + EXTENSION
        self.position_old = None
        self.last_sound = ""

        with open(SOUND_XYZ_FILES_JSON) as fp:
            self.xyz_files = json.load(fp)

        print(len(self.xyz_files))

    def build_file_name(self, coordinates: Coordinates) -> str:
        return self.xyz_files.get(self.convert_coordinates_to_indexes(coordinates), "")

    def sound_player(self, track):
        # loop trough the channel to find an available one and use it to play the track
        the_path = PATH + track
        if track != self.last_sound:
            self.last_sound = track

            for index, channel_tuple in enumerate(self.channels):
                offset_time = randint(80, 200) * 0.001
                channel = channel_tuple[0]

                if channel.get_busy() != 1:
                    sleep(offset_time)

                    print("Play ", the_path, "---> on channel ", index)
                    channel_tuple[1] = pygame.mixer.Sound(the_path)
                    try:
                        channel.play(channel_tuple[1])
                    except:
                        print("FAILED on channel ", index)
                    break

    @staticmethod
    def convert_coordinates_to_indexes(coordinates: Coordinates):
        return "{}_{}_{}".format(int(round((coordinates.x - ORIGIN[0]) / 30)),
                                 int(round((coordinates.y - ORIGIN[1]) / 30)),
                                 int(round((coordinates.z - ORIGIN[2]) / 30)))

    def build_play_list(self):
        """This function switches the different stats and call the sound that must be played."""

        while pygame.mixer.get_busy == 1:  # wait for the last instance to finish
            print('waiting for channel to be ready')

        self.sound_player(self.pattern_chord)

    def cyclic_call(self, position: Coordinates):
        self.pattern_chord = self.build_file_name(position)

        if (position.x != 0 or position.y != 0 or position.z != 0) and self.pattern_chord:
            self.build_play_list()
        else:
            print("No file found on position:", position, "(", self.pattern_chord, ")")

    def play(self, path):
        self.pattern_chord = path
        self.build_play_list()

    def run(self) -> None:
        while True:
            if not self.sound_queue.empty():
                message = SoundMessage.load(*self.sound_queue.get_nowait())
                scaled_position = Coordinates(message.coordinates.x / 10,
                                              message.coordinates.y / 10,
                                              message.coordinates.z / 10)
                self.cyclic_call(scaled_position)

    def main(self):
        for posX in range(-1800, -1400, 10):
            for posY in range(-1800, -1400, 10):
                print(posX, posY, 1896)
                self.cyclic_call(Coordinates(posX, posY, 1896))
                sleep(0.1)

if __name__ == "__main__":
    sm = SoundManager(None)

    with ContextManagedProcess(target=sm.main) as p:
            p.start()
