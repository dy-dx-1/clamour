import glob
import os
from numpy import array, array_split, append
from concurrent.futures import process as mp
from pydub import AudioSegment

WAV_DIRECTORY = "/media/david/OLKHON 4TB/NEW_WAV"

def process_directory(directory: str) -> None:
    for _, _, files in os.walk(f"{WAV_DIRECTORY}/{directory}"):
        for file in files:
            filename, ext = os.path.splitext(file)
            if ".wav" in ext:
                newfile = filename.split('spharm_', 1)[-1] + ".flac"
                #print(newfile)
                if not os.path.exists(os.path.join("/home/david/new_chambord_flacs/", newfile)):
                    try:
                        song = AudioSegment.from_wav(f"{WAV_DIRECTORY}/{directory}/{file}")
                    except:
                        try:
                            song = AudioSegment.from_file(f"{WAV_DIRECTORY}/{directory}/{file}", 'aiff')
                        except:
                            print("Wrong here")
                    
                    if 'song' in locals():
                        try:
                            song.export(os.path.join("/home/david/new_chambord_flacs/", newfile),format = "flac")
                        except:
                            print("Wrong here too")
                else:
                    print("Already converted!")


def process_batch(directories: array) -> None:
    for directory in directories:
        process_directory(directory)


def main():
    directories = array([])

    for _, d, _ in os.walk(f"{WAV_DIRECTORY}"):
        directories = append(directories, d)

    with mp.ProcessPoolExecutor() as executor:
        executor.map(process_batch, array_split(directories, 4))


if __name__ == "__main__":
    main()
