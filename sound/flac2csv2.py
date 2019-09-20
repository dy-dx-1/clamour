import json
import re

ORIGIN = [28, 4, 1959]

FILE_NAME_PATTERN = re.compile(r"X(?P<X>-?\d+)_Y(?P<Y>-?\d+)_Z(?P<Z>-?\d+).flac")


def convert_xyz_to_indexes(x, y, z):
    return "{}_{}_{}".format(
        int(round((x - ORIGIN[0]) / 30))
        , int(round((y - ORIGIN[1]) / 30))
        , int(round((z - ORIGIN[2]) / 30))
    )


if __name__ == '__main__':
    with open('files.json', 'r') as in_file:
        with open("xyz_files.json", "w") as out_file:

            indexes_filenames = {}
            max_x, max_y, max_z = 0, 0, 0
            filenames = json.load(in_file).keys()
            for filename in filenames:
                match = FILE_NAME_PATTERN.match(filename)

                max_x = max(max_x, int(match.group("X")))
                max_y = max(max_y, int(match.group("Y")))
                max_z = max(max_z, int(match.group("Z")))
                indexes = convert_xyz_to_indexes(
                    int(match.group("X"))
                    , int(match.group("Y"))
                    , int(match.group("Z"))
                )
                if indexes in indexes_filenames:
                    print(
                        "Duplicate location: {} has the same location as {} ({})"
                            .format(filename, indexes_filenames[indexes], indexes)
                    )
                indexes_filenames[indexes] = filename

            print(max_x, max_y, max_z)
            json.dump(indexes_filenames, out_file)
