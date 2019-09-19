import os
import numpy as np
import json


files = {}
# r=root, d=directories, f = files
for r, d, f in os.walk("/home/david/chambord_flacs"):
    for file in f:
        if '.flac' in file:
            filename, file_extension = os.path.splitext(file)
            pos = filename.split('_')
            X = int(pos[0][1:])
            Y = int(pos[1][1:])
            Z = int(pos[2][1:])
            files[file]=[X, X+30, Y, Y+30, Z, Z+30]

arr = np.array([files[key] for key in files])
print(np.amax(arr, axis=0))
print(np.amin(arr, axis=0))

json = json.dumps(files)
f = open("files.json","w")
f.write(json)
f.close()