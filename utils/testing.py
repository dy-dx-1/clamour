import matplotlib

matplotlib.use("WebAgg")

# Configure WebAgg BEFORE importing pyplot.
matplotlib.rcParams["webagg.address"] = "127.0.0.1"
matplotlib.rcParams["webagg.port"] = 8988
matplotlib.rcParams["webagg.open_in_browser"] = False

import matplotlib.pyplot as plt
import numpy as np

print("Backend:", matplotlib.get_backend())
print("Address:", matplotlib.rcParams["webagg.address"])
print("Port:", matplotlib.rcParams["webagg.port"])

x = np.linspace(0, 10, 100)
plt.plot(x, np.sin(x))
plt.title("WebAgg test")

print("Calling plt.show()...")
plt.show()

print("WebAgg exited.")