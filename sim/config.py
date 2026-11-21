"""
Simulation-specific configuration. 
Tags all run exactly the clamour code, so if you don't see a setting here, it's probably in clamour's config.py
"""
# Initial position of the tags {id: (x,y,z)} 
TAGS = {}

# Anchor configuration {id: (x,y,z)}, if empty, the value from clamour config is taken 
ANCHORS = {} 

# Selected map from maps/ 
MAP = None 

# Simulated UWB params 
UWB_RANGE = 30   # Range in meters after which UWB measures reliability drop out according to TODO 
UWB_NOISE = 0.15 # Std dev of a typical UWB measure in meters 
