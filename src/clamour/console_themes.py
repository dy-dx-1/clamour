"""
This file defines the printing function to use in Clamour. 
It uses the rich module for clearer printing and allows for clearer terminal output. 
"""
import os 
import yaml 
from typing import Literal


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
CONFIG_PATH = os.path.join(PROJECT_ROOT, 'configuration', 'clamour_config.yaml') 

# TODO check if can move elsewhere to avoid multiple opens 
with open(CONFIG_PATH, 'r') as f: # NOTE: maybe interesting to add error handling/checks in future. For now assuming easy enough to read and debug. 
    cfg = yaml.safe_load(f) 

def print(text:str, status:Literal["ok", "info", "error"], type:Literal["device_mgt", "tdma", "localization"]): 
    """ 
    Customized print command for clamour. 
    Args:
        text:   String to print out 
        status: Defines the type of update 
        type:   Specifies the affected functionality 
    """
    if cfg['suppress_all_msgs']: 
        return 
    
    if status == "ok": 
        text = "[OK] " + text 
    elif status == "info": 
        text = "[INFO] " + text 
    elif status == "error": 
        text = "[ERROR] " + text 
    else: 
        raise ValueError("Unknown value for arg status. Needs to correspond to print function in console_themes.py")
    
