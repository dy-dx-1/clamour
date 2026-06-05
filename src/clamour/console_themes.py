"""
This file defines the printing function to use in Clamour. 
It uses the rich module for clearer printing and allows for clearer terminal output. 
"""
from typing import Literal
from .config import SUPPRESS_ALL_MSGS, DEVICE_MSGS, TDMA_MSGS, LOC_MSGS

def print(text:str, status:Literal["ok", "info", "error"], type:Literal["device_mgt", "tdma", "localization"]): 
    """ 
    Customized print command for clamour. 
    Args:
        text:   String to print out 
        status: Defines the type of update 
        type:   Specifies the affected functionality 
    """
    if SUPPRESS_ALL_MSGS: 
        return 
    
    if status == "ok": 
        text = "[OK] " + text 
    elif status == "info": 
        text = "[INFO] " + text 
    elif status == "error": 
        text = "[ERROR] " + text 
    else: 
        raise ValueError("Unknown value for arg status. Needs to correspond to print function in console_themes.py")
    
