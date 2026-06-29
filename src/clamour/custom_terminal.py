"""
This file defines the printing function to use in Clamour. 
It uses the rich module for clearer printing and allows for clearer terminal output. 
"""
from rich.console import Console
from rich.text import Text 
from typing import Literal

from .config import GEN_MSGS, DEVICE_MSGS, TDMA_MSGS, LOC_MSGS

console = Console() 

def print(text:str, status:Literal["ok", "info", "error"], type:Literal["gen", "device", "tdma", "loc"]): 
    """ 
    Customized print command for clamour. 
    
    ARGS: 
        - text:   String to print out 
        - status: Defines the type of update 
        - type:   Specifies the affected functionality 
    """
    # TODO NOTE: Not supporting [WARNING], not used enough. When replacing these prints, add something to signal importance of these. 
    if type=="gen" and not GEN_MSGS: 
        return 
    if type=="device" and not DEVICE_MSGS: 
        return 
    if type=="tdma" and not TDMA_MSGS: 
        return 
    if type=="loc" and not LOC_MSGS: 
        return 

    if type == "gen": 
        text_style = "white"
        emoji = "📢"  
    elif type == "device": 
        text_style = "bright_blue"
        emoji = "⚙️" 
    elif type == "tdma": 
        text_style = "hot_pink"
        emoji = "📶" 
    elif type == "loc": 
        text_style = "dark_orange"
        emoji = "📍" # or 🌎? 
    else: 
        raise ValueError("Unknown value for arg status. Needs to correspond to print function in console_themes.py")

    body = Text(text, style=text_style)

    if status == "ok": 
        header = Text("[OK]    ", style="green1") 
    elif status == "info": 
        header = Text("[INFO]  ", style="yellow2") #or cyan3 or cyan2
    elif status == "error": 
        header = Text("[ERROR] ", style="red1") 
    else: 
        raise ValueError("Unknown value for arg status. Needs to correspond to print function in console_themes.py")

    header.append_text(Text.from_markup(f"{emoji}  : "))
    header.append_text(body) 

    console.rule(style="white") 
    console.print(header) 

if __name__ == "__main__": 
    #console.rule(style="white") 
    # For testing the print outputs 
    print("Clamour.keep_alive(): A process that needs to be kept alive died and will be restarted. Error: Test", "error", "gen")
    print("Process completed without exception", "ok", "gen") 
    print("This is error in PozyxTag.receive_data, RxInfo crashes: hello", "error", "loc") 
    print("This is an information message", "info", "tdma")
    print("This is a loc message", "info", "loc")
    print("This is a devicemanaget message", "info", "device")