"""
This file defines the printing function to use in Clamour. 
It uses the rich module for clearer printing and allows for clearer terminal output. 
"""
from rich.console import Console
from rich.text import Text 
from rich.emoji import Emoji 
from typing import Literal

#from .config import GEN_MSGS, DEVICE_MSGS, TDMA_MSGS, LOC_MSGS

console = Console() 

def print(text:str, status:Literal["ok", "info", "error"], type:Literal["gen", "device", "tdma", "loc"]): 
    """ 
    Customized print command for clamour. 
    Args:
        text:   String to print out 
        status: Defines the type of update 
        type:   Specifies the affected functionality 
    """
    # TODO: add config bypasses 
    if type == "gen": 
        text_style = "white"
        emoji = ":envelope:" 
    elif type == "device": 
        text_style = "bright_blue"
        emoji = ":gear:" 
    elif type == "tdma": 
        text_style = "hot_pink"
        emoji = ":antenna_bars:" 
    elif type == "loc": 
        text_style = "dark_orange"
        emoji = ":round_pushpin:" 
    else: 
        raise ValueError("Unknown value for arg status. Needs to correspond to print function in console_themes.py")

    body = Text(text, style=text_style)

    if status == "ok": 
        header = Text("[OK] ", style="bright_green") 
    elif status == "info": 
        header = Text("[INFO] ", style="bright_yellow") 
    elif status == "error": 
        header = Text("[ERROR] ", style="bright_red") 
    else: 
        raise ValueError("Unknown value for arg status. Needs to correspond to print function in console_themes.py")

    header.append_text(Text.from_markup(f"{emoji}  : ")) # NOTE: try.strip on emoji? 
    header.append_text(body) 

    console.print(header) 

if __name__ == "__main__": 
    console.rule(style="white") 
    # For testing the print outputs 
    print("Clamour.keep_alive(): A process that needs to be kept alive died and will be restarted. Error: Test", "error", "gen")
    print("Process completed without exception", "ok", "gen") 
    print("This is error in PozyxTag.receiveData, RxInfo crashes: hello", "error", "loc") 
    print("This is an information message", "info", "tdma")
    print("This is a loc message", "info", "loc")
    print("This is a devicemanaget message", "info", "device")