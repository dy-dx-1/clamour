"""
Testing bitcraze tag implementation 
"""
import sys
from pathlib import Path

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
from src.clamour.interfaces.bitcraze_tag import BitcrazeTag 

bc = BitcrazeTag() 