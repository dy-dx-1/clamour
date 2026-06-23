import sys
from pathlib import Path
from rich import print
import time 
import struct

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.bitcraze_tag import BitcrazeTag

with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9) as bc: 
    poll = bc.gen_message_header(4, 'POLL', 0xFF) 
    final = bc.gen_message_header(4, 'FINAL', 0xFF)

    _, T1 = bc._dw.transmit(poll, True) 
    answ = bc._dw.listen(0.1) # TODO add check for source/dest,  answ and twrseq 
    # HERE read R2, add it as a parameter in .listen to allow return of exact time 
    _, T3 = bc._dw.transmit(final, True) 
    report = bc._dw.listen(0.1) # TODO add check for source/dest, report and twrseq 
    # report[21] should be 0x4 for report 
    # report[22] should be twr_seq 
    time_info = report[23:38] # the rest is pressure related info, don't care 
    # Unpack the data
    R1, T2, R3 = struct.unpack('<5s5s5s', bytes(time_info))
    print(f"{T1=}")
    print(f"{T3=}")
    print(f"{R1=}")
    print(f"{T2=}")
    print(f"{R3=}")
    # TODO add wraparound helper 
    #T_r1 =  R2 - T1 
    #T_r2 =  R3 - T2 
    #T_rp1 = T2 - R1 
    #T_rp2 = T3 - R2 
    #tof = ((T_r1 * T_r2) - (T_rp1 * T_rp2)) / (T_r1+T_r2+T_rp1+T_rp2)