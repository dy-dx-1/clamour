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

def compute_clock_delta(t2, t1):
    TICK_DELTA_MASK = (1 << 40) - 1
    return (t2-t1) & TICK_DELTA_MASK

with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8, preamble_length=128, preamble_code=9) as bc: 
    poll = bc.gen_message_header(4, 'POLL', 0xFF) 
    final = bc.gen_message_header(4, 'FINAL', 0xFF)

    _, T1 = bc._dw.transmit(poll, True) 
    answ, R2 = bc._dw.listen(0.1, ranging=True) # TODO add check for source/dest,  answ and twrseq 
    _, T3 = bc._dw.transmit(final, True) 
    report = bc._dw.listen(0.1) # TODO add check for source/dest, report and twrseq 
    # report[21] should be 0x4 for report 
    # report[22] should be twr_seq 
    time_info = report[23:38] # the rest is pressure related info, don't care 
    # Unpack the data
    R1, T2, R3 = struct.unpack('<5s5s5s', bytes(time_info)) 

    T_r1 =  compute_clock_delta(R2, T1)
    T_r2 =  compute_clock_delta(int.from_bytes(R3, byteorder='little'), int.from_bytes(T2, byteorder='little'))
    T_rp1 = compute_clock_delta(int.from_bytes(T2, byteorder='little'), int.from_bytes(R1, byteorder='little'))
    T_rp2 = compute_clock_delta(T3, R2)
    tof_ticks = ((T_r1 * T_r2) - (T_rp1 * T_rp2)) / (T_r1+T_r2+T_rp1+T_rp2)
    distance = tof_ticks * bc._dw.TIME_UNIT * 299_792_458

    print("R1 =", hex(int.from_bytes(R1,'little')))
    print("T2 =", hex(int.from_bytes(T2,'little')))
    print("R3 =", hex(int.from_bytes(R3,'little')))

    print("T1 =", hex(T1))
    print("R2 =", hex(R2))
    print("T3 =", hex(T3))

    print("T_r1 =", T_r1)
    print("T_r2 =", T_r2)
    print("T_rp1 =", T_rp1)
    print("T_rp2 =", T_rp2)

    print("tof_ticks =", tof_ticks)
    print(f"{distance=}") 