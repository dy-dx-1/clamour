# CLAMOUR

**Collaborative Localization Adapted from Modular Onboard UWB Ranger**

CLAMOUR is a collaborative localization system designed to power self-localizing audioguides for the [Chambord Castle Museum](https://www.chambord.org/en/discovering/the-castle-visit/) exhibition. Based on a visitor's real-time position within the castle, the audioguide dynamically generates adaptive music.

An initial prototype was developed in 2018. Further details on the prototype and underlying research are available in the [arXiv paper (1905.03247)](https://arxiv.org/abs/1905.03247).

This project is a joint collaboration by the following laboratories:
- **MIST lab** - Polytechnique Montréal
- **NXI Gestatio** - UQAM
- **INIT Robots** - ETS.
---
## Table of Contents

- [Overview & Key Concepts](#overview--key-concepts)
- [System Architecture](#system-architecture)
  - [Localization Process (Estimator)](#localization-process-estimator)
  - [Sound Playing Process](#sound-playing-process)
  - [TDMA Communication Cycle](#tdma-communication-cycle)
  

---

## Overview & Key concepts

The primary objective of CLAMOUR is to achieve **collaborative localization and tracking with minimal fixed infrastructure**

**UWB (Ultra-WideBand):**
UWB is a radio technology used here to measure distances between devices. These mesurements are then used to estimate positions. 

**Anchor:**
An anchor is a fixed reference point whose position is know. A device can measure its distance to an anchor to obtain information about its localisation. 

**Tag:**
An UWB device that the system aims to localize. it can also communicate with neighboring tags and contribute to their localization.

**TDMA (Time-Division Multiple Access):**
A channel access method where time is divided into discrete slots. Each tag transmits only during its assigned slot to prevent radio collisions with neighboring devices.

---
## System Architecture

At runtime, each tag continuously executes **three main processes in parallel**: **Localization (Estimator)**, **TDMA**, and **Sound Playing**. The diagram below illustrates how these processes interact and exchange data.

```mermaid
flowchart TD
    START(["start<br/>config.py"])

        EST["<b>State Estimation (EKF / FG)</b><br/>Fuse ranges, clock/delay, topology &amp; other<br/>sensor data (pedometer/IMU) if available"]

        TDMA["<b>TDMA</b><br/>TDMA cycle; take ranges<br/>&amp; update topology in Task"]

        SND["<b>Sound</b><br/>Play sound according to<br/>position / state"]

    STATE[["state<br/>(estimated position)"]]

    START --> EST

    START --> TDMA
    START --> SND

    TDMA -->|"topology + ranges"| EST
    EST --> STATE
    STATE --> SND

    EST -. "loop" .-> EST
    TDMA -. "loop" .-> TDMA
    SND -. "loop" .-> SND
```
---
## Localization Process (Estimator)
The localization process runs continuously as an estimator (such as a factor graph or Extended Kalman Filter (EKF)) that fuses spatial and temporal observations over time. It consumes outputs produced during the TDMA cycle:

Measured distances between the tag and fixed anchors

Measured distances between the tag and neighboring tags

Logical clock and delay estimates

Known network topology (and surrounding changes)

From these inputs, the estimator produces continuously updated position estimates for the tag (and collaborative state information).

## Sound Playing Process
The sound playing process runs concurently with localization and TDMA. Its roles is straightforward: it consumes position estimates from the localization estimator and generates a sounds depending on the result.


## TDMA Communication Cycle

Each node operates as a state machine structured around a synchronized TDMA cycle : 

**Step 1 - Initialization:**

The device first start to wait a random delay, then attempts to detect other tags around itself. It then update itself and switch to the next state

**Step 2 - Synchronisation:**

This state prepares the tag for synchronization by converging the logical clock of the tags so that they can coordonnate their transmissions. Each tags exchanges Sync messages to align their logical clock within a 15 ms threshold. Each tag computes the clock offset against received neighbor timestamps plus a small delay. Then, depending on the offset magnitude, the tag eitheir applies a direct offset correction or uses collaborative offset compensation over recent neighbor offsets. After the preparations are done, the device switches to the Scheduling state

**Step 3 - Scheduling:**

This state attributes slots for each tag using the temporal information (such as their logical clock, their synchronisation timestamp and the duration of the scheduling window computed from the discovered neighbor) prepared during the synchronization state. When the planification window is finished, it switches to the Listen state. If a scheduling error is returned, it switches to the Synchronization state.

**Step 4 - Listen:**

This state can be divided in four different outcomes. If a re-synchronisation is needed, the device switches to the Synchronisation state. If the TDMA cycle is over, it also switches to the Synchronization state. If were in the cycle and the current slot is owned by the device, it switch to the Task state. Finally, if were in the cycle but the current slot is not owned by the device, it stays in the Listen state.

**Step 5 - Task:**

This is state starts by discovering periodically the other devices and cleaning up obsolete neighbor entries via a garbage collector, which removes neighbors whose timestamp is older than 20 seconds. After the other tags have been discovered, the program starts mesuring distances between devices and transmits the result. If the surroundings have changed, the device sends a message to the other devices about the change and then updates the logical clock and the delay with the surrounding device. The estimator receives : 
- the distance mesured between the device and the anchors
- the distance mesured between the device and the other tags
- the logical clock and the delay
- the known topology of the network

```mermaid
stateDiagram-v2
    [*] --> Initialization
    Initialization --> Synchronization : Neighbors detected & state initialized
    Synchronization --> Scheduling : Logical clocks converged
    Scheduling --> Listen : Planning window finished
    Scheduling --> Synchronization : Scheduling error
    Listen --> Listen : Active cycle (other tag's slot)
    Listen --> Task : Active cycle (own slot)
    Listen --> Synchronization : Cycle ended or re-sync needed
    Task --> Listen : Ranging & estimation complete

```
---