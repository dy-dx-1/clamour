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
  - [TDMA Communication Cycle](#tdma-communication-cycle)
  - [Sound Playing Process](#sound-playing-process)

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
flowchart LR
    subgraph Tag["Tag (Parallel Processes)"]
        TDMA["TDMA Process<br/>(Slot coordination, discovery,<br/>ranging, logical clock & topology)"]
        EST["Localization Process<br/>(Factor Graph / EKF Estimator)"]
        SND["Sound Playing Process<br/>(Adaptive music / audio output)"]
    end

    EXT["Anchors, Neighboring Tags<br/>& UWB Radio Channel"]

    EXT <-->|"TDMA slots, TWR ranges,<br/>clock/delay & topology updates"| TDMA
    TDMA -->|"Distances, clock/delay,<br/>topology & updates"| EST
    EST -->|"Estimated position"| SND
```
---

## Communication Cycle

Each node operates as a state machine structured around a synchronized TDMA cycle : 

**Step 1 - Initialization:**

The device first start to wait a random delay, then attempts to detect other tags around itself. It then update itself and switch to the next state

**Step 2 - Synchronisation:**

This state prepares the tag for synchronization by converging the logical clock of the tags so that they can coordonnate their transmissions. After the preparations are done, the device switches to the Scheduling state

**Step 3 - Scheduling:**

This state attributes slots for each tag using the temporal information prepared during the synchronization state. When the planification window is finished, it switches to the Listen state. If an error message is returned, it switches to the Synchronization state.

**Step 4 - Listen:**

This state can be divided in four different outcomes. If a re-synchronisation is needed, the device switches to the Synchronisation state. If the TDMA cycle is over, it also switches to the Synchronization state. If were in the cycle and the current slot is owned by the device, it switch to the Task state. Finally, if were in the cycle but the current slot is not owned by the device, it stays in the Listen state.

**Step 5 - Task:**

This is state starts by discovering periodically the other devices. It also cleans the obsolete information by the mean of a garbage collector. After the other tags have been discovered, the program starts mesuring distances between devices and transmits the result. If the surroundings have changed, the device sends a message to the other devices about the change and then updates the logical clock and the delay with the surrounding device. The estimator receives : 
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