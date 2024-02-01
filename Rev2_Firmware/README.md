# Project for MPPT

## Outline
This code is for the MPPT board developed in Fall 2022/Spring 2023. It converts power from the array at ~20-70V into power for the battery at 90-150V. 

The board uses the following peripherals on the STM32G473VET6:

ADC 1,2,4,5 \
Comparators 1,2,3,7 \
OP-Amps: 1,2,4\
DACs: 1,3,4 \
Timers: 7, 8 \
FDCAN: 1 \
DMA: 1, 2


The general interaction is:
```
==================
|    Algorithm   | (incremental inductance) 
==================
         |                     ===============
         |       --------->>>>| Status LED's |
         |       ^             ===============
         |       |
===================            =========
|   Application   | ---------> | FDCAN |
===================            =========                         
         |       ^
         |       |             ==========
         | -------<<<----------| Safety |
         |                     ==========
         | 
===================
|  Hardware class |
===================
         |      |
         |      |              ===============
         |      \---<<<--------| Gate Driver |
         |                     ===============
==================     
|      ADC's     |     
==================     
         |      |     
         |      |              ===============
         |      \----<<<-------|     DMA     |
         |                     ===============
         |
         | (Hardware connection via PCB)
         |
==================
|     Op-Amp's   |
==================
               | 
               | (Hardware connection via PCB)
               |
               |               ===============
               \--->>>---------| Comparators |
                               ===============
                                     |
                                     | (Provides Voltage reference)
                                     |
                                ===========
                                |   DAC   |
                                ===========

```
## Testing
### MPPT testing on a power supply:
Use the Eload, 2 DC power supplies. Load set to max out at constant voltage mode at 45V, input supply set at 40V, output set. The Resistance is the toaster which is 5.2 ohm. 
