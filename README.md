# Audio processing for Crazyflie drone

This repository contains code, data, and other material for audio-based algorithms running on the Crazyflie drone. 

*Created on July 27, 2020.*

## Overview

- `firmware/`: firmware to run on the audio shield, which processes the microphone signals and streams the result to the Crazyflie PCB. 
- `hardware/`: *KiCad* PCB design files for audio shield.
- `data/`: audio recordings, recorded with the audio shield, measurement microphones, etc.

## Contributors 

In alphabetical order:

- Isaac Bernardino Dinis 
- Frederike Dümbgen
- Adrien Hoffet

## License

All material in this repo is provided with the MIT license.

## EEPROM

Parameters to set in `crazyflie-lib-python/examples/write_ow.py`: 

```
mems[0].vid = 0xBC
mems[0].pid = 0xFF
board_name_id = OWElement.element_mapping[1]
board_rev_id = OWElement.element_mapping[2]
mems[0].elements[board_name_id] = 'Audio Shield'
mems[0].elements[board_rev_id] = 'V2'
```

