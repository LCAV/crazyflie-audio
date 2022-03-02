# Audio processing for Crazyflie drone

This repository contains the hardware files, firmware and other documentation for the audio deck of the Crazyflie drone. 

## Overview

- `firmware/`: Firmware to run on the audio shield, which extracts and transmits microphone data to the Crazyflie PCB. 
- `hardware/`: *KiCad* PCB design files for audio shield.
- `python/`: Helper functions to generate hard-coded sweeps etc. 
- `flight_control/`: Simple scripts to fly the Crazyflie drone for testing.
- `doc/`: Some documentation and helpers.

## Audio deck driver

The driver of the audio deck, running on the Crazyflie, is available in [this fork](https://github.com/lcav/crazyflie-firmware) of the official firmware.

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

## References

Please refer to the below publications for more information.

```
F. Dümbgen, A. Hoffet, A. Scholefield, and M. Vetterli, "Blind as a bat: audible 
echolocation on tiny robots", submitted to IEEE/RSJ International Conference on 
Intelligent Robotis and Systems, 2022.
```

```
F. Dümbgen, "Blind as a bat: spatial perception without sight", Ph.D. disseration, 
École Polytechnique Fédérale de Lausanne, 2021.
```
