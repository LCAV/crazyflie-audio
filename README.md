# Audio processing for Crazyflie drone

This repository contains the hardware files, firmware and other documentation for the audio deck of the Crazyflie drone. 

## Overview

This repository is part of the experimental framework described in the [paper](https://doi.org/10.1109/LRA.2022.3194669):
```
F. Dümbgen, A. Hoffet, M. Kolundžija, A. Scholefield, and M. Vetterli, "Blind as a bat: audible 
echolocation on small robots", IEEE Robotics and Automation Letters (Early Access), 2022.
```

The framework includes all components listed below. The components are kept modular so that researchers may focus only on what's relevant to them. For instance, to recreate the audio extension deck and perform more research on audio-based navigation on drones, only repository 2. is necessary; to run the audio-based algorithms in ROS but for a different robot, only repository 1. is enough as a starting point.      

1. [ROS processing pipeline](https://github.com/LCAV/audioROS) for processing audio for tasks such as obstacle detection and sound source localization. 
2. [Audio deck firmware](https://github.com/LCAV/crazyflie-audio) (**this repository**), which also includes the PCB files for reproducing the audio deck. 
3. [Public datasets](https://github.com/LCAV/audio-localization-dataset), for audio-based localization on the e-puck2 robot and the Crazyflie drone. 
4. [Crazyflie firmware](https://github.com/LCAV/crazyflie-firmware) (fork from official vendor firmware), with an added [audio deck driver](https://github.com/LCAV/crazyflie-firmware/blob/master/src/deck/drivers/src/audio_deck.c).
5. [Gtsam extension](https://github.com/duembgen/gtsam) for performing factor-graph inference using the echolocation measurements. 

## Contents

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

[1] [RA-L paper](https://doi.org/10.1109/LRA.2022.3194669) (main reference)
```
F. Dümbgen, A. Hoffet, M. Kolundžija, A. Scholefield, and M. Vetterli, "Blind as a bat: audible 
echolocation on small robots", IEEE Robotics and Automation Letters (Early Access), 2022. 
```

[2] [Ph.D. dissertation](https://infoscience.epfl.ch/record/290057) (including additional methods and experimental analysis):  
```
F. Dümbgen, "Blind as a bat: spatial perception without sight", Ph.D. disseration, 
École Polytechnique Fédérale de Lausanne, 2021.
```

