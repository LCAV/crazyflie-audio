# Audio shield measurements

Time: Tuesday 14.07.2020 and Wednesday 15.07.2020

Recorded by: Isaac Bernardino

Place: Audio room in INR018. 

Hardware: 1 speaker (MODEL?) and 4 MEMS microphones, mounted on the drone outside of the propellers. 
The drone is standing 1.925 m from the speaker.

Protocol: 

4 MEMS microphones are used and stored into 4 buffers containing 8060 samples, sampled at 32 kHz.
Using the debug mode we pause the programm when the system is stable and export the buffers in a binary file. The propellers thrust is set to 43000.

Contents: 

- `mX/` recordings when only motor `X` is running.
- `all_propellers/` recordings when all motors are running simultaneously.
- `external_source_only/` recordings when motors are off, only external source is playing. 
- `external_source_and_propellers/` all motors are on and external source is playing.



