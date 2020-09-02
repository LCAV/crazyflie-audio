# Recordings with measurement mics

Time: Thursday 16.07.2020

Recorded by: Isaac Bernardino

Place: Audio room in INR018. 

Hardware: 1 speaker (MODEL?) and 4 measurement microphones, mounted on the drone outside of the propellers. The drone is standing 1.965 m from the speaker.

Protocol: 

4 measurements microphones equally spaced (see schema.png) record at 48000 Hz. The recordings are stored in a numpy array. 
First column corresponds to first microphone (see schema.png) and so on. 
Using the debug mode we pause the programm when the system is stable and export the buffers in a binary file. 
The propellers thrust is set to 43000.
A 200Hz or white noise external source is used. 

Contents: 

- `white_noise/` recordings with white noise source.
- `200Hz/` recordings with 200 Hz sine wave.
- `propellers_only/' recordings without source. 



