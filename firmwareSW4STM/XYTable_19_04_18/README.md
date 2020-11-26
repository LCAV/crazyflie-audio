From Mac OS, open a terminal and locate the correct USB peripheral:
ls /dev/tty.*

Then use screen to open a Serial communication with the board.
screen /dev/tty.usbmodem1413 115200

The key map is printed when the board is started. If it does not show when executing the screen function. Just reset the nucleo with the black button.

Note that if you want to use windows, you can also, just use an other serial utility with the same baudrate (115200)...

Example: 

X-NUCLEO-IHM02A1
 -------------------------------------------
 Dual L6470 Expansion Board for STM32 NUCLEO
 Stacked on NUCLEO-F401RE
 X-CUBE-SPN2 v1.0.0
 STMicroelectronics, 2015, Edited by Adrien Hoffet, 12.2017
 Xmotor: Q,W,E/A,S,D (FORWARD/BACKWARD), (50mm, 5mm, 0.5mm)
 Ymotor: P,O,I/L,K,J (FORWARD/BACKWARD), (50mm, 5mm, 0.5mm)
 ZERO: Z (SOFTSTOP, ZERO COUNTER)


Wait end of movement..., current position: x = 0 [mm*10], y = 50[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = 550[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = 0[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = 0[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = 0[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -5[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -10[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -15[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -20[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -25[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -30[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -35[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -40[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -45[mm*10]
Wait end of movement..., current position: x = 0 [mm*10], y = -50[mm*10]
