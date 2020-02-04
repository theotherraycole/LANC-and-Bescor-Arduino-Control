# LANC-and-Bescor-Arduino-Control
Control a Bescor MP101 pan/tilt head and a LANC camera via an Arduino UNO using a Wii nunchuk

You will need the nunchuk library from http://www.gabrielbianconi.c
Connect the nunchuk according to the library's instructions.

Connect the pan/tilt controls per the sketch (I'll add more detailed instructions soon). Left/right and up/down must go to the pins shown in the code (don't change the pins). Only the 4 directional wires are used.

Connect LANC ground to ground and LANC signal to Pin 2. Voltage wire will not be used. My camera is 5V and so I just connect it directly. Feel free to add overvoltage protection circuitry...I likely will do that to mine eventually.

I power it off of the USB connection connected to my computer.

The LANC code is driven by interrupts using Timer 0. I have not observed any significant impact to delay/micros/millis as it allows the timer to overflow as normal.

This code is in use on a Canon Vixia HF G21 camera (NTSC). I make no guarantee it'll work for anyone else.

Nunhuk use:
  Normal mode:
    Joystick left, right, up, down for pan/tilt.
    Z Button down: Zoom in/out. Hold nunchuk and raise your arm up or down to zoom out/in. The more it tilts, the faster the zoom.
    C Button down: Zoom in/out. Hold nunchuk and twist your arm left/right to roll. The more roll, the faster the zoom.

  Navigation mode:
    Joystick left/right/up/down navigates menu
    Z button selects
    c button is Menu button

Both buttons down: Hold for 5 seconds to toggle power.
                   Hold for only a few seconds with joystick up to go to camera navigation mode. It will toggle camera display (assumes display was off already) and joystick acts using Navigation mode controls. Hold with joystick up again to toggle back to normal mode.
                   Hold for only a few seconds with joystick down to toggle camera display

You will want to adjust the CENTER defines based upon your nunchuk's readings. I also had to adjust the UP range on mine as the one I have will not go to a full up reading for some reason (cheap knockoff, I'm sure).
