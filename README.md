Programmed GPIO LED components on the MKII BoosterPack to receive pulse width modulation signals from a 16-bit timer whose duty cycles were incremented to create a full spectrum of color emission on the board.

Here are some additional notes I made while designing the project:

frequency with 20 clock divider = 50,000 Hz

period with 20 clock divider = 2*10^-5 seconds

minimum on/off frequency needed for human eye not to perceive led's blinking= 500 Hz
	- period of this frequency= .002 seconds

count of counter that takes .002 seconds = 100



