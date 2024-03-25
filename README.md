Programmed GPIO LED components on the MKII BoosterPack to receive pulse width modulation signals from a 16-bit timer whose duty cycles were incremented to create a full spectrum of color emission on the board.


**************************************************************************************************************************************************************************************************
Notes made while designing  project:

Frequency with clock divider value is 20 = 50,000 Hz
Period = 2*10^-5 seconds

Minimum on/off frequency needed for human eye not to perceive led's blinking= 500 Hz
	- Period of this frequency= .002 seconds

Counter value that takes .002 seconds = 100



