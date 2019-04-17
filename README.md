# senior_project
********DISCLAIMER********
This code is in no way complete, I would NOT recommend building a unit for yourself at this point
This readme is a very basic tutorial, I have yet to test this on a fresh machine.
**************************

Intermediate soldering ability required

Required hardware:
1 - DE0-Nano
1 - STM32F4 Discovery
6 - 10kΩ resistors
4 - 1kΩ resistors
1 - Molex 52602-0579
2 - 2x20 female headers
2 - 2x25 (alternatively 2 2x20s and 2 2x5s) female headers
1 - Sparkfun SD/MMC breakout board (BOB-12941)
1 - 1x10 male header

Required software:
Keil uVision (tested with version 5.26.2.0)
Quartus Prime (tested with version 16.1)
NOTE: any version later than these listed will more than likely work; however, if any issues are encountered
fall back on these versions.

Building instructions:
1) download project as zip

2) Building the STM32F4 Discovery code & programming the board
  a) Download and install the Lite edition of Keil uVision
  b) Open the project file (senior_project_micro.uvprojx) found in senior_project_mirco/MDK_ARM, 
     at this point you will likely be prompted to download the compililer for the specific chip on
     board the STM32F4 Discovery. Do this.
  c) Press "F7" or "Project -> build target" to compile the code.
  d) Load the program onto the board by clicking the "Load" icon next to the project name.
  
3) Synthesizing the FPGA code & programming the board flash
  a) Download and install Quartus Prime from Intel FPGA
  b) Open the project file (senior_project_fpga.qpf) found in senior_project_fpga, again you'll likely
     be prompted to download a pack to support the hardware.
  c) Double click on "Compile Design"
  d) When this has completed go to File -> Convert Programming Files
  e) Choose JTAG indirect (.jic) for Programming File Type
  f) Choose EPCS64 for Configuration Device
  g) Click "flash loader" then "add device" then "Cyclone IV E" then "EP4CE22" then "OK"
  h) Click "SOF Data" then "Add File" then choose "senior_project_fpga.sof" then "open"
  i) Click "generate"
  j) Close the "Convert Programming File" window then go to tools -> programmer
  k) Click "add file" then choose the .jic file you generated
  l) Click start to program the DE0 Nano flash
  
4) Printing and populating PCB
  a) Download the PCB gerber files from here <link>
  b) You will need to send the PCB files for printing, I suggest JLCPCB as they are cheap and have relatively
     short shipping times for a Chinese printer. You will upload the gerber file .zip to their site and it 
     should handle the rest.
  c) 
