# ENPM-662-Fall-2017-Semester-Project-
UR-5 Block Stacking in Simscape Multibody

the UR-5 arm is not perfectly accurate in its dimensions or its mass (unfortunately the CAD model I made did not import sucessfuly)
the model can do all of the required movements for transfering the block/stack to another location...
...but it cannot actually move the blocks because the contact forces are not set up correctly

the .m files are used to create the .mat files (run this first)
the .mat files contain the input signals for the Simscape Multibody models (just leave these alone)
the .slx files are the Simscape Multibody model used to generate the simulations (run this second)
note: if the possition of the block/stack moves, then the .m files must be updated to reflect that change

For those attempts (they do not work) to including contact forces...
Contact Forces Library for Simscape Multibody is required:
https://www.mathworks.com/matlabcentral/fileexchange/47417-simscape-multibody-contact-forces-library
