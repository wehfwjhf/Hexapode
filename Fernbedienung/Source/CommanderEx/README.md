Warning
#######

This is a Work In Progress!  There are no warrantees or Guarantees 
of any type that this code is useable for anything.  But I hope it is…

The CommanderEx sketch in this directory is forked from the official Commander
sketch by Mike Ferguson.  You can find the Official commander sources up at
https://github.com/vanadiumlabs/arbotix/tree/master/Commander


Differences from Official
=========================

The original main difference I made in the sources was to change the range
of values that is returned by the Joysticks.  In the Original, the range
was restricted to about +-102.  As I understood it, the main reason for
limiting these values was to not allow multiple hex values of 0xff to be
output in the packet.  Having two 0xff bytes in a row was a way to detect
the start of a packet. 

This version, limits the values to more or less -127 - 126.  In addition this version
averages the last 8 values read in for the joysticks as to do a little smoothing.

Additional Features
===================

There are a few other recent changes to the code that add a few extra features, most
of them can be removed by commenting out #defines in the sketch.

Optional Initialize XBee
------------------------

After seeing several people with issues where their XBees came unpaired and still communicating
at 9600 baud, I decided I would try to detect this in the code and configure the XBee.  Currently
the detection is pretty simple.  I try to enter into command mode +++ at our default baud rate of
38400, if this fails, I then try at 9600. If this succeeds, I assume a virgin XBee and I reconfigure
it to 38400 and set the MY and DL and network values.  The values I use are in defines that can be
changed. 
```
#define DEFAULT_MY 0x102  // Swap My/DL on 2nd unit
#define DEFAULT_DL 0x101
#define DEFAULT_ID 0x3332
```

The Init code also sets the guard time on the XBee to a very short period (10ms), so that I now
first try to talk to the XBee using this timeout.  This has removed the need to write stuff out
to the EEPROM, which my previous version did.  LAs such I no longer need the L1 button test to 
bypass the bypass... 

Note: Soon I will also introduce the same code in my Phantom_Phoenix project to properly configure
the XBee on the actual robot as well. 

Optional Map to the Original Values
-----------------------------------

I added optional code that if you hold down the R6 button at startup, the values returned by the
joysticks, will be back in the +-102 range.  I did this so, I can experiment going back and forth
between my Phantom_Phoenix code base and the Nuke code base.

Debug Support
-------------

This has always been there, but there is optional debug support built in the code base that if
you hold down the RT and LT buttons at startup time, the program will enter into debug mode.  In this
mode instead of outputting normal packets, it will print out a text version of the packet.  Recently
changed the output of the Joysticks from the 0-254 range to print out a signed value of the byte minus 128
as to match how the values are used in the Commander library.
