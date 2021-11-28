# StepperFlipClock

<img alt="Copal retrofit components" src="https://raw.githubusercontent.com/dpwe/StepperFlipClock/main/pics/copal_retrofit.png">

Design and firmware for a stepper motor retrofit to the often-failed AC synchronous flip clock motors.

Flip clocks, perhaps most iconic from the repeating opening sequence of the movie Groundhog Day, are delightful mechanisms for rendering the time as digits by flipping through a set of leaves on a rotating drum like a rolodex.

They were popular as a "modern" clock in the 1960s and 70s, based on part on high-performance, mass-produced mechanisms from the Copal corporation in Japan, which manufactured high-quality flip mechanisms integrated with AC-synchronous motors, able to keep very accurate time thanks to the provision of extremely stable 50/60 Hz AC power by electrical utilities in many parts of the world.

Flip clocks were supplanted by the even more modern 7 segment LED clocks, and quartz time references, in the 1970s and 80s, but they now have a retro appeal. Unfortunately, the synchronous AC motors, spinning valiantly 8 times a second for 50 years or more, have long exceeded their design life and are mostly no longer functional.  They are no longer manufactured and no significant replacement stock has turned up.

At the same time, cheap and precise stepper motors of similar size and shape *are* available.  However, these need an entirely different source of time reference, and indeed require a microcontroller to operate.

This repo provides the resources for my investigations into stepper motor retrofit replacements for the original Copal synchronous motors found in the flip clocks made by GE (and Copal) in the eary 1970s.  I include the electronics design, the firmware (as Arduino sketches), and some pictures of a couple of finished clock conversions.

# BurstClockSerial

This version of the firmware winds on the flip mechanism in "bursts", once per minute.  This makes sure the flaps fall within a fraction of a second of the actual minute change; a regular flip mechanism typically has a variation of +/- 10 sec or more for any particular minute due to mechanical variations in the flaps.

The code has settings for two variations of the Copal mechanism, one with 60 teeth per hour on the main wheel (as found in GE clock-radios such as the 7-4305), and one with 50 teeth per hour (as in the Copal 227 alarm clock).

# StepperClockCopal.fzz

This is the Fritzing source for the hardware setup for the stepper clock.  It's mainly the wiring diagram that I've tried to make legible, although the schematic should be correct also.  It shows which Arduino pins are connected to which pins on the DS3231 RTC board, on the ULN2003A stepper driver board, as well as other hardware including the frontlight LED, the piezo buzzer, and the alarm microswitch.

# 3Dprint

Directory containing the STL files I used for 3D printing pinion gears to mate the stepper motors with the flip clock mechanism.  `copol_pinion_1` works for the plastic-framed Copal mechanims.  `copol_pinion_2` attempted to scale down the teeth for the older, metal-framed mechanisms, but was a bit beyond the scale capabilities of the printing technology.  `copol_pinion_4b` converts the 5mm stepper shaft to the 2mm shaft on the original Copal motors, so you can take the small final gear off the early Copal motors and mount it on this 3D printed adapter.

<img alt="Copal - assembled" src="https://raw.githubusercontent.com/dpwe/StepperFlipClock/main/pics/copal_assembled.jpg" width=300 height=200> <img alt="Copal - back off" src="https://raw.githubusercontent.com/dpwe/StepperFlipClock/main/pics/copal_backoff.jpg" width=300 height=200> <img alt="Copal - mechanism" src="https://raw.githubusercontent.com/dpwe/StepperFlipClock/main/pics/copal_mechanism.jpg" width=300 height=200>

