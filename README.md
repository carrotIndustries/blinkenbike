The blinkenbike
===============

To look cooler on my local [Critical Mass](https://en.wikipedia.org/wiki/Critical_Mass_(cycling)) 
I decided to add some addressable LED strips to my bike as many other people did before me.

Rather than using an ESP32 board running [WLED](https://kno.wled.ge/), 
I took this as an excuse for another electronics project, so here we 
are.

![Photo of a bicycle with multi-color LED strips mounted to either side 
of the top tube. On the handlebar, there's the Cyclotron mini, showing 
Brn 010 on its LCD with orange backlight.](media/blinkenbike.jpg)



Having developed the [Cylotron 
mini](https://github.com/carrotIndustries/cyclotron-mini/) gave me the 
idea to also run the LEDs off of the hub dynamo and use its rotation 
for timing animations. I quickly scrapped the first part of that after 
learning that I won't get much use out of WS2812 LEDs on the 3 watts a hub dynamo provides.
Instead, everything's powered from a USB-PD powerbank.

The Cyclotron mini's connector to the mount on the handlebars includes 
a spare pin that had I no particular idea about what it's gonna be good 
for when designing it.

This spare pin now comes in really handy as it enables the Cyclotron 
mini to send the wheel position data to the LED controller. Apart from 
that, the Cyclotron mini is also the user interface for configuring 
parameters such as pattern or brightness.

# Hardware

The blinkenbike LED controller does these things:

 - Tell a USB-PD source, a powerbank in this case, to output 15V
 - Regulate the voltage from the powerbank to 5V for the LEDs
 - Drive the WS2812 LED strip based on the data received from the Cyclotron mini

![Photo of a green 8×5cm circuit board on a wooden surface. On the top 
edge, there are is a right-angle DSUB-9 and a USB Type C connector. 
The board is populated with various small ICs and a large inductor.](media/led-controller.jpg)

As usual, the board is made with [Horizon 
EDA](https://horizon-eda.org/). [Schematics](hw/output/schematic.pdf)

## USB-PD

USB-PD is pretty complicated, fortunately there are easy-to-use ICs 
such as the [CH224K](https://www.wch.cn/products/CH224.html) that just 
require a couple of pin straps to get the right voltage from a USB-PD 
source. Out of caution the board also has provision for an STUSB4500 in 
case the chip from china didn't work for for whatever reason. 

Fortunately, the CH224K worked right away and I've had no problems with 
it whatsoever.

It's also worth mentioning that to my best knowledge, no western 
silicon vendor makes a USB-PD IC that's as easy to use as the CH224K.

## Power supply

I calculated that the LED strip of 288 LEDs could draw up to 10A at 5V with 
all LEDs at full brightness, so running them directly off of the 5V 
output of a powerbank was out of the question.

An [SiC4347](https://www.vishay.com/docs/75921/sic437.pdf) buck 
converter steps down the voltage from the powerbank down to 
the 5V required by the LED strip. As with most other switching 
regulator ICs I've used so far, the circuit supporting it is pretty 
straight-forward.

What's not so straight-forward, is its footprint:

![Drawing of a QFN-like footprint with way too many dimensions lines](media/sic437.svg)

This motivated me to add some [enhancements](https://blog.horizon-eda.org/progress/2022/10/15/progress-2022-05-10.html#cursed-footprints)
to Horizon EDA to make creating such footprints easier.

## MCU

An STM32F042 takes care of receiving commands and wheel position data 
from the cyclotron mini, generating the selected pattern and sending it 
off to the WS2812 LED strip. The MCU is powered from a 3.3V linear 
regulator. It's connected to the USB port for firmware updates by means 
of its builtin ROM bootloader. For programming and debugging, there's a 
Tag-Connect footprint on the board.

An SN74LVC2T45 level translator converts the 3.3V logic levels from the 
MCU to the 5V levels required by the LEDs.

## Connector

The LED strips as well as the Cyclotron mini connect to the LED 
controller with a single DSUB-9 connector. For increased current 
handling, 3 pins each are wired in parallel for the 5V supply and its 
return path.

# Interface to the Cyclotron mini

The single line from the cyclotron mini to the LED controller is used 
as a unidirectional UART running at 1024 baud. The unusual baud rate 
stems from the UART being bit-banged in the cyclotron mini's main timer 
interrupt running at that frequency.

Commands are sent as three-byte ASCII-ish packets:

 1. Parameter, encoded as 'A' + parameter index
 2. upper nibble of the parameter value encoded as '0...9, a...f'
 3. lower nibble

That way, the start of a packet is unambiguously defined by an 
uppercase letter.

Every second byte sent is the wheel position in 5mm increments, encoded 
by values between 128 and 255, so there's no ambiguity with the other 
commands as they're all ASCII characters < 128. The wheel 
position thus wraps around every 127*5mm = 635mm which is taken care of 
on the receiving end by only considering the difference between two 
consecutive values.

So far, these parameters are implemented:

 - Brightness
 - Pattern
 - Speed override
 - Speed multiplier

# Firmware

## WS2812 protocol

The thing with WS2812 LEDs is that their protocol isn't supported 
by any of the MCU's builtin peripherals. Bit-banging it would consume 
too much CPU time, so one needs to make creative use of the available 
peripherals. After doing some reading on this topic, I went with the 
DMA-to-PWM approach. This involves converting the bits in the RGB 
framebuffer to duty cycle values that 
get transferred to a timer by means of DMA. That way, the only CPU 
involvement is converting the bits in the framebuffer to PWM values and 
triggering the DMA transfers.

## Power limiting

Initially, I had planned to use my powerbank's 20V-capable output as that would 
have provided enough power to run all LEDs at full brightness. 
Unfortunately, it turned out that this output shuts itself off after 
about 30 seconds of low load, making it not practical for my use case.

The 15V output doesn't have this problem, but is limited to 2A, so 
can't be used to run all LEDs at full brightness. To avoid overloading 
the powerbank, the firmware sums up the brightness values of all LEDs 
to get a current estimate and scales down the global brightness to stay 
within the power budget.

## Animations

The idea that sparked this project was to display animations 
synchronised to the bike's motion so that they appear to stand still 
from the point of a stationary observer. Even though the cyclotron mini 
uses the signal from the hub dynamo that provides 28 half cycles per 
rotation rather than just 1 as the usual reed contact, the resulting 
position resolution of approx. 80mm is still far too coarse for smooth 
animations. So we need to interpolate in some way. One way of looking 
at this is that we need to synchronise a local high-resolution 
oscillator to a reference that only provides occasional low-resolution 
phase updates. Nothing's better suited for this than a phase-locked 
loop. As I've [written before](https://github.com/carrotIndustries/redbook/blob/master/README.md#clock-recovery)
implementing a PLL can be done in just a couple of lines of code. As we 
don't care about the phase difference between the VCO and the 
reference, we can omit the integrator in the PLL's loop filter.

In this implementation VCO and phase detector are combined in a single 
accumulator: Each wheel position update increments the accumulator by 
the delta since the last update. On each frame, the local speed is 
calculated by dividing the accumulator value by 16. The local speed is 
then subtracted from the accumulator to close the loop. While this may 
sound a bit crude, it works surprisingly well.

The concept of an accumulator that's kept near zero also comes in handy 
for converting from the 5mm increments received from the cyclotron mini 
to the 1000mm/144 increments necessitated by the 144 LEDs/m LED strips. 
The aforementioned PLL actually operates in increments of 1/18mm as 
that's an integer fraction of both. To convert the 1/18mm increments 
back to 1000mm/144 increments, the local speed also gets added to a 
second accumulator. When this accumulator exceeds 125 (1000mm/144 * 
18), we know to advance the animation by one LED and subtract 125 from 
the accumulator. This implementation has the nice property that it's 
not subject to rounding errors, works on exact fractions and doesn't 
require floating point maths.

With all the time and effort spent on this, the actual patterns are 
rather boring chase animations and a PRBS. Double buffering prevents 
tearing.

# Mechanical Construction

The LED controller itself is housed in a 3D-printed two-piece shell 
that I designed in FreeCAD. The button on the top is connected to the 
BOOT0 pin of the STM32 to enter the DFU bootloader.

![Photo of a silver 3D printed box with a USB type C and DSUB-9 
connector on a wooden surface.](media/led-controller-case.jpg)

The LED strips are glued to flat aluminium strips that attach to the 
bike's frame with 3D-printed clips. They also guide the UART cable to 
the front of the bike. Power is fed from both ends to reduce voltage 
drop along the LED strips.

![Photo of two aluminium strips with LED strips attached. They're joind 
with a 3D-printed bracket on the left side. On the right side, there 
are cables going to a DSUB-9 connector](media/led-strip.jpg)
