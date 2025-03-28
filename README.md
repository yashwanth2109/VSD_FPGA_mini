# TASK 1:
# PURPOSE OF THE MODULE:

The purpose of the module named “top” is used to implement a FPGA RGB LED controller which uses an internal oscillator(clock) and a counter(to create a slower clock signal for testing) to set the LED to emit blue light while keeping the red and green off.

# DESCRIPTION OF THE INTERNAL LOGIC AND THE OSCILLATOR:

## Internal Logic:

led_red, blue, green - controls the LED’s RED, BLUE, GREEN respectively

testwire - acts as a test signal which is set to the 6th bit of frequency_counter_i to create a slower signal for testing

int_osc  - internal oscillator signal

frequency_counter_i[27:0] - 28 bit register which acts as a counter

## LOGIC: During the rising edge of int_osc the register bit is incremented

## Oscillator:

SB_HFOSC - hardware primitive for high frequency Oscillators

SB_RGBA_DRV - hardware primitive for driving RGB LEDs

CLKHF_DIV(“0b10”) - sets the oscillator frequency to 12MHz

CLKHFPU(1’b1) - Powers up the oscillator 

CLKHFEN(1’b1) - Enables the oscillator

CLKHF(int_osc) - Clock output

# FUNCTIONALITY OF THE RGB LED DRIVERS AND ITS RELATIONSHIP WITH THE OUTPUTS:

RGBLEDEN(1’b1) - Enables the RGB LED Driven

CURREN(1’b1) - Enables the current.

RGB0PWM(1’b0) - Red LED off  (RGB0 - Red)

RGB1PWM(1’b0) - GreenLED off  (RGB1 - Green)

RGB2PWM(1’b1) - Blue LED on  (RGB0 - Blue)

1’b1 = LED On:
1’b0 = LED Off

defparam RGB_DRIVER.RGB0_CURRENT = “0b000001”;

defparam RGB_DRIVER.RGB1_CURRENT = “0b000001”;

defparam RGB_DRIVER.RGB2_CURRENT = “0b000001”;

Defparam sets the current level of each LED.  “0b000001” sets low current to reduce the power consumption

# PCF FILE AND PIN ASSIGNMENTS:

set_io  led_red 39      => RED LED  pin 39 in pin assignments        = (RGB0)

set_io  led_blue 40     => BLUE LED  pin 40 in pin assignments       = (RGB1)

set_io  led_green 41    => GREEN LED pin 41 in pin assignments       = (RGB2)

set_io  hw_clk 20       => HARDWARE CLOCK  pin 20 in pin assignments = (IOB_25b_G3)

set_io  testwire 17     => TESTWIRE pin 17 in pin assignments        = (IOB_33b_SPI_SI)






