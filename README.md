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

# Build & Flash Instructions

Clone the Repository

    git clone https://github.com/Skandakm29/VsdSquadron_mini_fpga-.git

    cd VsdSquadron_mini_fpga-
 
Compile and Flash the FPGA

    make build
    make flash

# Video Output


# Task 2:
UART (Universal Asynchronous Receiver-Transmitter) is a serial communication protocol used for asynchronous data transfer between devices. It does not require a separate clock signal; instead, it uses a baud rate to synchronize communication.

UART loopback is a special mode where the transmitted (TX) data is directly routed to the receiver (RX), allowing self-testing without external connections.

## Study the Existing Code:
clk - system clock signal (input)

txbyte - 8bit input to be transmitted

senddata = control signal to transmit data (input)

txdone = flag that indicates whether the data is transmitted or not 

    tx = TX wire 

## This module is a FINITE STATE MACHINE with four states:

    STATE_IDLE = 8’d0 - Module waits in the state until senddata is high

    STATE_STARTTX = 8’d0 - Sending Start bit (From LSB)

    STATE_TXING = 8’d0 - The 8 bit data is sent, one bit at a time

    STATE_TXDONE = 8’d0 - Sending stop bit and module returns to idle state

## Internal Registers and Wires:

    reg[7:0] state = 8'b0; // Holds the current state of the FSM

    reg[7:0] buf_tx = 8'b0;   // Buffer to store the byte being transmitted

    reg[7:0] bits_sent = 8'b0; // keeps track for the number of bits transmitted (8 bits)

    reg txbit = 1'b1;         // Holds the TX output signal

    reg txdone = 1'b0;        // Flag to indicate transmission is complete

    assign tx = txbit; // Assign txbit register to the output tx pin

# Logic:

    if (senddata == 1 && state == STATE_IDLE) begin
    state <= STATE_STARTTX; // Move to start bit state
    buf_tx <= txbyte;       // Store the byte to be transmitted
    txdone <= 1'b0;         // Clear the txdone flag
    end else if (state == STATE_IDLE) begin
    txbit <= 1'b1;  // TX is high when idle
    txdone <= 1'b0; // Reset completion flag
    end
When the senddata is High and the FSM state is in IDLEstate, the state is changed to STARTTX to move the start bit.

The 8 bit input to be sent is stored in a buffer register buf_tx.

Txdone is cleared to indicate transmission has started

    if (state == STATE_STARTTX) begin
    txbit <= 1'b0; // Send start bit (UART requires start bit = 0)
    state <= STATE_TXING; // Move to data transmission state
    end
The conditional statement checks whether the state is STARTTX. The first bit 0 is sent as the starting bit to initiate the transmission.

After this execution the state changes to TXING because the bits stated to transmit

    if (state == STATE_TXING && bits_sent < 8'd8) begin
    txbit <= buf_tx[0]; // Send the least significant bit (LSB) first
    buf_tx <= buf_tx >> 1; // Shift the buffer right to prepare the next bit
    bits_sent = bits_sent + 1; // Increment bit counter
     end else if (state == STATE_TXING) begin
    // Send stop bit (high)
    txbit <= 1'b1;
    bits_sent <= 8'b0;  // Reset bit counter
    state <= STATE_TXDONE;
    end
The conditional block checks whether the state is TXING  and bits sent are less than 8 bits.

After checking, the txbit starts to send from the least significant bit of the buffer register. Then, the pointer is shifted to the 

right, sending the other bits one by one. Once each bits shift, the increment counter bits_sent is incremented.

If 8 bits are already sent in this section reset the bit counter and set the txbit flag high changing the state to TXDONE

    if (state == STATE_TXDONE) begin
    txdone <= 1'b1;  // Indicate that transmission is complete
    state <= STATE_IDLE; // Return to idle state
    end
Once the transmission is done and the state is in TXDONE, the txdone flag is set to high indicating the transmission is complete and 

state is set to STATE_IDLE 

## Direct Connection Logic

    assign uarttx = uartrx;
Any data sent on uarttx is instantly received on uartrx.

This eliminates the need for external connections during testing.

Helps debug UART transmission in an FPGA-based system.

# BLOCK DIAGRAM FOR UART LOOPBACK:

https://github.com/yashwanth2109/VSD_FPGA_mini/blob/main/Task%202%20BD.png?raw=true

# CIRCUIT DIAGRAM FOR UART LOOPBACK



# Testing and Output

Clone & Setup Repository

    git clone https://github.com/Skandakm29/VsdSquadron_mini_fpga_uart_loopback.git
    cd "VsdSquadron_mini_fpga_uart_loopback"
Build & Flash FPGA Bitstream

Build the Bitstream

    make build
Generates top.bin for the FPGA.

Flash to FPGA

    sudo make flash
Uploads the bitstream to the FPGA.

UART Loopback Testing

Open Serial Termina

    sudo picocom -b 9600 /dev/ttyUSB0 --echo
If this shows an error in picocom:

    sudo apt install picocom
Remove and Reconnect the USB and make sure to select it again in the devices before running the terminal
Send Data & Verify Output
Expected Output:
Sent Data (TX)
Received Data (RX)
I - II
H - HH

# Video Output



# Task 3:
# UART Transmitter Module
## Overview
This module implements an 8N1 UART Transmitter, enabling serial data transmission using an 8-bit data frame, no parity bit, and 1 stop

bit. It generates a 9600 baud clock from a 12 MHz oscillator and provides a simple state-machine-based transmission mechanism.The code

for this module can accessed here.

# Study the Existing Code
Understanding the code

Internal Oscillator - Generates the main clock signal (clk_out).

This drives all other modules like the baud rate generator, counter, UART, etc.

Frequency Counter - Uses clk_out to count the frequency and displays it (probably through UART or an LED interface).

Helps verify your oscillator output or system clock stability.

Baud Rate Generator - Converts the main clock into a baud_clk (e.g., 9600 Hz for UART communication).

Synchronizes UART TX and RX timings.

UART Receiver (RX) - Captures serial data from PC (or other UART source).

Converts it to parallel data.
 
Sends received data to - Loopback module and LED control logic

UART Loopback - Sends back the received data directly to UART TX (echo test).

It is Useful for testing whether UART RX and TX are functioning properly.


RGB LED Controller - Takes parallel data from UART RX.

Decodes certain data values (e.g., ASCII characters or specific codes) to light up different RGB LED colors.


UART Transmitter (TX) - Transmits the loopback data 

# Program Flow:
Data flow:

Clock Signal: The Internal Oscillator drives entire system.


UART Comm: UART RX receives → Sends to loopback → UART TX transmits back.

UART RX also sends data → RGB LED controller → LED color changes.

Frequency Counter: Monitors clock → Could send data via UART TX.

## Your UART terminal received:
kotlin

CopyEdit

    Recieved data= 52
    Recieved data= 47

These are ASCII codes:

     52 = '4'
     47 = '/'

So you typed '4/' in the terminal, and it correctly looped back and displayed the decimal ASCII values—meaning your UART RX, loopback,

and TX paths are working.

## State Machine States:

    IDLE STATE (STATE_IDLE)
If senddata = 1 and the state is STATE_IDLE, it:

Moves to the STATE_STARTTX state.

Loads txbyte (8-bit data to transmit) into buf_tx.

Clears txdone (indicates transmission is ongoing).

Otherwise, if still in STATE_IDLE, it:

Keeps txbit high (1) because UART idles at high.

Ensures txdone remains low (0).

Start Bit Transmission (STATE_STARTTX)

Once in STATE_STARTTX, it:

Sets txbit low (0) (start bit in UART communication).

Moves to STATE_TXING to transmit data bits.

Sending Data Bits (STATE_TXING)

If state == STATE_TXING and bits_sent < 8, it sends the Least Significant Bit (LSB) of buf_tx.

Shifts buf_tx right (>> 1).

Increments bits_sent.

Stp Bit Transmission (STATE_TXDONE)

After 8 data bits are transmitted, it:

Sends the stop bit (1).

Resets bits_sent to 0.

Moves to STATE_TXDONE.

Transmission Complete (STATE_TXDONE → STATE_IDLE)

In STATE_TXDONE, it:

Sets txdone = 1 (indicates transmission complete).

Returns to STATE_IDLE.

# System Architecture

# Block diagram

# Circuit diagram

# Programming and Synthesis

Clone & Setup Repository

    git clone https://github.com/Skandakm29/VsdSquadron_mini_fpga_uart_loopback.git
    cd "VsdSquadron_mini_fpga_uart_loopback"
Build & Flash FPGA Bitstream

Build the Bitstream

    make build
Generates top.bin for the FPGA.

Flash to FPGA

    sudo make flash
Uploads the bitstream to the FPGA.

UART Loopback Testing

Open Serial Termina

    sudo picocom -b 9600 /dev/ttyUSB0 --echo
If this shows an error in picocom:

    sudo apt install picocom
Remove and Reconnect the USB and make sure to select it again in the devices before running the terminal

# Video Output





# Task 4:
## Architecture Summary
The sense_uart_tx module facilitates structured and sensor-based UART data transmission. The core components of this architecture are:

Sensor Data Processing

Baud Clock Creation

UART Data Transfer Mechanism

State Machine for Transmission Control

## Functional Workflow
Sensor Data Acquisition - Sensor readings are captured at specific time intervals. The data_valid signal flags when new data is ready to

be sent. A 32-bit buffer temporarily stores sensor values before transmission.

Baud Rate Management - A baud rate generator ensures a consistent 9600 baud frequency. A counter mechanism regulates accurate timing for

each bit.

UART Transmission Steps:

START BIT: Transmission begins with a low (0) start bit.


DATA BITS: Sends 8-bit segments of the 32-bit sensor data sequentially.


STOP BIT: Ends with a high (1) stop bit to complete the data frame.


Controlled state transitions ensure accurate communication timing.



## Transmission Status Indicators

tx_done indicates the end of a transmission cycle.

ready ensures the module can handle back-to-back sensor data without loss.

## Port Descriptions

Clock and Reset

clk: Governs all synchronous logic.

reset_n: Resets internal states and modules.

## Sensor Inputs

sensor_data [31:0]: 32-bit input from the sensor block.

data_valid: Signals the availability of new sensor data.

## UART Output Signal

tx_out: Carries the UART-encoded serial data to external devices.


## Control Signals

tx_start: Begins the UART transmission process.


tx_done: Indicates that data transmission has completed.


ready: Confirms the system is prepared for the next transmission.

## Internal Design Components

Finite State Machine (FSM):

IDLE: Awaits the data_valid signal.

START: Sends the initial start bit (0).

DATA: Outputs 8-bit segments of the sensor data one after the other.

STOP: Transmits the final stop bit (1).

DONE: Asserts tx_done and resets to the IDLE state.

Baud Generator - Utilizes clock division to generate a precise 9600 Hz baud signal.

Shift Register - Temporarily stores the full 32-bit sensor data. Shifts out 8-bit chunks with each transmission cycle.

# Block Diagram 

# Circuit Diagram

# Programming and Synthesis

Clone & Setup Repository

    git clone https://github.com/Skandakm29/VsdSquadron_mini_fpga_uart_loopback.git
    cd "VsdSquadron_mini_fpga_uart_loopback"
Build & Flash FPGA Bitstream

Build the Bitstream

    make build
Generates top.bin for the FPGA.

Flash to FPGA

    sudo make flash
Uploads the bitstream to the FPGA.

UART Loopback Testing

Open Serial Termina

    sudo picocom -b 9600 /dev/ttyUSB0 --echo
If this shows an error in picocom:

    sudo apt install picocom
Remove and Reconnect the USB and make sure to select it again in the devices before running the terminal

# Video Output

























