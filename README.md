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

https://github.com/user-attachments/assets/9cbd1d75-d844-45b5-a4e9-73b0fb15277a




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

![Image](https://github.com/user-attachments/assets/7a4f7f77-5763-4e6a-a678-f19fc70d71c3)

# CIRCUIT DIAGRAM FOR UART LOOPBACK
 ![Image](https://github.com/user-attachments/assets/e4b5eed7-4945-4277-99c0-d7d4b17fd960)

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

# Circuit diagram

![Image](https://github.com/user-attachments/assets/93a868e5-8212-4010-9d19-6c88a1c4461f)


# Block diagram

![Image](https://github.com/user-attachments/assets/2f3548ea-210d-47e4-89cb-de6b6b667abd)

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

https://github.com/user-attachments/assets/d3a37f0b-0511-4e0e-be31-a68e053cb7b2





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

![Image](https://github.com/user-attachments/assets/ac76cd3a-a78a-4627-bbe9-fc0437238db9)

# Circuit Diagram

![Image](https://github.com/user-attachments/assets/28d3f472-c836-48c9-820f-11b9e07931da)

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
https://github.com/user-attachments/assets/dba22279-39f0-445d-a74d-769c01abd376





# PROJECT - Real time Sensor Data Acquisition and Transmission System:
## Introduction
This report presents a complete design for a real-time sensor-based measurement and communication system using an FPGA. The system measures distance using the HC-SR04 ultrasonic sensor, 

processes the signal on the FPGA board, and transmits the result to a computer through UART. Included are system details, schematics, annotated Verilog modules, testing procedures, and

a video demo.

## System Details
The design consists of several interconnected components:

1. 12 MHz internal oscillator on the FPGA serves as the main clock source.

2. A timing module generates periodic measurement triggers, e.g., every 50 ms or 250 ms.

3.  ultrasonic sensor module (hc_sr04.v) emits a 10 µs trigger pulse and calculates distance based on echo return time.

4. UART transmission is handled by a module (uart_tx_8n1.v) that sends the calculated distance in ASCII format at a baud rate of 9600.

RGB LEDs can optionally display status visually. The top-level Verilog file connects all submodules. It holds the distance result, converts it to ASCII, and sends it to a PC via UART

using a USB-to-Serial adapter.

 # Block Diagram

 ![Image](https://github.com/user-attachments/assets/3c138c13-4ce2-451f-b51d-4eeb65809072)

 # Circuit Diagram

 ![Image](https://github.com/user-attachments/assets/fd626953-c339-4fa2-a0ae-04fafcc82537)

# Verilog Modules
## UART Transmitter – uart_tx_8n1.v

    // 8N1 UART Module, transmit only

    module uart_tx_8n1 (
    clk,        // input clock
    txbyte,     // outgoing byte
    senddata,   // trigger tx
    txdone,     // outgoing byte sent
    tx,         // tx wire
    );

    /* Inputs */
    input clk;
    input[7:0] txbyte;
    input senddata;

    /* Outputs */
    output txdone;
    output tx;

    /* Parameters */
    parameter STATE_IDLE=8'd0;
    parameter STATE_STARTTX=8'd1;
    parameter STATE_TXING=8'd2;
    parameter STATE_TXDONE=8'd3;

    /* State variables */
    reg[7:0] state=8'b0;
    reg[7:0] buf_tx=8'b0;
    reg[7:0] bits_sent=8'b0;
    reg txbit=1'b1;
    reg txdone=1'b0;

    /* Wiring */
    assign tx=txbit;

    /* always */
    always @ (posedge clk) begin
        // start sending?
        if (senddata == 1 && state == STATE_IDLE) begin
            state <= STATE_STARTTX;
            buf_tx <= txbyte;
            txdone <= 1'b0;
        end else if (state == STATE_IDLE) begin
            // idle at high
            txbit <= 1'b1;
            txdone <= 1'b0;
        end

        // send start bit (low)
        if (state == STATE_STARTTX) begin
            txbit <= 1'b0;
            state <= STATE_TXING;
        end
        // clock data out
        if (state == STATE_TXING && bits_sent < 8'd8) begin
            txbit <= buf_tx[0];
            buf_tx <= buf_tx>>1;
            bits_sent = bits_sent + 1;
        end else if (state == STATE_TXING) begin
            // send stop bit (high)
            txbit <= 1'b1;
            bits_sent <= 8'b0;
            state <= STATE_TXDONE;
        end

        // tx done
        if (state == STATE_TXDONE) begin
            txdone <= 1'b1;
            state <= STATE_IDLE;
        end

    end

    endmodule

## Core Features:
Implements a finite state machine (FSM) with the following states:

IDLE – Waits for transmission request.

START – Sends start bit (logic 0).

DATA – Transmits 8 bits, starting from LSB.

STOP – Sends stop bit (logic 1), then returns to IDLE.

Operates with a 9600 Hz enable signal to time each UART bit transmission step.

## Ultrasonic Sensor Module – ultra_sonic_sensor.v

    module hc_sr04 #(
    parameter ten_us = 10'd120  // ~120 cycles for ~10µs at 12MHz
    )(
    input             clk,         // ~12 MHz clock
    input             measure,     // start a measurement when in IDLE
      output reg [1:0]  state,       // optional debug: current state
      output            ready,       // high in IDLE (between measurements)
      input             echo,        // ECHO pin from HC-SR04
      output            trig,        // TRIG pin to HC-SR04
      output reg [23:0] distanceRAW, // raw cycle count while echo=1
      output reg [15:0] distance_cm  // computed distance in cm
    );

      // -----------------------------------------
      // State definitions
      // -----------------------------------------
      localparam IDLE      = 2'b00,
                 TRIGGER   = 2'b01,
                 WAIT      = 2'b11,
                 COUNTECHO = 2'b10;
    
      // 'ready' is high in IDLE
      assign ready = (state == IDLE);
    
      // 10-bit counter for ~10µs TRIGGER
      reg [9:0] counter;
      wire trigcountDONE = (counter == ten_us);

      // Initialize registers (for simulation & synthesis without reset)
      initial begin
        state       = IDLE;
        distanceRAW = 24'd0;
        distance_cm = 16'd0;
        counter     = 10'd0;
      end
    
      // -----------------------------------------
      // 1) State Machine
      // -----------------------------------------
      always @(posedge clk) begin
        case (state)
          IDLE: begin
            // Wait for measure pulse
            if (measure && ready)
              state <= TRIGGER;
          end

      TRIGGER: begin
        // ~10µs pulse, then WAIT
        if (trigcountDONE)
          state <= WAIT;
      end

      WAIT: begin
        // Wait for echo rising edge
        if (echo)
          state <= COUNTECHO;
      end

      COUNTECHO: begin
        // Once echo goes low => measurement done
        if (!echo)
          state <= IDLE;
      end

      default: state <= IDLE;
    endcase
      end
    
      // -----------------------------------------
      // 2) TRIG output is high in TRIGGER
      // -----------------------------------------
      assign trig = (state == TRIGGER);
    
      // -----------------------------------------
      // 3) Generate ~10µs trigger pulse
      // -----------------------------------------
      always @(posedge clk) begin
        if (state == IDLE) begin
          counter <= 10'd0;
        end
        else if (state == TRIGGER) begin
          counter <= counter + 1'b1;
        end 
        // No else needed; once we exit TRIGGER, we stop incrementing.
        end

      // -----------------------------------------
      // 4) distanceRAW increments while ECHO=1
      // -----------------------------------------
      always @(posedge clk) begin
        if (state == WAIT) begin
          // Reset before new measurement
          distanceRAW <= 24'd0;
        end
        else if (state == COUNTECHO) begin
          // Add 1 each clock cycle while echo=1
          distanceRAW <= distanceRAW + 1'b1;
        end
      end
    
      // -----------------------------------------
      // 5) Convert distanceRAW to centimeters
      // -----------------------------------------
      // distance_cm = (distanceRAW * 34300) / (2 * 12000000)
      always @(posedge clk) begin
        distance_cm <= (distanceRAW * 34300) / (2 * 12000000);
      end

    endmodule

    //===================================================================
    // 2) Refresher for ~50ms or ~250ms pulses
    //===================================================================
    module refresher250ms(
      input  clk,  // 12MHz
      input  en,
      output measure
    );
      // For ~50ms at 12MHz: 12,000,000 * 0.05 = 600,000
      // For ~250ms at 12MHz: 12,000,000 * 0.25 = 3,000,000
      reg [18:0] counter;
    
      // measure = 1 if counter == 1 => single‐cycle pulse
      assign measure = (counter == 22'd1);
    
      initial begin
        counter = 22'd0;
      end
    
      always @(posedge clk) begin
        if (~en || (counter == 22'd600000))  
      // change to 3_000_000 if you want 250ms
      counter <= 22'd0;
    else
      counter <= counter + 1;
      end
    endmodule
    
## FSM Operation:

IDLE → TRIGGER → WAIT → COUNTECHO → IDLE

A 10 µs trigger pulse is generated in the TRIGGER state.

Echo signal duration is counted during COUNTECHO and stored as distanceRAW.

Final distance is calculated in centimeters using:
 
    distance_cm = (distanceRAW * 34300) / (2 * 12000000);

## Measurement Interval Controller: 

Functionality:

Counts up to a predefined value (e.g., 3,000,000 cycles for 250 ms at 12 MHz).

Emits a one-clock-cycle measure pulse each time the count resets.

## Top-Level Module – top.v

Responsibilities:
Divides the 12 MHz clock to generate a 9600 Hz tick for UART transmission.

Connects the sensor, refresher, and UART modules.

Holds and converts distance to ASCII using a small FSM, then transmits via UART.

# Verification

## Testbench (ultra_sonic_sensor_tb.v):
    `include "uart_trx.v"
    `include "ultra_sonic_sensor.v"
    
    //----------------------------------------------------------------------------
    //                         Module Declaration
    //----------------------------------------------------------------------------
    module top (
      // outputs
      output wire led_red,    // Red
      output wire led_blue,   // Blue
      output wire led_green,  // Green
      output wire uarttx,     // UART Transmission pin
      input  wire uartrx,     // UART Reception pin
      input  wire hw_clk,
      input  wire echo,       // External echo signal from sensor
      output wire trig        // Trigger output for sensor
    );
    
      //----------------------------------------------------------------------------
      // 1) Internal Oscillator ~12 MHz
      //----------------------------------------------------------------------------
      wire int_osc;
      SB_HFOSC #(.CLKHF_DIV("0b10")) u_SB_HFOSC (
        .CLKHFPU(1'b1),
        .CLKHFEN(1'b1),
        .CLKHF(int_osc)
      );
    
      //----------------------------------------------------------------------------
      // 2) Generate 9600 baud clock (from ~12 MHz)
      //----------------------------------------------------------------------------
      reg  clk_9600 = 0;
      reg  [31:0] cntr_9600 = 32'b0;
      parameter period_9600 = 625; // half‐period for 12 MHz -> 9600 baud
    
      always @(posedge int_osc) begin
        cntr_9600 <= cntr_9600 + 1'b1;
        if (cntr_9600 == period_9600) begin
          clk_9600  <= ~clk_9600;
          cntr_9600 <= 32'b0;
        end
      end
    
      //----------------------------------------------------------------------------
      // 3) RGB LED driver (just tying them to uartrx for demonstration)
      //----------------------------------------------------------------------------
      SB_RGBA_DRV #(
        .RGB0_CURRENT("0b000001"),
        .RGB1_CURRENT("0b000001"),
        .RGB2_CURRENT("0b000001")
      ) RGB_DRIVER (
        .RGBLEDEN(1'b1),
        .RGB0PWM(uartrx),
        .RGB1PWM(uartrx),
        .RGB2PWM(uartrx),
        .CURREN(1'b1),
        .RGB0(led_green),
        .RGB1(led_blue),
        .RGB2(led_red)
      );
    
      //----------------------------------------------------------------------------
      // 4) Ultrasonic Sensor signals
      //    We'll assume ultra_sonic_sensor.v (hc_sr04) has output distance_cm [15:0]
      //----------------------------------------------------------------------------
      wire [23:0] distanceRAW;       // If the sensor module also provides raw
      wire [15:0] distance_cm;       // MUST exist in hc_sr04, or define it
      wire        sensor_ready;
      wire        measure;
    
      hc_sr04 u_sensor (
        .clk        (int_osc),
        .trig       (trig),
        .echo       (echo),
        .ready      (sensor_ready),
        .distanceRAW(distanceRAW),
        .distance_cm(distance_cm),  // must exist in your sensor module
        .measure    (measure)
      );
    
      //----------------------------------------------------------------------------
      // 5) Trigger the sensor every ~250 ms or 50 ms
      //----------------------------------------------------------------------------
      refresher250ms trigger_timer (
        .clk (int_osc),
        .en  (1'b1),  // always enabled
        .measure (measure)
      );
    
      //----------------------------------------------------------------------------
      // 6) Finite‐State Machine to Print distance_cm as ASCII
      //----------------------------------------------------------------------------
      reg [3:0] state;
      localparam IDLE    = 4'd0,
                 DIGIT_4 = 4'd1,
                 DIGIT_3 = 4'd2,
                 DIGIT_2 = 4'd3,
                 DIGIT_1 = 4'd4,
                 DIGIT_0 = 4'd5,
                 SEND_CR = 4'd6,
                 SEND_LF = 4'd7,
                 DONE    = 4'd8;
    
      reg [31:0] distance_reg; // latch distance_cm for division
      reg [7:0]  tx_data;
      reg        send_data;
    
      // We run this state machine at clk_9600 so we only load
      // one character per 1-bit time. (Simplistic approach.)
      always @(posedge clk_9600) begin
        // By default, don't load a new character
        send_data <= 1'b0;

    case (state)
      //-------------------------------------------------
      // IDLE: wait for sensor_ready
      //-------------------------------------------------
      IDLE: begin
        if (sensor_ready) begin
          distance_reg <= distance_cm; // store the 16-bit measurement
          state <= DIGIT_4;           // go print all digits
        end
      end

      //-------------------------------------------------
      // Print the top decimal digit (5 digits total => "00057")
      //-------------------------------------------------
      DIGIT_4: begin
        tx_data  <= ((distance_reg / 10000) % 10) + 8'h30;
        send_data <= 1'b1;
        state    <= DIGIT_3;
      end
      DIGIT_3: begin
        tx_data  <= ((distance_reg / 1000) % 10) + 8'h30;
        send_data <= 1'b1;
        state    <= DIGIT_2;
      end
      DIGIT_2: begin
        tx_data  <= ((distance_reg / 100) % 10) + 8'h30;
        send_data <= 1'b1;
        state    <= DIGIT_1;
      end
      DIGIT_1: begin
        tx_data  <= ((distance_reg / 10) % 10) + 8'h30;
        send_data <= 1'b1;
        state    <= DIGIT_0;
      end
      DIGIT_0: begin
        tx_data  <= (distance_reg % 10) + 8'h30;
        send_data <= 1'b1;
        state    <= SEND_CR;
      end

      //-------------------------------------------------
      // Carriage Return + Line Feed
      //-------------------------------------------------
      SEND_CR: begin
        tx_data   <= 8'h0D; // '\r'
        send_data <= 1'b1;
        state     <= SEND_LF;
      end
      SEND_LF: begin
        tx_data   <= 8'h0A; // '\n'
        send_data <= 1'b1;
        state     <= DONE;
      end

      //-------------------------------------------------
      // Go back to IDLE
      //-------------------------------------------------
      DONE: begin
        state <= IDLE;
      end

      default: state <= IDLE;
    endcase
      end
    
      //----------------------------------------------------------------------------
      // 7) UART Transmitter
      //----------------------------------------------------------------------------
      uart_tx_8n1 sensor_uart (
        .clk      (clk_9600),
        .txbyte   (tx_data),
        .senddata (send_data),
        .tx       (uarttx)
      );
    
    endmodule
    
Simulates measure and dummy echo signals.

Outputs waveforms to wave.vcd.

Validates that the COUNTECHO timing matches the computed distance.


# On-Board Testing
## Connections:
TRIG pin → Sensor TRIG, ECHO pin ← Sensor ECHO

3 V power to sensor, shared GND with FPGA

UART TX from FPGA to PC via USB–Serial adapter

## PC Terminal Setup:

Baud rate: 9600


Data bits: 8


Parity: None


Stop bits: 1


## Testing Method:
Position an object approximately 10 cm from the sensor.

The terminal should display a reading near "00010".

Moving the object should change the displayed distance dynamically.


# Video Output

# Compilation and Running

    make build

This runs the required synthesis tools (Yosys, nextpnr, icepack) to generate the bitstream file (e.g., top.bin).

    sudo make flash

Uploads the bitstream to the FPGA board using iceprog or similar.

    sudo make terminal

Launches a terminal window (e.g., using screen or minicom) at 9600 baud to monitor the distance readings. (Serial Terminal)

# Video Output

https://github.com/user-attachments/assets/5a92950a-49de-4bc7-b689-9e6aeaa5128a

# Conclusion
This project successfully implements a real-time sensor interface using FPGA:\

Periodic distance measurement with HC-SR04


Real-time echo counting and distance calculation


ASCII-based serial communication at 9600 baud


Optional local display using RGB LEDs


The design provides a foundation that can be expanded with additional sensor types, error correction, or enhanced UART configurations.

























