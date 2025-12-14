This is a Xilinx Design Constraints file that maps Verilog signals to physical FPGA pins.
    # Example ~ You DON'T need this;
    set_property PACKAGE_PIN E3 [get_ports clk]
    set_property PACKAGE_PIN C12 [get_ports rst_n]
    
    ^^^ This tells Vivado to connect the clk signal to physical pin E3 on the chip

You might wonder when you'd need this, and i'll tell you why.
1; Deploying to Xilinx Artix-7, Zynq, etc.
2; Need to map buttons, LEDs, UART pins
3; Physical hardware synthesis

Currently we're using Verilator, which compiles Verilog to C++ for software simulation. No physical pins needed.
So currently this script isn't needed currently but is added here for you to mess around with.

--- Delete all text above to implement this script if needed ---

create_clock -period 10.000 -name clk [get_ports clk]
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
set_property -dict {PACKAGE_PIN C12 IOSTANDARD LVCMOS33} [get_ports rst_n]

set_input_delay -clock clk 2.000 [get_ports rst_n]

set_max_delay 8.000 -from [get_pins cpu/dp/registers_reg*/C] -to [get_pins cpu/dp/alu_inst/*]
set_max_delay 8.000 -from [get_pins cpu/cu/state_reg*/C] -to [get_pins cpu/dp/*]

set_false_path -from [get_ports rst_n]

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_IBUF]