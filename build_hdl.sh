#!/bin/bash

set -e

echo "Building HDL"

if command -v iverilog &> /dev/null; then
    echo "Running simulation with Icarus Verilog"
    iverilog -g2012 -o cpu57_sim hdl/cpu57.v hdl/cpu57_tb.v
    vvp cpu57_sim
    
    if command -v gtkwave &> /dev/null; then
        echo "Opening waveform viewer"
        gtkwave cpu57_tb.vcd &
    fi
else
    echo "Icarus Verilog not found. Install with: sudo apt install iverilog"
fi

if command -v yosys &> /dev/null && [ "$SKIP_SYNTHESIS" != "1" ]; then
    echo ""
    echo "Yosys synthesis available but skipped by default (design has 1MB+ memory)"
    echo "To attempt synthesis anyway, run: SKIP_SYNTHESIS=0 bash build_hdl.sh"
    echo "Warning: This will consume significant RAM and may hang your system"
else
    if [ "$SKIP_SYNTHESIS" = "0" ]; then
        echo "Attempting Yosys synthesis..."
        timeout 30s yosys -p "read_verilog -sv hdl/cpu57.v; hierarchy -top cpu57; write_json cpu57.json" 2>&1 | head -n 100
        if [ $? -eq 124 ]; then
            echo "Synthesis timed out (expected for large designs)"
        fi
    fi
fi

if command -v verilator &> /dev/null; then
    echo "Linting with Verilator"
    verilator --lint-only -Wall \
        -Wno-DECLFILENAME \
        -Wno-UNUSEDSIGNAL \
        -Wno-VARHIDDEN \
        -Wno-EOFNEWLINE \
        hdl/cpu57.v
    if [ $? -eq 0 ]; then
        echo "Lint complete - no errors"
    else
        echo "Lint found issues (see above)"
    fi
else
    echo "Verilator not found. Install with: sudo apt install verilator"
fi

echo "Build complete"