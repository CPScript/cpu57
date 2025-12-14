module cpu57_tb;

    reg clk;
    reg rst_n;
    
    cpu57_system dut (
        .clk(clk),
        .rst_n(rst_n)
    );
    
    always #5 clk = ~clk;
    
    integer cycle_limit = 200;
    integer current_cycle = 0;
    
    initial begin
        $dumpfile("cpu57_tb.vcd");
        $dumpvars(0, cpu57_tb);
        
        clk = 0;
        rst_n = 0;
        
        #20;
        rst_n = 1;
        #10;
        
        $display("Loading test program...");
        
        dut.mem.mem[0] = 8'h12;
        dut.mem.mem[1] = 8'h00;
        dut.mem.mem[2] = 8'h00;
        dut.mem.mem[3] = 8'h00;
        dut.mem.mem[4] = 8'h64;
        dut.mem.mem[5] = 8'h00;
        dut.mem.mem[6] = 8'h00;
        dut.mem.mem[7] = 8'h00;
        dut.mem.mem[8] = 8'h00;
        dut.mem.mem[9] = 8'h00;
        
        dut.mem.mem[10] = 8'h12;
        dut.mem.mem[11] = 8'h01;
        dut.mem.mem[12] = 8'h00;
        dut.mem.mem[13] = 8'h00;
        dut.mem.mem[14] = 8'h05;
        dut.mem.mem[15] = 8'h00;
        dut.mem.mem[16] = 8'h00;
        dut.mem.mem[17] = 8'h00;
        dut.mem.mem[18] = 8'h00;
        dut.mem.mem[19] = 8'h00;
        
        dut.mem.mem[20] = 8'h02;
        dut.mem.mem[21] = 8'h00;
        dut.mem.mem[22] = 8'h00;
        dut.mem.mem[23] = 8'h01;
        dut.mem.mem[24] = 8'h00;
        dut.mem.mem[25] = 8'h00;
        dut.mem.mem[26] = 8'h00;
        dut.mem.mem[27] = 8'h00;
        dut.mem.mem[28] = 8'h00;
        dut.mem.mem[29] = 8'h00;
        
        dut.mem.mem[30] = 8'h12;
        dut.mem.mem[31] = 8'h03;
        dut.mem.mem[32] = 8'h00;
        dut.mem.mem[33] = 8'h00;
        dut.mem.mem[34] = 8'h02;
        dut.mem.mem[35] = 8'h00;
        dut.mem.mem[36] = 8'h00;
        dut.mem.mem[37] = 8'h00;
        dut.mem.mem[38] = 8'h00;
        dut.mem.mem[39] = 8'h00;
        
        dut.mem.mem[40] = 8'h03;
        dut.mem.mem[41] = 8'h02;
        dut.mem.mem[42] = 8'h00;
        dut.mem.mem[43] = 8'h03;
        dut.mem.mem[44] = 8'h00;
        dut.mem.mem[45] = 8'h00;
        dut.mem.mem[46] = 8'h00;
        dut.mem.mem[47] = 8'h00;
        dut.mem.mem[48] = 8'h00;
        dut.mem.mem[49] = 8'h00;
        
        dut.mem.mem[50] = 8'h12;
        dut.mem.mem[51] = 8'h04;
        dut.mem.mem[52] = 8'h00;
        dut.mem.mem[53] = 8'h00;
        dut.mem.mem[54] = 8'hE8;
        dut.mem.mem[55] = 8'h03;
        dut.mem.mem[56] = 8'h00;
        dut.mem.mem[57] = 8'h00;
        dut.mem.mem[58] = 8'h00;
        dut.mem.mem[59] = 8'h00;
        
        dut.mem.mem[60] = 8'h41;
        dut.mem.mem[61] = 8'h02;
        dut.mem.mem[62] = 8'h00;
        dut.mem.mem[63] = 8'h00;
        dut.mem.mem[64] = 8'h05;
        dut.mem.mem[65] = 8'h00;
        dut.mem.mem[66] = 8'h00;
        dut.mem.mem[67] = 8'h00;
        dut.mem.mem[68] = 8'h00;
        dut.mem.mem[69] = 8'h00;
        
        dut.mem.mem[70] = 8'hFF;
        dut.mem.mem[71] = 8'h00;
        dut.mem.mem[72] = 8'h00;
        dut.mem.mem[73] = 8'h00;
        dut.mem.mem[74] = 8'h00;
        dut.mem.mem[75] = 8'h00;
        dut.mem.mem[76] = 8'h00;
        dut.mem.mem[77] = 8'h00;
        dut.mem.mem[78] = 8'h00;
        dut.mem.mem[79] = 8'h00;
        
        $display("Starting simulation...");
        $display("Expected: R0=95, R1=5, R2=190, R3=2, R4=1000");
        $display("");
    end
    
    always @(posedge clk) begin
        if (rst_n) begin
            current_cycle = current_cycle + 1;
            
            if (current_cycle <= 30) begin
                $display("[%0d] PC=%h State=%0d ByteIdx=%0d Opcode=%h ExecEn=%b Addr=%h MemData=%h", 
                         current_cycle, dut.cpu.dp.pc, dut.cpu.cu.state, 
                         dut.cpu.cu.byte_idx, dut.cpu.cu.opcode,
                         dut.cpu.cu.execute_enable,
                         dut.cpu.dp.address_bus, dut.mem.data_out);
            end
            else if (current_cycle % 20 == 0) begin
                $display("[Cycle %0d] PC=%h State=%0d Halted=%b", 
                         current_cycle, dut.cpu.dp.pc, dut.cpu.cu.state, dut.cpu.cu.halted);
            end
            
            if (dut.cpu.cu.halted) begin
                #50;
                $display("");
                $display("==========================================");
                $display("CPU Halted at cycle %0d", current_cycle);
                $display("==========================================");
                print_results();
                $finish;
            end
            
            if (current_cycle >= cycle_limit) begin
                $display("");
                $display("==========================================");
                $display("TIMEOUT after %0d cycles", cycle_limit);
                $display("==========================================");
                print_results();
                $finish;
            end
        end
    end
    
    task print_results;
        begin
            $display("");
            $display("Register values:");
            $display("  R0 = %h (%0d)", dut.cpu.dp.registers[0], dut.cpu.dp.registers[0]);
            $display("  R1 = %h (%0d)", dut.cpu.dp.registers[1], dut.cpu.dp.registers[1]);
            $display("  R2 = %h (%0d)", dut.cpu.dp.registers[2], dut.cpu.dp.registers[2]);
            $display("  R3 = %h (%0d)", dut.cpu.dp.registers[3], dut.cpu.dp.registers[3]);
            $display("  R4 = %h (%0d)", dut.cpu.dp.registers[4], dut.cpu.dp.registers[4]);
            $display("");
            $display("CPU State:");
            $display("  PC = %h", dut.cpu.dp.pc);
            $display("  SP = %h", dut.cpu.dp.sp);
            $display("  Cycles = %0d", dut.cpu.cu.cycle_count);
            $display("  State = %0d", dut.cpu.cu.state);
            $display("  Halted = %b", dut.cpu.cu.halted);
            $display("");
            $display("Flags:");
            $display("  Zero = %b", dut.cpu.dp.flag_zero);
            $display("  Carry = %b", dut.cpu.dp.flag_carry);
            $display("  Negative = %b", dut.cpu.dp.flag_negative);
            $display("");
            $display("I/O:");
            $display("  Port 5 = %h (%0d)", dut.io.ports[5], dut.io.ports[5]);
            $display("");
            
            // verify
            if (dut.cpu.dp.registers[0] == 57'd95 &&
                dut.cpu.dp.registers[1] == 57'd5 &&
                dut.cpu.dp.registers[2] == 57'd190 &&
                dut.cpu.dp.registers[3] == 57'd2 &&
                dut.cpu.dp.registers[4] == 57'd1000 &&
                dut.cpu.cu.halted == 1'b1) begin
                $display("✓ PASS: All tests passed!");
            end else begin
                $display("✗ FAIL: Results don't match expected values");
                if (dut.cpu.dp.registers[0] != 57'd95)
                    $display("  R0: expected 95, got %0d", dut.cpu.dp.registers[0]);
                if (dut.cpu.dp.registers[1] != 57'd5)
                    $display("  R1: expected 5, got %0d", dut.cpu.dp.registers[1]);
                if (dut.cpu.dp.registers[2] != 57'd190)
                    $display("  R2: expected 190, got %0d", dut.cpu.dp.registers[2]);
                if (dut.cpu.dp.registers[3] != 57'd2)
                    $display("  R3: expected 2, got %0d", dut.cpu.dp.registers[3]);
                if (dut.cpu.dp.registers[4] != 57'd1000)
                    $display("  R4: expected 1000, got %0d", dut.cpu.dp.registers[4]);
                if (!dut.cpu.cu.halted)
                    $display("  CPU did not halt");
            end
            $display("==========================================");
        end
    endtask

endmodule