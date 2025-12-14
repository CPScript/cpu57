module cpu57 (
    input wire clk,
    input wire rst_n,
    output wire [56:0] pc_out,
    output wire [56:0] sp_out,
    output wire halted,
    output wire [63:0] cycle_count,
    output wire bus_active,
    output wire [56:0] address_bus,
    output wire [56:0] data_bus_out,
    input wire [56:0] data_bus_in,
    output wire mem_write_en,
    output wire mem_read_en,
    output wire [7:0] io_port_addr,
    output wire [56:0] io_port_data_out,
    input wire [56:0] io_port_data_in,
    output wire io_write_en,
    output wire io_read_en
);

    wire [56:0] pc;
    wire [56:0] sp;
    wire flag_zero;
    wire flag_carry;
    wire flag_overflow;
    wire flag_negative;
    wire [7:0] opcode;
    wire [4:0] rd;
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [56:0] immediate;
    
    wire [2:0] state;
    wire [3:0] byte_idx;
    wire execute_enable;
    wire writeback_enable;
    
    datapath dp (
        .clk(clk),
        .rst_n(rst_n),
        .opcode(opcode),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .immediate(immediate),
        .pc(pc),
        .sp(sp),
        .flag_zero(flag_zero),
        .flag_carry(flag_carry),
        .flag_overflow(flag_overflow),
        .flag_negative(flag_negative),
        .data_bus_in(data_bus_in),
        .data_bus_out(data_bus_out),
        .address_bus(address_bus),
        .mem_write_en(mem_write_en),
        .mem_read_en(mem_read_en),
        .io_port_addr(io_port_addr),
        .io_port_data_out(io_port_data_out),
        .io_port_data_in(io_port_data_in),
        .io_write_en(io_write_en),
        .io_read_en(io_read_en),
        .execute_enable(execute_enable),
        .writeback_enable(writeback_enable),
        .state(state),
        .byte_idx(byte_idx)
    );
    
    control_unit cu (
        .clk(clk),
        .rst_n(rst_n),
        .opcode(opcode),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .immediate(immediate),
        .flag_zero(flag_zero),
        .flag_carry(flag_carry),
        .flag_overflow(flag_overflow),
        .flag_negative(flag_negative),
        .state(state),
        .halted(halted),
        .bus_active(bus_active),
        .cycle_count(cycle_count),
        .pc(pc),
        .mem_data_in(data_bus_in),
        .execute_enable(execute_enable),
        .writeback_enable(writeback_enable),
        .byte_idx(byte_idx)
    );
    
    assign pc_out = pc;
    assign sp_out = sp;

endmodule

module datapath (
    input wire clk,
    input wire rst_n,
    input wire [7:0] opcode,
    input wire [4:0] rd,
    input wire [4:0] rs1,
    input wire [4:0] rs2,
    input wire [56:0] immediate,
    output reg [56:0] pc,
    output reg [56:0] sp,
    output reg flag_zero,
    output reg flag_carry,
    output reg flag_overflow,
    output reg flag_negative,
    input wire [56:0] data_bus_in,
    output reg [56:0] data_bus_out,
    output reg [56:0] address_bus,
    output reg mem_write_en,
    output reg mem_read_en,
    output reg [7:0] io_port_addr,
    output reg [56:0] io_port_data_out,
    input wire [56:0] io_port_data_in,
    output reg io_write_en,
    output reg io_read_en,
    input wire execute_enable,
    input wire writeback_enable,
    input wire [2:0] state,
    input wire [3:0] byte_idx
);

    reg [56:0] registers [0:31];
    reg [56:0] fetch_addr;
    wire [56:0] alu_result;
    wire alu_zero;
    wire alu_carry;
    wire alu_overflow;
    wire alu_negative;
    
    wire [56:0] rs1_data;
    wire [56:0] rs2_data;
    
    assign rs1_data = registers[rs1];
    assign rs2_data = registers[rs2];
    
    integer i;
    
    alu alu_inst (
        .opcode(opcode),
        .a(rs1_data),
        .b(rs2_data),
        .result(alu_result),
        .zero(alu_zero),
        .carry(alu_carry),
        .overflow(alu_overflow),
        .negative(alu_negative)
    );
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 57'b0;
            end
            pc <= 57'b0;
            sp <= 57'd1048568;
            flag_zero <= 1'b0;
            flag_carry <= 1'b0;
            flag_overflow <= 1'b0;
            flag_negative <= 1'b0;
            data_bus_out <= 57'b0;
            address_bus <= 57'b0;
            mem_write_en <= 1'b0;
            mem_read_en <= 1'b0;
            io_port_addr <= 8'b0;
            io_port_data_out <= 57'b0;
            io_write_en <= 1'b0;
            io_read_en <= 1'b0;
            fetch_addr <= 57'b0;
        end else begin
            mem_write_en <= 1'b0;
            mem_read_en <= 1'b0;
            io_write_en <= 1'b0;
            io_read_en <= 1'b0;
            
            if (state == 3'b000) begin
                if (byte_idx == 0) begin
                    fetch_addr <= pc;
                    address_bus <= pc;
                    mem_read_en <= 1'b1;
                end else begin
                    fetch_addr <= fetch_addr + 57'd1;
                    address_bus <= fetch_addr + 57'd1;
                    mem_read_en <= 1'b1;
                end
            end
            
            if (execute_enable) begin
                case (opcode)
                    8'h01: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        flag_carry <= alu_carry;
                        flag_negative <= alu_negative;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h02: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        flag_carry <= alu_carry;
                        flag_negative <= alu_negative;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h03: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h04: begin
                        if (rs2_data != 57'b0) begin
                            registers[rd] <= alu_result;
                            flag_zero <= alu_zero;
                        end
                        pc <= pc + 57'd10;
                    end
                    
                    8'h05: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h06: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h07: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h08: begin
                        registers[rd] <= alu_result;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h09: begin
                        registers[rd] <= alu_result;
                        flag_carry <= alu_carry;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h0A: begin
                        registers[rd] <= alu_result;
                        flag_carry <= alu_carry;
                        flag_zero <= alu_zero;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h0B: begin
                        registers[rd] <= alu_result;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h0C: begin
                        registers[rd] <= alu_result;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h10: begin
                        address_bus <= rs1_data;
                        mem_read_en <= 1'b1;
                    end
                    
                    8'h11: begin
                        address_bus <= rs1_data;
                        data_bus_out <= registers[rd];
                        mem_write_en <= 1'b1;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h12: begin
                        registers[rd] <= immediate;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h13: begin
                        registers[rd] <= rs1_data;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h14: begin
                        flag_zero <= alu_zero;
                        flag_carry <= (rs1_data < rs2_data);
                        flag_negative <= alu_negative;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h20: begin
                        pc <= immediate;
                    end
                    
                    8'h21: begin
                        if (flag_zero) begin
                            pc <= immediate;
                        end else begin
                            pc <= pc + 57'd10;
                        end
                    end
                    
                    8'h22: begin
                        if (!flag_zero) begin
                            pc <= immediate;
                        end else begin
                            pc <= pc + 57'd10;
                        end
                    end
                    
                    8'h23: begin
                        if (flag_carry) begin
                            pc <= immediate;
                        end else begin
                            pc <= pc + 57'd10;
                        end
                    end
                    
                    8'h24: begin
                        if (!flag_carry) begin
                            pc <= immediate;
                        end else begin
                            pc <= pc + 57'd10;
                        end
                    end
                    
                    8'h30: begin
                        address_bus <= sp;
                        data_bus_out <= pc + 57'd10;
                        mem_write_en <= 1'b1;
                        sp <= sp - 57'd8;
                        pc <= immediate;
                    end
                    
                    8'h31: begin
                        sp <= sp + 57'd8;
                        address_bus <= sp + 57'd8;
                        mem_read_en <= 1'b1;
                    end
                    
                    8'h32: begin
                        address_bus <= sp;
                        data_bus_out <= registers[rd];
                        mem_write_en <= 1'b1;
                        sp <= sp - 57'd8;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h33: begin
                        sp <= sp + 57'd8;
                        address_bus <= sp + 57'd8;
                        mem_read_en <= 1'b1;
                    end
                    
                    8'h40: begin
                        io_port_addr <= immediate[7:0];
                        io_read_en <= 1'b1;
                        registers[rd] <= io_port_data_in;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h41: begin
                        io_port_addr <= immediate[7:0];
                        io_port_data_out <= registers[rd];
                        io_write_en <= 1'b1;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h00, 8'hFF: begin
                        pc <= pc + 57'd10;
                    end
                    
                    default: begin
                        pc <= pc + 57'd10;
                    end
                endcase
            end
            
            if (writeback_enable) begin
                case (opcode)
                    8'h10: begin
                        registers[rd] <= data_bus_in;
                        pc <= pc + 57'd10;
                    end
                    
                    8'h31: begin
                        pc <= data_bus_in;
                    end
                    
                    8'h33: begin
                        registers[rd] <= data_bus_in;
                        pc <= pc + 57'd10;
                    end
                    
                    default: begin
                    end
                endcase
            end
        end
    end

endmodule

module alu (
    input wire [7:0] opcode,
    input wire [56:0] a,
    input wire [56:0] b,
    output reg [56:0] result,
    output reg zero,
    output reg carry,
    output reg overflow,
    output reg negative
);

    wire [57:0] add_result;
    wire [57:0] sub_result;
    wire [113:0] mul_result;
    
    assign add_result = {1'b0, a} + {1'b0, b};
    assign sub_result = {1'b0, a} - {1'b0, b};
    assign mul_result = a * b;
    
    always @(*) begin
        result = 57'b0;
        zero = 1'b0;
        carry = 1'b0;
        overflow = 1'b0;
        negative = 1'b0;
        
        case (opcode)
            8'h01: begin
                result = add_result[56:0];
                carry = add_result[57];
                zero = (result == 57'b0);
                negative = result[56];
            end
            
            8'h02, 8'h14: begin
                result = sub_result[56:0];
                carry = sub_result[57];
                zero = (result == 57'b0);
                negative = result[56];
            end
            
            8'h03: begin
                result = mul_result[56:0];
                zero = (result == 57'b0);
            end
            
            8'h04: begin
                result = (b != 57'b0) ? (a / b) : 57'b0;
                zero = (result == 57'b0);
            end
            
            8'h05: begin
                result = a & b;
                zero = (result == 57'b0);
            end
            
            8'h06: begin
                result = a | b;
                zero = (result == 57'b0);
            end
            
            8'h07: begin
                result = a ^ b;
                zero = (result == 57'b0);
            end
            
            8'h08: begin
                result = ~a;
                zero = (result == 57'b0);
            end
            
            8'h09: begin
                result = a << b[5:0];
                carry = (b[5:0] > 6'd0) && (a >> (57 - b[5:0])) != 57'b0;
                zero = (result == 57'b0);
            end
            
            8'h0A: begin
                result = a >> b[5:0];
                carry = (b[5:0] > 6'd0) && (a & ((57'b1 << b[5:0]) - 57'b1)) != 57'b0;
                zero = (result == 57'b0);
            end
            
            8'h0B: begin
                result = (a << b[5:0]) | (a >> (57 - b[5:0]));
            end
            
            8'h0C: begin
                result = (a >> b[5:0]) | (a << (57 - b[5:0]));
            end
            
            default: begin
                result = 57'b0;
            end
        endcase
    end

endmodule

module control_unit (
    input wire clk,
    input wire rst_n,
    output reg [7:0] opcode,
    output reg [4:0] rd,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output reg [56:0] immediate,
    input wire flag_zero,
    input wire flag_carry,
    input wire flag_overflow,
    input wire flag_negative,
    output reg [2:0] state,
    output reg halted,
    output reg bus_active,
    output reg [63:0] cycle_count,
    input wire [56:0] pc,
    input wire [56:0] mem_data_in,
    output reg execute_enable,
    output reg writeback_enable,
    output reg [3:0] byte_idx
);

    localparam STATE_FETCH = 3'b000;
    localparam STATE_DECODE = 3'b001;
    localparam STATE_EXECUTE = 3'b010;
    localparam STATE_MEMORY = 3'b011;
    localparam STATE_WRITEBACK = 3'b100;
    
    reg [7:0] inst_byte0;
    reg [7:0] inst_byte1;
    reg [7:0] inst_byte2;
    reg [7:0] inst_byte3;
    reg [7:0] inst_byte4;
    reg [7:0] inst_byte5;
    reg [7:0] inst_byte6;
    reg [7:0] inst_byte7;
    reg [7:0] inst_byte8;
    reg [7:0] inst_byte9;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_FETCH;
            halted <= 1'b0;
            bus_active <= 1'b0;
            cycle_count <= 64'b0;
            opcode <= 8'b0;
            rd <= 5'b0;
            rs1 <= 5'b0;
            rs2 <= 5'b0;
            immediate <= 57'b0;
            byte_idx <= 4'b0;
            execute_enable <= 1'b0;
            writeback_enable <= 1'b0;
            inst_byte0 <= 8'b0;
            inst_byte1 <= 8'b0;
            inst_byte2 <= 8'b0;
            inst_byte3 <= 8'b0;
            inst_byte4 <= 8'b0;
            inst_byte5 <= 8'b0;
            inst_byte6 <= 8'b0;
            inst_byte7 <= 8'b0;
            inst_byte8 <= 8'b0;
            inst_byte9 <= 8'b0;
        end else if (!halted) begin
            cycle_count <= cycle_count + 64'b1;
            execute_enable <= 1'b0;
            writeback_enable <= 1'b0;
            
            case (state)
                STATE_FETCH: begin
                    if (byte_idx == 0) begin
                        bus_active <= 1'b1;
                        byte_idx <= 1;
                    end else if (byte_idx == 1) begin
                        byte_idx <= 2;
                    end else if (byte_idx == 2) begin
                        inst_byte0 <= mem_data_in[7:0];
                        byte_idx <= 3;
                    end else if (byte_idx == 3) begin
                        inst_byte1 <= mem_data_in[7:0];
                        byte_idx <= 4;
                    end else if (byte_idx == 4) begin
                        inst_byte2 <= mem_data_in[7:0];
                        byte_idx <= 5;
                    end else if (byte_idx == 5) begin
                        inst_byte3 <= mem_data_in[7:0];
                        byte_idx <= 6;
                    end else if (byte_idx == 6) begin
                        inst_byte4 <= mem_data_in[7:0];
                        byte_idx <= 7;
                    end else if (byte_idx == 7) begin
                        inst_byte5 <= mem_data_in[7:0];
                        byte_idx <= 8;
                    end else if (byte_idx == 8) begin
                        inst_byte6 <= mem_data_in[7:0];
                        byte_idx <= 9;
                    end else if (byte_idx == 9) begin
                        inst_byte7 <= mem_data_in[7:0];
                        byte_idx <= 10;
                    end else if (byte_idx == 10) begin
                        inst_byte8 <= mem_data_in[7:0];
                        byte_idx <= 11;
                    end else if (byte_idx == 11) begin
                        inst_byte9 <= mem_data_in[7:0];
                        state <= STATE_DECODE;
                        byte_idx <= 0;
                        bus_active <= 1'b0;
                    end
                end
                
                STATE_DECODE: begin
                    opcode <= inst_byte0;
                    rd <= inst_byte1[4:0];
                    rs1 <= inst_byte2[4:0];
                    rs2 <= inst_byte3[4:0];
                    immediate <= {9'b0, inst_byte9, inst_byte8, inst_byte7, inst_byte6, inst_byte5, inst_byte4};
                    state <= STATE_EXECUTE;
                end
                
                STATE_EXECUTE: begin
                    execute_enable <= 1'b1;
                    
                    if (opcode == 8'hFF) begin
                        halted <= 1'b1;
                    end else if (opcode == 8'h10 || opcode == 8'h11 || 
                                opcode == 8'h31 || opcode == 8'h33 || 
                                opcode == 8'h30 || opcode == 8'h32) begin
                        state <= STATE_MEMORY;
                        bus_active <= 1'b1;
                    end else begin
                        state <= STATE_FETCH;
                    end
                end
                
                STATE_MEMORY: begin
                    bus_active <= 1'b1;
                    if (opcode == 8'h10 || opcode == 8'h31 || opcode == 8'h33) begin
                        state <= STATE_WRITEBACK;
                    end else begin
                        state <= STATE_FETCH;
                    end
                end
                
                STATE_WRITEBACK: begin
                    writeback_enable <= 1'b1;
                    bus_active <= 1'b0;
                    state <= STATE_FETCH;
                end
                
                default: begin
                    state <= STATE_FETCH;
                end
            endcase
        end
    end

endmodule

module memory (
    input wire clk,
    input wire rst_n,
    input wire [56:0] address,
    input wire [56:0] data_in,
    output reg [56:0] data_out,
    input wire write_en,
    input wire read_en
);

    (* ram_style = "block" *) reg [7:0] mem [0:1048575];
    wire [19:0] addr;
    
    assign addr = address[19:0];
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= 57'b0;
        end else begin
            if (write_en && address < 57'd1048576) begin
                mem[addr] <= data_in[7:0];
                if (addr < 20'd1048575) mem[addr + 20'd1] <= data_in[15:8];
                if (addr < 20'd1048574) mem[addr + 20'd2] <= data_in[23:16];
                if (addr < 20'd1048573) mem[addr + 20'd3] <= data_in[31:24];
                if (addr < 20'd1048572) mem[addr + 20'd4] <= data_in[39:32];
                if (addr < 20'd1048571) mem[addr + 20'd5] <= data_in[47:40];
                if (addr < 20'd1048570) mem[addr + 20'd6] <= data_in[55:48];
            end
            
            if (read_en && address < 57'd1048576) begin
                data_out <= {1'b0,
                            (addr < 20'd1048570) ? mem[addr + 20'd6] : 8'b0,
                            (addr < 20'd1048571) ? mem[addr + 20'd5] : 8'b0,
                            (addr < 20'd1048572) ? mem[addr + 20'd4] : 8'b0,
                            (addr < 20'd1048573) ? mem[addr + 20'd3] : 8'b0,
                            (addr < 20'd1048574) ? mem[addr + 20'd2] : 8'b0,
                            (addr < 20'd1048575) ? mem[addr + 20'd1] : 8'b0,
                            mem[addr]};
            end
        end
    end

endmodule

module io_controller (
    input wire clk,
    input wire rst_n,
    input wire [7:0] port_addr,
    input wire [56:0] data_in,
    output reg [56:0] data_out,
    input wire write_en,
    input wire read_en
);

    reg [56:0] ports [0:255];
    
    integer i;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            /* verilator lint_off BLKSEQ */
            for (i = 0; i < 256; i = i + 1) begin
                ports[i] = 57'b0;
            end
            /* verilator lint_on BLKSEQ */
            data_out <= 57'b0;
        end else begin
            if (write_en) begin
                ports[port_addr] <= data_in;
            end
            
            if (read_en) begin
                data_out <= ports[port_addr];
            end
        end
    end

endmodule

module cpu57_system (
    input wire clk,
    input wire rst_n
);

    wire [56:0] pc;
    wire [56:0] sp;
    wire halted;
    wire [63:0] cycle_count;
    wire bus_active;
    wire [56:0] address_bus;
    wire [56:0] data_bus_cpu_to_mem;
    wire [56:0] data_bus_mem_to_cpu;
    wire mem_write_en;
    wire mem_read_en;
    wire [7:0] io_port_addr;
    wire [56:0] io_port_data_cpu_to_io;
    wire [56:0] io_port_data_io_to_cpu;
    wire io_write_en;
    wire io_read_en;
    
    cpu57 cpu (
        .clk(clk),
        .rst_n(rst_n),
        .pc_out(pc),
        .sp_out(sp),
        .halted(halted),
        .cycle_count(cycle_count),
        .bus_active(bus_active),
        .address_bus(address_bus),
        .data_bus_out(data_bus_cpu_to_mem),
        .data_bus_in(data_bus_mem_to_cpu),
        .mem_write_en(mem_write_en),
        .mem_read_en(mem_read_en),
        .io_port_addr(io_port_addr),
        .io_port_data_out(io_port_data_cpu_to_io),
        .io_port_data_in(io_port_data_io_to_cpu),
        .io_write_en(io_write_en),
        .io_read_en(io_read_en)
    );
    
    memory mem (
        .clk(clk),
        .rst_n(rst_n),
        .address(address_bus),
        .data_in(data_bus_cpu_to_mem),
        .data_out(data_bus_mem_to_cpu),
        .write_en(mem_write_en),
        .read_en(mem_read_en)
    );
    
    io_controller io (
        .clk(clk),
        .rst_n(rst_n),
        .port_addr(io_port_addr),
        .data_in(io_port_data_cpu_to_io),
        .data_out(io_port_data_io_to_cpu),
        .write_en(io_write_en),
        .read_en(io_read_en)
    );

endmodule