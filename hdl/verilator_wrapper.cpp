#include <verilated.h>
#include "Vcpu57_system.h"
#include "Vcpu57_system___024root.h"
#include <cstring>
#include <cstdint>

struct HDLCPUState {
    Vcpu57_system* top;
    VerilatedContext* contextp;
    uint64_t time;
};

extern "C" {

HDLCPUState* hdl_cpu_init() {
    HDLCPUState* state = new HDLCPUState();
    state->contextp = new VerilatedContext();
    state->top = new Vcpu57_system(state->contextp);
    state->time = 0;
    
    state->top->clk = 0;
    state->top->rst_n = 0;
    state->top->eval();
    
    return state;
}

void hdl_cpu_reset(HDLCPUState* state) {
    state->top->rst_n = 0;
    state->top->clk = 0;
    state->top->eval();
    state->time += 10;
    
    state->top->clk = 1;
    state->top->eval();
    state->time += 10;
    
    state->top->rst_n = 1;
    state->top->clk = 0;
    state->top->eval();
    state->time += 10;
}

void hdl_cpu_clock(HDLCPUState* state) {
    state->top->clk = 0;
    state->top->eval();
    state->time += 5;
    
    state->top->clk = 1;
    state->top->eval();
    state->time += 5;
}

void hdl_cpu_load_memory(HDLCPUState* state, uint64_t addr, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len && (addr + i) < 1048576; i++) {
        state->top->rootp->cpu57_system__DOT__mem__DOT__mem[addr + i] = data[i];
    }
}

uint64_t hdl_cpu_read_register(const HDLCPUState* state, uint8_t reg) {
    if (reg >= 32) return 0;
    return state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__registers[reg] & 0x1FFFFFFFFFFFFFF;
}

uint64_t hdl_cpu_read_pc(const HDLCPUState* state) {
    return state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__pc & 0x1FFFFFFFFFFFFFF;
}

uint64_t hdl_cpu_read_sp(const HDLCPUState* state) {
    return state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__sp & 0x1FFFFFFFFFFFFFF;
}

uint8_t hdl_cpu_read_flags(const HDLCPUState* state) {
    uint8_t flags = 0;
    if (state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__flag_zero) flags |= 0x01;
    if (state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__flag_carry) flags |= 0x02;
    if (state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__flag_overflow) flags |= 0x04;
    if (state->top->rootp->cpu57_system__DOT__cpu__DOT__dp__DOT__flag_negative) flags |= 0x08;
    return flags;
}

bool hdl_cpu_is_halted(const HDLCPUState* state) {
    return state->top->rootp->cpu57_system__DOT__cpu__DOT__cu__DOT__halted;
}

uint64_t hdl_cpu_get_cycle_count(const HDLCPUState* state) {
    return state->top->rootp->cpu57_system__DOT__cpu__DOT__cu__DOT__cycle_count;
}

void hdl_cpu_read_memory(const HDLCPUState* state, uint64_t addr, uint8_t* data, size_t len) {
    for (size_t i = 0; i < len && (addr + i) < 1048576; i++) {
        data[i] = state->top->rootp->cpu57_system__DOT__mem__DOT__mem[addr + i];
    }
}

uint64_t hdl_cpu_read_io_port(const HDLCPUState* state, uint8_t port) {
    return state->top->rootp->cpu57_system__DOT__io__DOT__ports[port] & 0x1FFFFFFFFFFFFFF;
}

void hdl_cpu_get_bus_state(const HDLCPUState* state, uint64_t* addr_out, uint64_t* data_out, bool* active_out) {
    *addr_out = state->top->rootp->cpu57_system__DOT__address_bus & 0x1FFFFFFFFFFFFFF;
    *data_out = state->top->rootp->cpu57_system__DOT__data_bus_mem_to_cpu & 0x1FFFFFFFFFFFFFF;
    *active_out = state->top->rootp->cpu57_system__DOT__bus_active;
}

void hdl_cpu_destroy(HDLCPUState* state) {
    delete state->top;
    delete state->contextp;
    delete state;
}

}