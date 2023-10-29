#include <stdint.h>
extern "C" {
extern int read_mem1(void*, uint32_t, uint32_t, uint64_t, uint8_t*);
extern int write_mem1(void*, uint32_t, uint32_t, uint64_t, uint8_t);
extern int read_mem2(void*, uint32_t, uint32_t, uint64_t, uint16_t*);
extern int write_mem2(void*, uint32_t, uint32_t, uint64_t, uint16_t);
extern int read_mem4(void*, uint32_t, uint32_t, uint64_t, uint32_t*);
extern int write_mem4(void*, uint32_t, uint32_t, uint64_t, uint32_t);
extern int read_mem8(void*, uint32_t, uint32_t, uint64_t, uint64_t*);
extern int write_mem8(void*, uint32_t, uint32_t, uint64_t, uint64_t);
extern uint64_t enter_trap(void*, uint64_t, uint64_t, uint64_t);
extern uint64_t leave_trap(void*, uint64_t);
extern void wait(void*, uint64_t);
extern void print_string(void*, char*);
extern void print_disass(void*, uint64_t, char*);
extern void pre_instr_sync(void*);
extern void notify_phase(void*, uint32_t);
extern void call_plugin(void*, uint64_t);
extern uint8_t read_mem_buf[];
}
