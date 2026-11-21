#ifndef NATIVE_SIM_CISTERN_H
#define NATIVE_SIM_CISTERN_H

#include <stdint.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
void native_sim_cistern_start(void);
void native_sim_cistern_process(void);
#endif

#endif
