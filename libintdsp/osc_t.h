#pragma once
#include "libintdsp.h"

struct osc_t{
  node_t* io;
  uint16_t phi;
  uint16_t acc;
  int16_t bias;
  int16_t gain;
  int16_t* table;
};
//range 0-65xxx
void proc_osc(void* processor);
node_t* new_osc(agraph_t* graph, char* signature);
