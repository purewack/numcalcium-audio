#include "libintdsp.h"

struct lpf_t{
  spl_t spl, 
  int16_t h, h2, h3;
  int16_t a;
};

void proc_lpf(lpf_t* node);
