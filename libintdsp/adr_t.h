#include "libintdsp.h"

struct adr_t{
  spl_t spl;
  int32_t a_v, r_v, a;
  int32_t acc,vv,v;
  bool state, old_state;
};