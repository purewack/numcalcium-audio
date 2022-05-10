#include "../libintdsp.h"
#include "../osc_t.h"
#include "../adr_t.h"

void proc_osc(void* v){
  osc_t* oo = (osc_t*)v;
  int16_t phi_dt, phi_l, phi_h;
  int16_t s_l, s_h;
  int16_t intp;

  phi_dt = oo->phi & 0xFF;
  phi_l = oo->phi >> 8;
  phi_h = (phi_l + 1);
  phi_h = phi_h & 0xFF;

  s_l = oo->table[phi_l];
  s_h = oo->table[phi_h];
  
  intp = ( (s_h-s_l)*phi_dt ) >> 8;
  oo->io->out = oo->bias + (((s_l + intp)*oo->gain)>>8);

  if(oo->io->deps_count)
    oo->phi += oo->io->in;
  else
    oo->phi += oo->acc;
}
node_t* new_osc(agraph_t* gg, char* sig){
    node_t* b = new_node(gg,sig);
    osc_t* n = (osc_t*)malloc(sizeof(osc_t));
    LOGL("new node: osc");
    LOGL(sig);
    n->bias = 0;
    n->acc = 900;
    n->table = sint;
    n->gain = 255;
    b->processor = n;
    b->processor_func = proc_osc;
    n->io = b;
    return b;
}


// node_t* new_adr(agraph_t* gg, char* sig){
//     node_t* b = new_node(gg,sig);
//     adr_t* n = malloc(sizeof(adr_t));
//     b->processor = n;
//     b->processor_func = proc_adr;
//     n->io = b;
//     return b;
// }
// void proc_adr(adr_t* aa){
//   if(aa->state && !aa->old_state)
//     aa->a = aa->a_v;
//   else if(!aa->state && aa->old_state)
//     aa->a = -aa->r_v;
//   if(aa->state != aa->old_state)
//     aa->old_state = aa->state;
  
//   //if(aa->aa == 0) return;
  
//   aa->vv += aa->a;
//   if(aa->vv > 1048576) aa->vv = 1048576;
//   if(aa->vv < 0) aa->vv = 0;
//   aa->v = aa->vv >> 8;
  
//   aa->acc = (aa->spl * aa->v);
//   aa->spl = aa->acc >> 12; 
// }

// void proc_lpf(lpf_t* p){
//   p->h = p->h + ((p->a * (p->spl-p->h))>>12);
//   p->h2 = p->h2 + ((p->a * (p->h-p->h2))>>12);
//   p->h3 = p->h3 + ((p->a * (p->h2-p->h3))>>12);
//   p->spl = p->h3;
// }
