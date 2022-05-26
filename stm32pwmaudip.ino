#include <libmaple/libmaple_types.h>
#include <libmaple/timer.h>
#include <libmaple/dma.h>
#include <libmaple/gpio.h>
#include "boards.h"
#include "io.h"

#include "numcalcium-base/hw.h"
#include "numcalcium-base/hw.cpp"

#define DEBUG
#define VERBOSE
#define LOGL(X) Serial.println(X)
#define LOGNL(X) Serial.print(X)
#include "libintdsp/libintdsp.h"
#include "libintdsp/config.h"
#include "libintdsp/_source/types.h"
#include "libintdsp/_source/dsp.h"
#include "libintdsp/_source/lifecycle.h"
#include "libintdsp/_source/tables.h"
#include "libintdsp/_source/init.c"
#include "libintdsp/_source/graph.c"
#include "libintdsp/_source/nodes.c"
agraph_t gg;
int16_t spl_out_a,spl_out_b;


int16_t sin_setup(int16_t p){
  return int16_t(sin(2.f*3.1415f * float(p)/float(LUT_COUNT)));
}

node_t* adr1;
node_t* lpf1;
void setup() {
  io_mux_init();
  soft_i2s_init();
  Serial.begin(9600);
  LOGL("setup start");

//////////////////////////// 

  libintdsp_init(&gg,sin_setup);  
  auto* dac = new_dac(&gg,"dac",&spl_out_a);
  auto* dac2 = new_dac(&gg,"dac2",&spl_out_b);

  auto os1 = new_osc(&gg,"osca");
  auto os2 = new_osc(&gg,"oscb");
    auto* os1_params = (osc_t*)os1->processor;
    auto* os2_params = (osc_t*)os2->processor;
    set_osc_freq(os1_params,4400,31250);
    set_osc_freq(os2_params,3300,31250);
    os1_params->gain = 20;
    os2_params->gain = 20;
    os2_params->table = sawt;

  auto* os3 = new_osc(&gg,"oscc");
    auto* os3_params = (osc_t*)os3->processor;
    set_osc_freq(os3_params,550,31250);
    os3_params->gain = 30;
    
  auto* lfo = new_osc(&gg,"lfo");
    auto* lfo_params = (osc_t*)lfo->processor;
    lfo_params->acc = 4;
    lfo_params->bias = 1500;
    lfo_params->gain = 5;

  lpf1 = new_lpf(&gg,"lpf");
    set_lpf_freq((lpf_t*)(lpf1->processor), 24000, 31250);
    
  adr1 = new_adr(&gg, "adr1");
  set_adr_attack_ms((adr_t*)(adr1->processor), 1000, 31250);
  set_adr_release_ms((adr_t*)(adr1->processor), 500, 31250);

  LOGL("connecting");
  connect(&gg,os1,adr1);
  connect(&gg,os2,lpf1);
  connect(&gg,lpf1,dac);
  connect(&gg,adr1,dac);
  connect(&gg,os3,dac2);
  connect(&gg,lfo,os3);

  // connect(&gg,os1,dac2);
  // connect(&gg,os1,adr1);
  // connect(&gg,adr1,dac);
  LOGL("connected");
}

void loop() {
  // for(int i=0; i<6; i++){
  //   LOGL(kmux.io[i].state); 
  // }
  // LOGL("-------");
  // LOGL(kmux.itr);
  // LOGL("-------");
  // delay(500);
  // if(kmux.turns != kmux.turns_old){
  //   kmux.turns_old = kmux.turns;
  //   Serial.println(kmux.turns);
  // }

  if(io.bscan_down & BUTTON(2)){
    C_BIT(io.bscan_down, 2);
    set_lpf_freq((lpf_t*)(lpf1->processor), 8000, 31250);
  } 
  if(io.bscan_up & BUTTON(3)){
    C_BIT(io.bscan_up, 3);
    set_lpf_freq((lpf_t*)(lpf1->processor), 24000, 31250);
  }
 
  if(abuf.req){
    //benchStart();
    int s = 0;
    int e = abuf.buf_len>>1;
    if(abuf.req == 2){
      s = abuf.buf_len>>1;
      e = abuf.buf_len;
    }
    abuf.req = 0;

    //((adr_t*)(adr1->processor))->state = kmux.io[0].state&STATE_BIT_CURRENT;
    
    for(int i=s; i<e; i+=2){
      proc_graph(&gg);
      abuf.buf[i] = spl_out_a;
      abuf.buf[i+1] = spl_out_b;
    }
    //benchEnd();
  }
}
