#include <libmaple/libmaple_types.h>
#include <libmaple/timer.h>
#include <libmaple/dma.h>
#include <libmaple/gpio.h>
#include "boards.h"
#include "io.h"

#define DEBUG
#define VERBOSE
#define LOGL(X) Serial.println(X)
#define LOGNL(X) Serial.print(X)
#include "libintdsp/libintdsp.h"
#include "libintdsp/osc_t.h"
#include "libintdsp/adr_t.h"
#include "libintdsp/lpf_t.h"
#include "libintdsp/private/libintdsp.cpp"
#include "libintdsp/private/nodes.cpp"
agraph_t gg;
int16_t spl_out_a,spl_out_b;


#define SYS_PDOWN PB5

#define LCD_LIGHT PA6
#define LCD_CS PA4
#define LCD_DC PA3
#define LCD_CK PA5
#define LCD_RST PA2
#define LCD_MOSI PA7

#define SEG_A PB11
#define SEG_B PB10
#define SEG_C PB1
#define SEG_D PB0

#define ROW_A PA14
#define ROW_B PA15
#define ROW_C PB3
#define ROW_D PB4
#define ROW_E PB8
#define ROW_F PB9

#define B_OK PA8

#define GP_A PA0
#define GP_B PA1
////////////////////////
#define COMMS_SDA PB7 //BCK
#define COMMS_SDL PB6
////////////////////////
#define COMMS_MOSI PB15 //DOUT - bsr.15.31
#define COMMS_MISO PB14 
#define COMMS_CK   PB13 
#define COMMS_CS PB12   //WS - bsr.12.28
///////////////////////
#define COMMS_RX PA10
#define COMMS_TX PA9
////////////////////////

#define K_Y 0
#define K_0 1
#define K_DOT 2
#define K_R 3
#define K_1 4
#define K_2 5
#define K_3 6
#define K_P 7
#define K_4 8
#define K_5 9
#define K_6 10
#define K_N 11
#define K_7 12
#define K_8 13
#define K_9 14
#define K_D 15
#define K_F1 16
#define K_F2 17
#define K_F3 18
#define K_X 19

struct t_io {
  volatile uint8_t state;
  volatile uint8_t old_state;
};

struct HW
{
  volatile t_io io[20];
  const uint8_t rows[6] = {ROW_A, ROW_B, ROW_C, ROW_D, ROW_E, ROW_F};
  const uint8_t cols[4] = {SEG_A, SEG_B, SEG_C, SEG_D};
  uint8_t itr;
  uint8_t row;
  timer_dev* timer;
} kmux;

void kmux_irq(){
  digitalWrite(kmux.rows[kmux.row],1);
  kmux.io[kmux.itr+0].state = digitalRead(kmux.cols[0]);
  kmux.io[kmux.itr+1].state = digitalRead(kmux.cols[1]);
  kmux.io[kmux.itr+2].state = digitalRead(kmux.cols[2]);
  kmux.io[kmux.itr+3].state = digitalRead(kmux.cols[3]);
  digitalWrite(kmux.rows[kmux.row],0);

  kmux.itr = (kmux.itr+4)%20;
  kmux.row = (kmux.row+1)%5;
}


void benchSetup(){
  pinMode(PA0, OUTPUT);
}

void benchStart(){
  GPIOA->regs->BSRR = 1<<0;
}

void benchEnd(){
  GPIOA->regs->BSRR = (1<<0)<<16;
}
//TIM4 CH 1 = PB6
//TIM4 CH 2 = PB7
//TIM1 CH2  = PA9
//TIM1 CH3  = PA10
//TIM2 CH2  = PA1
//https://i0.wp.com/blog.io-expert.com/wp-content/uploads/2019/08/clocks.png
//bit banging of WS, BCK, DATA on PB port pins 12,13,14 respectively
//GPIOB_BSSR register [0:15] set, [16:31] reset, non intrusive on other port pins - dma compatible

//either 
//  Timer_IRQ@srate -> (buf -> DMA -> GPIO_BSSR)
//or
//  DMA_IRQ(n)@halfbuf -> (buf -> Timer(n) -> Timer(n)CCMP_Pin)
struct soft_i2s_t{
  uint32_t dout_bits[32];
  int16_t buf[128];
  uint8_t buf_len;
  uint8_t buf_i;
  uint8_t req = 0;
} i2s;

// #define COMMS_MOSI PB15 //DOUT - bsr.15.31
// #define COMMS_MISO PB14 
// #define COMMS_CK   PB13 
// #define COMMS_CS PB12   //WS - bsr.12.28
void i2s_bits_irq(){
  auto r = dma_get_irq_cause(DMA1, DMA_CH2) == DMA_TRANSFER_COMPLETE ? 1 : 0;
  auto rr = 16*r;
  auto ws = 0x90000000 | (0x1000*r);
  auto s = i2s.buf[i2s.buf_i];
  i2s.dout_bits[0+rr] = (( s & (0x8000>>0))<<0) | ws;
  i2s.dout_bits[1+rr] = (( s & (0x8000>>1))<<1) | ws;
  i2s.dout_bits[2+rr] = (( s & (0x8000>>2))<<2) | ws;
  i2s.dout_bits[3+rr] = (( s & (0x8000>>3))<<3) | ws;

  i2s.dout_bits[4+rr] = (( s & (0x8000>>4))<<4) | ws;
  i2s.dout_bits[5+rr] = (( s & (0x8000>>5))<<5) | ws;
  i2s.dout_bits[6+rr] = (( s & (0x8000>>6))<<6) | ws;
  i2s.dout_bits[7+rr] = (( s & (0x8000>>7))<<7) | ws;
  
  i2s.dout_bits[8 +rr] = (( s & (0x8000>>8 ))<<8 ) | ws;
  i2s.dout_bits[9 +rr] = (( s & (0x8000>>9 ))<<9 ) | ws;
  i2s.dout_bits[10+rr] = (( s & (0x8000>>10))<<10) | ws;
  i2s.dout_bits[11+rr] = (( s & (0x8000>>11))<<11) | ws;
  
  i2s.dout_bits[12+rr] = (( s & (0x8000>>12))<<12) | ws;
  i2s.dout_bits[13+rr] = (( s & (0x8000>>13))<<13) | ws;
  i2s.dout_bits[14+rr] = (( s & (0x8000>>14))<<14) | ws;
  i2s.dout_bits[15+rr] = (( s & (0x8000>>15))<<15) | ws;

  if(i2s.buf_i+1 == i2s.buf_len) i2s.req = 2;
  if(i2s.buf_i+1 == i2s.buf_len>>1) i2s.req = 1;
  i2s.buf_i = (i2s.buf_i+1)%i2s.buf_len;
}


node_t* adr1;
node_t* lpf1;
void setup() {
  disableDebugPorts();
  benchSetup();
  Serial.begin(9600);
  LOGL("setup start");

//////////////////////////// 

  libintdsp_init(&gg);  
  auto* dac = new_dac(&gg,"dac",&spl_out_a);
  auto* dac2 = new_dac(&gg,"dac2",&spl_out_b);

  auto os1 = new_osc(&gg,"osca");
  auto os2 = new_osc(&gg,"oscb");
    auto* os1_params = (osc_t*)os1->processor;
    auto* os2_params = (osc_t*)os2->processor;
    os1_params->acc = 1800;
    os2_params->acc = 1200;
    os1_params->gain = 20;
    os2_params->gain = 20;
    os2_params->table = sawt;

  auto* os3 = new_osc(&gg,"oscc");
    auto* os3_params = (osc_t*)os3->processor;
    os3_params->acc = 750;
    os3_params->gain = 30;
    
  auto* lfo = new_osc(&gg,"lfo");
    auto* lfo_params = (osc_t*)lfo->processor;
    lfo_params->acc = 4;
    lfo_params->bias = 1500;
    lfo_params->gain = 5;

  lpf1 = new_lpf(&gg,"lpf");
    
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


///////////////////

  for(int i=0; i<4; i++)
    pinMode(kmux.cols[i], INPUT_PULLDOWN);
  for(int i=0; i<6; i++)
    pinMode(kmux.rows[i], OUTPUT);

  kmux.itr = 0;
  kmux.timer = TIMER3;
  timer_pause(kmux.timer);
  //48M / 48 / 2000 = 25hz*20keys
  timer_set_prescaler(kmux.timer, 48);
  timer_set_reload(kmux.timer, 2000);
  timer_attach_interrupt(kmux.timer, TIMER_UPDATE_INTERRUPT, kmux_irq);
  timer_enable_irq(kmux.timer, TIMER_UPDATE_INTERRUPT);
  timer_resume(kmux.timer);
  
///////////////////////
  i2s.buf_len = 128;
  i2s.buf_i = 0;

  //timer4ch2 PB7 BCK
  gpio_set_mode(GPIOB, 15, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 12, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 13, GPIO_AF_OUTPUT_PP);
  timer_pause(TIMER1);
  timer_set_prescaler(TIMER1, 0);
  timer_set_compare(TIMER1, TIMER_CH1, 24-1);
  timer_set_reload(TIMER1, 48-1);
  timer_dma_enable_req(TIMER1, TIMER_CH1);
  (TIMER1->regs.adv)->CCER |= 0b101;
 
  dma_init(DMA1);
  dma_disable(DMA1, DMA_CH2);
  int m = DMA_TRNS_CMPLT | DMA_HALF_TRNS | DMA_FROM_MEM | DMA_CIRC_MODE | DMA_MINC_MODE;
  dma_setup_transfer(DMA1, DMA_CH2 , (void*)&(GPIOB->regs->BSRR), DMA_SIZE_32BITS, i2s.dout_bits, DMA_SIZE_32BITS, m);
  dma_set_num_transfers(DMA1, DMA_CH2, 32);  
  dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_HIGH);
  dma_attach_interrupt(DMA1, DMA_CH2, i2s_bits_irq);
  dma_enable(DMA1, DMA_CH2);

  timer_resume(TIMER1);
//////////////////////
  
  LOGL("setup complete");
}

void loop() {
  // for(int i=0; i<6; i++){
  //   LOGL(kmux.io[i].state); 
  // }
  // LOGL("-------");
  // LOGL(kmux.itr);
  // LOGL("-------");
  // delay(500);

  if(i2s.req){
    benchStart();
    int s = 0;
    int e = i2s.buf_len>>1;
    if(i2s.req == 2){
      s = i2s.buf_len>>1;
      e = i2s.buf_len;
    }
    i2s.req = 0;

    ((adr_t*)(adr1->processor))->state = kmux.io[0].state;
    set_lpf_freq((lpf_t*)(lpf1->processor), kmux.io[1].state ? 8000 : 20000, 31250);

    for(int i=s; i<e; i+=2){
      proc_graph(&gg);
      i2s.buf[i] = spl_out_a;
      i2s.buf[i+1] = spl_out_b;
    }
    benchEnd();
  }
}
