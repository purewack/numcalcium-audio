#include <libmaple/libmaple_types.h>
#include <libmaple/timer.h>
#include <libmaple/dma.h>
#include <libmaple/gpio.h>
#include "boards.h"
#include "io.h"
#define LOGL(X) Serial.println(X)
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

#define BENCH_PIN COMMS_SDA

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

struct osc_t{
  uint16_t phi;
  uint16_t acc;
  int16_t* table;
};

osc_t osc;
int16_t sint[256];


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
  dma_dev* dma;
  dma_channel dma_ch;
  timer_dev* timer;
  void* port;
  uint32_t dout_bits[64];
  uint16_t buf[128];
  uint8_t buf_len;
  uint8_t buf_i;
  uint8_t req = 0;
} i2s;
// #define COMMS_MOSI PB15 //DOUT - bsr.15.31
// #define COMMS_MISO PB14 
// #define COMMS_CK   PB13 
// #define COMMS_CS PB12   //WS - bsr.12.28
void i2s_bits_irq(){
  auto r = dma_get_irq_cause(i2s.dma, i2s.dma_ch) == DMA_TRANSFER_COMPLETE ? 1 : 0;
  auto rr = 16*r;
  auto ws = (0x10000000>>rr) | 0x80000000;
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

void setup() {
  disableDebugPorts();
  Serial.begin(9600);
  LOGL("setup start");
  for(int i=0; i<256; i++){
    sint[i] = int16_t(512.f * sin(2.0f * 3.1415f * float(i)/256.f));
  }
  int f = 60;
  osc.table = sint;
  osc.acc = 900;//(256*f)/30125;
  osc.phi = 0;

//   for(int i=0; i<4; i++)
//     pinMode(kmux.cols[i], INPUT_PULLDOWN);
//   for(int i=0; i<6; i++)
//     pinMode(kmux.rows[i], OUTPUT);

//   kmux.itr = 0;
//   kmux.timer = TIMER3;
//   timer_pause(kmux.timer);
//   //48M / 48 / 2000 = 25hz*20keys
//   timer_set_prescaler(kmux.timer, 48);
//   timer_set_reload(kmux.timer, 2000);
//   timer_attach_interrupt(kmux.timer, TIMER_UPDATE_INTERRUPT, kmux_irq);
//   timer_enable_irq(kmux.timer, TIMER_UPDATE_INTERRUPT);
//   timer_resume(kmux.timer);
  
  for(int i=0; i<128;i+=2){
    if(i<64) {
      i2s.buf[i] = 3000;
      i2s.buf[i+1] = -1000;
    }
    else {
      i2s.buf[i] = -1000;
      i2s.buf[i+1] = 3000;
    }
  }


  i2s.dma = DMA1;
  i2s.dma_ch = DMA_CH4;
  i2s.port = (void*)&(GPIOB->regs->BSRR);
  i2s.buf_len = 128;
  i2s.buf_i = 0;
  
  //timer4ch2 PB7 BCK
  gpio_set_mode(GPIOB, 15, GPIO_OUTPUT_PP);
  gpio_set_mode(GPIOB, 12, GPIO_OUTPUT_PP);
  pinMode(COMMS_SDA, PWM);
  timer_pause(TIMER4);
  timer_set_prescaler(TIMER4, 0);
  timer_set_compare(TIMER4, TIMER_CH2, 17);
  timer_set_reload(TIMER4, 48);
  timer_dma_enable_req(TIMER4, TIMER_CH2);
  //timer_attach_interrupt(TIMER4, TIMER_CH2, i2s_data_irq);
  timer_resume(TIMER4);



  dma_init(i2s.dma);
  dma_disable(i2s.dma, i2s.dma_ch);
  int m = DMA_TRNS_CMPLT | DMA_HALF_TRNS | DMA_FROM_MEM | DMA_CIRC_MODE | DMA_MINC_MODE;
  dma_setup_transfer(i2s.dma, i2s.dma_ch , i2s.port, DMA_SIZE_32BITS, i2s.dout_bits, DMA_SIZE_32BITS, m);
  dma_set_num_transfers(i2s.dma, i2s.dma_ch, 32);  
  dma_set_priority(i2s.dma, i2s.dma_ch, DMA_PRIORITY_HIGH);
  dma_attach_interrupt(i2s.dma, i2s.dma_ch, i2s_bits_irq);
  dma_enable(i2s.dma, i2s.dma_ch);

  
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
  // digitalWrite(BENCH_PIN,HIGH);
    int s = 0;
    int e = i2s.buf_len>>1;
    if(i2s.req == 2){
      s = i2s.buf_len>>1;
      e = i2s.buf_len;
    }
    i2s.req = 0;

    auto a = osc.acc;//kmux.io[4].state ? osc.acc : osc.acc*2;

    for(int i=s; i<e; i+=2){
      auto phi_dt = osc.phi & 0xFF;
      auto phi_l = osc.phi >> 8;
      auto phi_h = (phi_l + 1);
      phi_h = phi_h & 0xFF;
      
      auto s_l = osc.table[phi_l];
      auto s_h = osc.table[phi_h];
      
      auto intp = ( (s_h-s_l)*phi_dt ) >> 8;
      int spl = s_l + intp;
      spl = (spl*100 )>>8;
      i2s.buf[i] = uint16_t(spl + 512);
      i2s.buf[i+1] = uint16_t(spl + 512);
      osc.phi += a;
    }
  // digitalWrite(BENCH_PIN,LOW);
  }
}
