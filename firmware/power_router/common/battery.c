#include "battery.h"
#include "pin.h"
#include <stdio.h>
#include "state.h"

// hardware connections:

// CAN1R = PB8 on AF9
// CAN1D = PB9 on AF9

// CAN2R = PB5 on AF9
// CAN2D = PB6 on AF9

CAN_TypeDef *can1 = CAN1;

static void battery_can_init(CAN_TypeDef *can)
{
  // go to initialization mode
  can->MCR &= ~CAN_MCR_SLEEP; // stop being lazy. wake up
  can->MCR |= CAN_MCR_INRQ; // request initialization mode
  while (!can->MSR & CAN_MSR_INAK); // wait until we are in init mode
  can->MCR |= CAN_MCR_NART;
  can->BTR = 0x0145000d; // this is for 250kbit 
  can->MCR &= ~CAN_MCR_INRQ; // request to leave initialization mode
  while (can->MSR & CAN_MSR_INAK); // wait until we leave init mode
  can->FMR |= CAN_FMR_FINIT; // enter filter init mode
  can->FS1R |= 1 | 1 << 14; // first filter is 32 bits wide
  can->sFilterRegister[0].FR1  = 0x1000000;
  can->sFilterRegister[0].FR2  = 0x1000000;
  can->sFilterRegister[14].FR1 = 0x1000000;
  can->sFilterRegister[14].FR2 = 0x1000000;
  can->FA1R |= 1 | 1 << 14; // activate filters #1 and #14
  can->FMR &= ~CAN_FMR_FINIT; // exit filter init mode
  can->IER |= CAN_IER_FMPIE0; // fifo message pending interrupt enable
}

void battery_init()
{
  printf("battery_init()\r\n");
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN |
                  RCC_APB1ENR_CAN2EN;
  pin_set_alternate_function(GPIOB, 8, 9);
  pin_set_alternate_function(GPIOB, 9, 9);
  pin_set_alternate_function(GPIOB, 5, 9);
  pin_set_alternate_function(GPIOB, 6, 9);

  battery_can_init(CAN1);
  battery_can_init(CAN2);

  NVIC_SetPriority(CAN1_RX0_IRQn, 4); // low priority
  NVIC_SetPriority(CAN2_RX0_IRQn, 4);
  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /*
  // send a packet to grab bit widths on logic-analyzer to debug timing
  CAN1->sTxMailBox[0].TIR = 0x12345678;
  CAN1->sTxMailBox[0].TDTR = 0x8; // tx 8 bytes
  CAN1->sTxMailBox[0].TDLR = 0x42425353;
  CAN1->sTxMailBox[0].TDHR = 0x98765432;
  CAN1->sTxMailBox[0].TIR |= 1; // set txrq bit
  */
}

static void battery_can_rx(CAN_TypeDef *can, const uint32_t can_idx)
{
  if (can_idx > 1)
    return; // get outta here
  const uint32_t id   = can->sFIFOMailBox[0].RIR; // receive ID
  const uint32_t dtr  = can->sFIFOMailBox[0].RDTR;
  const uint32_t low  = can->sFIFOMailBox[0].RDLR;
  const uint32_t high = can->sFIFOMailBox[0].RDHR;
  const uint32_t pgn = ((id >> 3) & 0xffff00) >> 8;
  const uint32_t len = dtr & 0xf;
  /*
  printf("can %d rx pgn 0x%04x rx %d bytes\r\n", 
         (int)can_idx, (unsigned)pgn, (int)len);
  */
  /*
  printf("rx id = 0x%08x\r\n", (unsigned)(id >> 3));
  printf("rx len = %d\r\n", (unsigned)(dtr & 0xf));
  printf("rx msg = 0x%08x%08x\r\n", (unsigned)low, (unsigned)high);
  */
  if (pgn == 0xff21 && len == 7)
  {
    const float i = (float)(low & 0xffff) * 0.05f - 1600.0f;
    const float v = (float)((low & 0xffff0000) >> 16) * 0.05f;
    const float celsius = (float)(high & 0xff) - 40.0f;
    /*
    printf("  info msg: %.2f volts %.2f amps %.0f deg C\r\n", 
           v, i, celsius);
    */
    g_state.battery_voltage[can_idx] = v;
    g_state.battery_current[can_idx] = i;
    g_state.battery_celsius[can_idx] = celsius;
  }
  can->RF0R |= CAN_RF0R_RFOM0; // release mailbox
}

void can1_rx0_vector()
{
  battery_can_rx(CAN1, 0);
}

void can2_rx0_vector()
{
  battery_can_rx(CAN2, 1);
}
