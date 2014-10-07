/**
 * \file fcc_test_dut.c
 *
 * Copyright (C) 2012-2014, Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 * Modification and other use of this code is subject to Atmel's Limited
 * License Agreement (license.txt).
 *
 *
 * -------------------- fcc_test_dut.c --------------------------------------
 *
 * Nota Bene:
 * The register values are shown by TIB Status.
 * See SAMR21 Datasheet for details
 *
 * CH11 = 2405 MHz (Lowest Frequency)
 * CH18 = 2440 MHz (Mid-frequency default)
 * CH26 = 2480 MHz (Highest Frequency)
 * ANT1 = SMA
 * ANT2 = Chip
 * TX_PWR = 0x00 is maximum power +3.5 dBm
 * TX_PWR = 0x07 is 0dBm (default)
 * TX_PWR = 0x0F in minimum power -17 dBm
 * XTAL_TRIM = 0x00 is +30 kHz
 * XTAL_TRIM = 0x08 is +0 kHz (default)
 * XTAL_TRIM = 0x0f is -30 kHz
 * 
 * --------- revision history ---------------------------------
 * rev 2.1.2 initial release working with 48-pin XPRO. PW 2014-08-04
 * rev 2.2.2 cleaned up appSetCwMode(). Added while(1) loop to prevent CPU access to TRX during testing.
 * rev 2.3.2 added XTAL_TRIM adjustment. PW 2014-09-12
 *
 *
 * -------------------------------------------------------------------------*/

/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#include "halBoard.h"
#include "halUart.h"
#include "halLed.h"
#include "at86rf233.h"
#include "halPhy.h"

/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
  #define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
  #define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

//*** NEW STUFF ***//
#define SPLASH_STRING "Atmel Remote TX Test"
#define VERSION_ID "V2.3.2"
#define TOP_MENU "Menu: T)EST s)tatus C)H++ c)h-- P)OW++ p)ow-- a)nt F)REQ++ f)req--"
#define DEFAULT_CHANNEL 18
#define DEFAULT_ATTENUATION 0x07 //0dBm
#define DEFAULT_ANTENNA 1 //ANT1 = SMA, ANT2 = CHIP
#define DEFAULT_XTAL_TRIM 0x08 //midscale
#define DEFAULT_XTAL_MODE 0xf0


HAL_GPIO_PIN(RF_ANT1, A, 9);
HAL_GPIO_PIN(RF_ANT2, A, 12);


#define PM_APBCMASK_RFCTRL_Pos      21

#define MMIO_REG(mem_addr, type) (*(volatile type *)(mem_addr))
#define RFCTRL_FECTRL MMIO_REG(0x42005400, uint16_t)


/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t
{
  APP_STATE_INITIAL,
  APP_STATE_IDLE,
} AppState_t;

typedef struct
{
	uint16_t channel;
	uint16_t txPwr;
	uint8_t antCtrl;
	uint8_t xtalTrim;
	
} tib_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

//*** NEW STUFF ***//
static void replySendData(void);
static void replyDataConf(NWK_DataReq_t *req);
static void initLamp(void);
static void lightLamp(void);
static void extinguishLamp(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;

//*** NEW STUFF ***//
static bool replyDataReqBusy = false;
static NWK_DataReq_t replyDataReq;
static char replyMessage[APP_BUFFER_SIZE];
static char newLetter;
static tib_t tib;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void phyWriteRegister(uint8_t reg, uint8_t value)
{
HAL_PhySpiSelect();
HAL_PhySpiWriteByteInline(RF_CMD_REG_W | reg);
HAL_PhySpiWriteByteInline(value);
HAL_PhySpiDeselect();
}

/*************************************************************************//**
*****************************************************************************/
static uint8_t phyReadRegister(uint8_t reg)
{
uint8_t value;

HAL_PhySpiSelect();
HAL_PhySpiWriteByteInline(RF_CMD_REG_R | reg);
value = HAL_PhySpiWriteByteInline(0);
HAL_PhySpiDeselect();

return value;
}

/*************************************************************************//**
*****************************************************************************/
static void phyWaitState(uint8_t state)
{
while (state != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_MASK));
}

/*************************************************************************//**
*****************************************************************************/
static void phyTrxSetState(uint8_t state)
{
phyWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
phyWaitState(TRX_STATUS_TRX_OFF);

phyWriteRegister(TRX_STATE_REG, state);
phyWaitState(state);
}

/*************************************************************************//**
*****************************************************************************/
void appSetChannel(uint8_t channel)
{
uint8_t reg;

reg = phyReadRegister(PHY_CC_CCA_REG) & ~0x1f;
phyWriteRegister(PHY_CC_CCA_REG, reg | channel);
}

/*****************************************************************************
*****************************************************************************/
static void appSetCwMode(void)
{
	
HAL_PhyReset();

phyTrxSetState(TRX_CMD_TRX_OFF);


if (tib.antCtrl == 1)
	phyWriteRegister(ANT_DIV_REG, (1 << ANT_CTRL) | (1 << ANT_EXT_SW_EN));
else
	phyWriteRegister(ANT_DIV_REG, (2 << ANT_CTRL) | (1 << ANT_EXT_SW_EN));

appSetChannel(tib.channel);
phyWriteRegister(PHY_TX_PWR_REG, tib.txPwr);

phyWriteRegister(XOSC_CTRL_REG, DEFAULT_XTAL_MODE | tib.xtalTrim );

phyWriteRegister(TRX_CTRL_0_REG, 1);
phyWriteRegister(TRX_CTRL_1_REG, 0);
phyWriteRegister(TST_CTRL_DIGI_REG, 0x0f);

HAL_PhySpiSelect();
HAL_PhySpiWriteByte(RF_CMD_FRAME_W);
HAL_PhySpiWriteByte(127);
for (uint8_t i = 0; i <= 127; i++)
{
	HAL_PhySpiWriteByte((uint8_t)rand());
}

HAL_PhySpiDeselect();

phyWriteRegister(PART_NUM_REG, 0x54);
phyWriteRegister(PART_NUM_REG, 0x46);

phyTrxSetState(TRX_CMD_PLL_ON);
phyWriteRegister(TRX_STATE_REG, TRX_CMD_TX_START);

while(1); // This keeps the CPU busy while the RF233 is in test mode.
}

/*****************************************************************************
*****************************************************************************/


static void lightLamp(void)
{
	HAL_LedOn(0);
}


static void extinguishLamp(void)
{
	HAL_LedOff(0);
}


uint8_t* _sbrk(int incr)
{
	static uint8_t heap[100];
	static uint8_t *heap_end;
	uint8_t *prev_heap_end;

	if (0 == heap_end)
	heap_end = heap;

	prev_heap_end = heap_end;

	if ((heap_end + incr) >= (heap + sizeof(heap)))
	return NULL;

	heap_end += incr;
	return prev_heap_end;
}


static void replySendData(void)
{
	if (replyDataReqBusy)
	return;
	
	replyDataReq.dstAddr = 1-APP_ADDR;
	replyDataReq.dstEndpoint = APP_ENDPOINT;
	replyDataReq.srcEndpoint = APP_ENDPOINT;
	replyDataReq.options = NWK_OPT_ACK_REQUEST;
	replyDataReq.data = replyMessage;
	replyDataReq.size = strlen(replyMessage);
	replyDataReq.confirm = replyDataConf;
	NWK_DataReq(&replyDataReq);

	replyDataReqBusy = true;
}

static void replyDataConf(NWK_DataReq_t *req)
{
	if (NWK_SUCCESS_STATUS == req->status)
	{
		replyDataReqBusy = false;
	}
	(void)req;
}


/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
  appDataReqBusy = false;
  (void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void)
{
  if (appDataReqBusy || 0 == appUartBufferPtr)
    return;

  memcpy(appDataReqBuffer, appUartBuffer, appUartBufferPtr);

  appDataReq.dstAddr = 1-APP_ADDR;
  appDataReq.dstEndpoint = APP_ENDPOINT;
  appDataReq.srcEndpoint = APP_ENDPOINT;
  appDataReq.options = NWK_OPT_ENABLE_SECURITY;
  appDataReq.data = appDataReqBuffer;
  appDataReq.size = appUartBufferPtr;
  appDataReq.confirm = appDataConf;
  NWK_DataReq(&appDataReq);

  appUartBufferPtr = 0;
  appDataReqBusy = true;
}

/*************************************************************************//**
*****************************************************************************/
void HAL_UartBytesReceived(uint16_t bytes)
{
  for (uint16_t i = 0; i < bytes; i++)
  {
    uint8_t byte = HAL_UartReadByte();

    if (appUartBufferPtr == sizeof(appUartBuffer))
      appSendData();

    if (appUartBufferPtr < sizeof(appUartBuffer))
      appUartBuffer[appUartBufferPtr++] = byte;
  }

  SYS_TimerStop(&appTimer);
  SYS_TimerStart(&appTimer);
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
  appSendData();
  (void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
 
 newLetter = ind->data[0];
 
 switch(newLetter)
 {
	 case 'T':
	 {
		lightLamp();
		appSetCwMode(); // Key up the transmitter. 		 
	 } break;
	  
	 case 's':
	 {
		 sprintf(replyMessage, "\r\n" "TIB Values:\r\n" "CHANNEL = %d\r\n" "TX_PWR = 0x%02X\r\n" "ANT_CTRL = %d\r\n" "XTAL_TRIM = 0x%02X\r\n", tib.channel, tib.txPwr, tib.antCtrl, tib.xtalTrim);
		 
	 } break;
	 
	 case 'C':
	 {
		if (tib.channel != 26) //Upper limit CH26 2480 MHz
			tib.channel++;

		sprintf(replyMessage, "\r\n" "CHANNEL = %d" "\r\n", tib.channel);
		 
	 } break;
	 
	 case 'c':
	 {
		if (tib.channel != 11) //Lower limit CH11 2405 MHz
			tib.channel--;

		sprintf(replyMessage, "\r\n" "CHANNEL = %d" "\r\n", tib.channel);
		 
	 } break;
	 
	 case 'f': //lower frequency = bigger XTAL_TRIM value
	 {
		 if(tib.xtalTrim != 0x0f) //Upper limit is 0x0f
			tib.xtalTrim++;
			
		sprintf(replyMessage, "\r\n" "XTAL_TRIM = 0x%02X\r\n", tib.xtalTrim);
	 } break;
	 
	 case 'F': //higher frequency = smaller XTAL_TRIM value
	 {
		 if(tib.xtalTrim != 0x00) //lower limit is 0x00
		   tib.xtalTrim--;
		   
		 sprintf(replyMessage, "\r\n" "XTAL_TRIM = 0x%02X\r\n", tib.xtalTrim);
	 } break;
	 
	 case 'P': //higher power = lower TX_PWR value
	 {
		if (tib.txPwr != 0x00)
			tib.txPwr--;

		sprintf(replyMessage, "\r\n" "TX_PWR = 0x%02X" "\r\n", tib.txPwr);

	 }  break;
	 
	 case 'p': //lower power = higher TX_PWR value
	 {
		if (tib.txPwr != 0x0f)
			tib.txPwr++;
			
		sprintf(replyMessage, "\r\n" "TX_PWR = 0x%02X" "\r\n", tib.txPwr);

	 }  break;
	 
	 case 'a':
	 {
		 if (tib.antCtrl == 1)
		   tib.antCtrl = 2;

		 sprintf(replyMessage, "\r\n" "ANT_CTRL = %d" "\r\n", tib.antCtrl);
	 } break;
	 
	 default:
	 {
		 sprintf(replyMessage, "\r\n" TOP_MENU "\r\n");
	 } break;
 }
  replySendData();
  
  return true;
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
  NWK_SetAddr(APP_ADDR);
  NWK_SetPanId(APP_PANID);
  PHY_SetChannel(APP_CHANNEL);
#ifdef PHY_AT86RF212
  PHY_SetBand(APP_BAND);
  PHY_SetModulation(APP_MODULATION);
#endif
  PHY_SetRxState(true);

  NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

  HAL_BoardInit();

  appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
  appTimer.mode = SYS_TIMER_INTERVAL_MODE;
  appTimer.handler = appTimerHandler;
  
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
  switch (appState)
  {
    case APP_STATE_INITIAL:
    {
		appInit();
	    HAL_LedInit();
				
		HAL_GPIO_RF_ANT1_out();
		HAL_GPIO_RF_ANT1_pmuxen();

		HAL_GPIO_RF_ANT2_out();
		HAL_GPIO_RF_ANT2_pmuxen();
		
		PORT->Group[HAL_GPIO_PORTA].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_F_Val;
		PORT->Group[HAL_GPIO_PORTA].PMUX[6].bit.PMUXE = PORT_PMUX_PMUXE_F_Val;

		PM->APBCMASK.reg |= (1<<PM_APBCMASK_RFCTRL_Pos);

		// change amount of shift
		RFCTRL_FECTRL = (0 << 4/*DIG1*/) | (1 << 2/*DIG2*/);
		
		phyWriteRegister(ANT_DIV_REG, (1 << ANT_CTRL) | (1 << ANT_EXT_SW_EN));
		
		extinguishLamp();
		
		tib.antCtrl = DEFAULT_ANTENNA;
	    tib.channel = DEFAULT_CHANNEL;
	    tib.txPwr = DEFAULT_ATTENUATION;
		tib.xtalTrim = DEFAULT_XTAL_TRIM;
	  
	    sprintf(replyMessage, "\r\n" SPLASH_STRING " " VERSION_ID "\r\n");
	   
	    replySendData();
		
	   
      appState = APP_STATE_IDLE;
    } break;
	

    case APP_STATE_IDLE:
      break;

    default:
      break;
  }
}

/*************************************************************************//**
*****************************************************************************/
  int testMain = 0;

int main(void)
{
  SYS_Init();
  HAL_UartInit(38400);
  HAL_LedInit();
  while (1)
  {
  	HAL_LedOn(0);
  	for (int i = 0; i < 100; i++) {
	  	HAL_TimerDelay(10000);
  	}
  	HAL_LedOff(0);
  	for (int i = 0; i < 100; i++) {
	  	HAL_TimerDelay(10000);
  	}
  	testMain++;
    // SYS_TaskHandler();
    // HAL_UartTaskHandler();
    // APP_TaskHandler();
  }
}
