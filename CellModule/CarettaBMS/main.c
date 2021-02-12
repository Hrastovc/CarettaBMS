/****************************************************************************************
 *   _____                         _     _             ____    __  __    _____ 
 *  / ____|                       | |   | |           |  _ \  |  \/  |  / ____|
 * | |        __ _   _ __    ___  | |_  | |_    __ _  | |_) | | \  / | | (___  
 * | |       / _` | | '__|  / _ \ | __| | __|  / _` | |  _ <  | |\/| |  \___ \ 
 * | |____  | (_| | | |    |  __/ | |_  | |_  | (_| | | |_) | | |  | |  ____) |
 *  \_____|  \__,_| |_|     \___|  \__|  \__|  \__,_| |____/  |_|  |_| |_____/ 
 *
 ***************************************************************************************/

/************************************************************************************//**
* \file       main.c
* \brief      CarettaBMS, Open source BMS
* \internal
*----------------------------------------------------------------------------------------
*                                    L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of CarettaBMS. CarettaBMS is free software: you can redistribute it
* and/or modify it under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation.
*
* CarettaBMS is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with CarettaBMS. It
* should be located in ".\Doc\license.md". If not, check on gitHub repository to obtain
* a copy.
*
* \endinternal
****************************************************************************************/


/****************************************************************************************
* Include files
****************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "CarettaBMS_CMD.h"
#include "command.h"
#include "FIFO.h"


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define F_CPU (3333333333)

#define RX_BUFFER_SIZE 16
#define TX_BUFFER_SIZE 16

#define ALARM_BUFFER_SIZE 8

#define readUserRow(x) (*x)

#define UR_BOOTLOADER_CONDITION_ADDR ((volatile uint8_t  *)(&USERROW_USERROW0 + 31))
#define UR_MODULE_NUMBER_ADDR        ((volatile uint8_t  *)(&USERROW_USERROW0 + 30))

#define RESP_OK                      (uint8_t)(0x0F)

#define ERROR_UNKNOWN_COMMAND        (uint8_t)(0xEA)
#define ERROR_SEQUENCE               (uint8_t)(0xEA)

#define ALARM_OVER_TEMP              (uint8_t)(0xA0)
#define ALARM_CPU_OUTATIME           (uint8_t)(0xA1)
#define ALARM_RTC_OVERFLOW           (uint8_t)(0xA2)
#define ALARM_OVERVOLTAGE            (uint8_t)(0xA3)
#define ALARM_CELL_CHARGED           (uint8_t)(0xA4)
#define ALARM_CELL_EMPTY             (uint8_t)(0xA5)
#define ALARM_UNDERVOLTAGE           (uint8_t)(0xA6)
#define ALARM_FRAME_PARITY           (uint8_t)(0xA7)

#define USE_I2C


/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef struct
{
  /* Parameters */
  volatile uint16_t vrefCalibrated;
  volatile uint16_t balanceCurrent;
  volatile uint16_t maxTempBMS;
  volatile uint16_t maxTempBMSRecover;
  volatile uint16_t voltLimitHigh;
  volatile uint16_t voltLimitHighRecover;
  volatile uint16_t voltageCharged;
  volatile uint16_t voltageChargedRecover;
  volatile uint16_t voltageEmpty;
  volatile uint16_t voltageEmptyRecover;
  volatile uint16_t voltLimitLow;
  volatile uint16_t voltLimitLowRecover;
  volatile uint16_t measureInterval;
  volatile uint16_t voltBalanceStart;
  volatile uint16_t voltBalanceStop;
  volatile uint16_t balanceHysteresis;
  volatile uint8_t sleepRequested;
  /* program global variables */
  volatile uint8_t moduleNum;
  volatile uint8_t topModule;
  volatile uint16_t targetVoltage;
  volatile uint32_t cellVoltage;
  volatile uint16_t intTemp;
  volatile uint16_t ADCextInput[2];
}tSettings;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
//void task(void);
void syncResponse(void);
void setBalancingCurrent(uint8_t current);
void watchdogConfig(uint8_t x);
void goToBootloader(void);
void goToSleep(uint8_t mode);
void calcVoltageReading(void);
uint16_t takeADCreading(void);
uint8_t readWait(void);
void respondIfBot(uint8_t ch);
void respondIfNotBot(uint8_t ch);
void respond(uint8_t data);
void respondTwoBytes(uint16_t data);
void forwardBufferIfNotBot(void);
void forward(uint8_t data);
uint8_t readSave(void);
void resetProcessor(void);
void twoByteParameter(volatile uint16_t *parameter);
void writeUserRow1byte(volatile uint8_t *addr, uint8_t val);
void emptyAlarmQ(void);
void createAlarm(uint8_t code);
void respondOK(uint8_t respSize);
void respondOKnoPayload(void);
void emptyOutgoingBuffer(void);
void setMeasureInterval(uint16_t interval);
void task(tSettings *p);
void ADCset1V5refMuxIntRef(void);
void ADCtriggers(uint8_t state);


/****************************************************************************************
* Local data declaration
****************************************************************************************/
FIFO_t RXfifo;
uint8_t RXbuffer[RX_BUFFER_SIZE];

FIFO_t TXfifo;
uint8_t TXbuffer[TX_BUFFER_SIZE];

FIFO_t alarmQ;
uint8_t alarmBuffer[ALARM_BUFFER_SIZE];

tSettings EEMEM defaultSettings = {
  1500,  //vrefCalibrated
    10,  //balanceCurrent
    60,  //maxTempBMS
     5,  //maxTempBMSRecover
  4300,  //voltLimitHigh
  4200,  //voltLimitHighRecover
  4200,  //voltageCharged
  4150,  //voltageChargedRecover
  3200,  //voltageEmpty
  3300,  //voltageEmptyRecover
  2500,  //voltLimitLow
  2600,  //voltLimitLowRecover
  1000,  //measureInterval
  3600,  //voltBalanceStart
  4300,  //voltBalanceStop
    50,  //balanceHysteresis
     0,  //sleepRequested
     0,  //moduleNum
     0,  //topModule
  5000,  //targetVoltage
     0,  //cellVoltage
     0,  //intTemp
 {0, 0}  //ADCextInput[2]
};

tSettings EEMEM storedSettings = {
  1500,  //vrefCalibrated
    10,  //balanceCurrent
    60,  //maxTempBMS
     5,  //maxTempBMSRecover
  4300,  //voltLimitHigh
  4200,  //voltLimitHighRecover
  4200,  //voltageCharged
  4150,  //voltageChargedRecover
  3200,  //voltageEmpty
  3300,  //voltageEmptyRecover
  2500,  //voltLimitLow
  2600,  //voltLimitLowRecover
  1000,  //measureInterval
  3600,  //voltBalanceStart
  4300,  //voltBalanceStop
    50,  //balanceHysteresis
     0,  //sleepRequested
     0,  //moduleNum
     0,  //topModule
  5000,  //targetVoltage
     0,  //cellVoltage
     0,  //intTemp
 {0, 0}  //ADCextInput[2]
};

tSettings settings;


/************************************************************************************//**
** \brief     This is main program entry point
**
****************************************************************************************/
int main(void)
{
  cli();
  
  /* CPUINT init */
  CPUINT_LVL1VEC = AC0_AC_vect_num;
  
  /* PORT init */
  /* PA3 -> PWM, PA6 -> TxD */
  PORTA_DIRSET = PIN3_bm | PIN6_bm;
  PORTA_PIN6CTRL = PORT_INVEN_bm;
  
  /* VREF init */
  /* Moved to ADCset1V5refMuxIntRef() */
  //VREF_CTRLA = VREF_ADC0REFSEL_1V5_gc | VREF_DAC0REFSEL_4V34_gc;
  
  /* ADC init */
  ADCset1V5refMuxIntRef();
  ADCtriggers(1);
  ADC0_CTRLB = 0x04; //ACC16
  ADC0_CTRLD = ADC_INITDLY_DLY32_gc | ADC_ASDV_ASVON_gc | 0x02;
  ADC0_CTRLA = ADC_ENABLE_bm;
  
  /* AC init */
  AC0_MUXCTRLA = AC_MUXNEG_VREF_gc | AC_INVERT_bm;
  AC0_CTRLA = AC_RUNSTDBY_bm | AC_HYSMODE_50mV_gc | AC_ENABLE_bm;
  AC0_INTCTRL = AC_CMP_bm;
  
  /* USART init */
  USART0_CTRLA = USART_RXCIE_bm;
  USART0_CTRLB = USART_RXEN_bm | USART_TXEN_bm;
  USART0_CTRLC |= USART_PMODE_ODD_gc | USART_CHSIZE_9BITL_gc;
  USART0_BAUD = 1389;
  
  /* TCA init */
  TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
  TCA0_SINGLE_PER = 102;
  TCA0_SINGLE_CMP0 = 0;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm;
  
  /* EVSYS init */
  EVSYS_SYNCCH0 = EVSYS_SYNCCH0_TCA0_CMP0_gc;
  EVSYS_ASYNCUSER1 = EVSYS_ASYNCUSER1_SYNCCH0_gc;
  
  /* Load settings */
  eeprom_read_block(&settings, &storedSettings, sizeof(tSettings));
  
  /* Read user row */
  settings.moduleNum = readUserRow(UR_MODULE_NUMBER_ADDR);
  
  /* Init. global variables. */
  settings.targetVoltage = 5000;
  
  /* RTC init */
  RTC_CLKSEL = RTC_CLKSEL_INT1K_gc;
  RTC_INTCTRL = RTC_OVF_bm;
  RTC_PITINTCTRL = RTC_PI_bm;
  while(RTC_PITSTATUS & RTC_CTRLBUSY_bm);
  RTC_PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
  /* 25ms in required for the ADC to take 2 measurement, before running task for the first time. */
  setMeasureInterval(settings.measureInterval);
  while(RTC_STATUS & RTC_CTRLABUSY_bm);
  RTC_CTRLA = RTC_RUNSTDBY_bm | RTC_RTCEN_bm;
  
  /* FIFO init */
  FIFOinit(&RXfifo, RXbuffer, RX_BUFFER_SIZE);
  FIFOinit(&TXfifo, TXbuffer, TX_BUFFER_SIZE);
  FIFOinit(&alarmQ, alarmBuffer, ALARM_BUFFER_SIZE);
  
  /* Update bootloader flag */
  writeUserRow1byte(UR_BOOTLOADER_CONDITION_ADDR, 0xEA);
  
  /* WDT init */
  watchdogConfig(WDT_PERIOD_4KCLK_gc);
  
  sei();
  
  for(;;) 
  {
    uint8_t ch = readSave();
    
    /* Reads all the alarm codes from one of the module, TODO: handle alarm codes */
    if(ch == CMD_ALARM_CODES)
    {
      /* read alarm source module number */
      uint8_t sourceModule = readSave();
      /* read the number of alarm codes send */
      uint8_t len = readSave();
      /* read all the codes */
      do readSave();
      while(--len);
      /* Forward alarm,
       * except if the circle was completed and you were the source of the alarm.
       */
      if(sourceModule == settings.moduleNum)
      {
        emptyOutgoingBuffer();
      }
      else
      {
        do 
        {
          forward(FIFOread(&TXfifo));
        }
        while(FIFOavailable(&TXfifo));
      }
    }
    
    /* Command to go to the bootloader */
    else if(ch == CMD_GOTO_BOOTLOADER)
    {
      writeUserRow1byte(UR_BOOTLOADER_CONDITION_ADDR, 0xEB);
      if(!settings.moduleNum) respond(0xEB);
      forwardBufferIfNotBot();
      resetProcessor();
    }
    
    /* New voltage reference calibration value */
    else if(ch == CMD_VOLTAGE_CAL)
    {
      if(settings.moduleNum == readSave())
      {
        twoByteParameter(&settings.vrefCalibrated);
        emptyOutgoingBuffer();
      }
      else
      {
        readSave();
        readSave();
      }
    }
    
    /* New balance current limit value */
    else if(ch == CMD_BALANCE_CURRENT)
    {
      twoByteParameter(&settings.balanceCurrent);
    }
    
    /* Respond with all the external temperatures. */
    else if(ch == CMD_EXT_INPUTS)
    {
      respondOK(0x04);
      for(uint8_t i; i < 2; i++)
      {
        respond((uint8_t)settings.ADCextInput[i]);
        respond((uint8_t)(settings.ADCextInput[i] >> 8));
      }
    }
    
    /*  */
    else if(ch == CMD_MAX_TEMP_BMS)
    {
      twoByteParameter(&settings.maxTempBMS);
    }
    
    /*  */
    else if(ch == CMD_MAX_TEMP_RECOVER)
    {
      twoByteParameter(&settings.maxTempBMSRecover);
    }
    
    /*  */
    else if(ch == CMD_HIGH_VOLT_LIMIT)
    {
      twoByteParameter(&settings.voltLimitHigh);
    }
    
    /*  */
    else if(ch == CMD_HIGH_VOLT_RECOVER)
    {
      twoByteParameter(&settings.voltLimitHighRecover);
    }
    
    /*  */
    else if(ch == CMD_CHARGED_VOLTAGE)
    {
      twoByteParameter(&settings.voltageCharged);
    }
    
    /*  */
    else if(ch == CMD_CHARGED_RECOVER)
    {
      twoByteParameter(&settings.voltageChargedRecover);
    }
    
    /*  */
    else if(ch == CMD_EMPTY_VOLTAGE)
    {
      twoByteParameter(&settings.voltageEmpty);
    }
    
    /*  */
    else if(ch == CMD_EMPTY_RECOVER)
    {
      twoByteParameter(&settings.voltageEmptyRecover);
    }
    
    /*  */
    else if(ch == CMD_LOW_VOLT_LIMIT)
    {
      twoByteParameter(&settings.voltLimitLow);
    }
    
    /*  */
    else if(ch == CMD_LOW_VOLT_RECOVER)
    {
      twoByteParameter(&settings.voltLimitLowRecover);
    }
    
    /*  */
    else if(ch == CMD_GOTO_SLEEP)
    {
      settings.sleepRequested = 0x01;
      respondOKnoPayload();
    }
    
    /*  */
    else if(ch == CMD_TOP_MODULE)
    {
      settings.topModule = 0x01;
      respondOKnoPayload();
      emptyOutgoingBuffer();
      forward(CMD_NOT_TOP_MODULE);
    }
    
    /*  */
    else if(ch == CMD_NOT_TOP_MODULE)
    {
      settings.topModule = 0x00;
      respondOKnoPayload();
    }
    
    /*  */
    else if(ch == CMD_MEASURE_INTERVAL)
    {
      twoByteParameter(&settings.measureInterval);
      setMeasureInterval(settings.measureInterval);
    }
    
    /* Set the new module number */
    else if(ch == CMD_NMBR_OF_MODULES)
    {
      settings.moduleNum = readWait() - 1;
      writeUserRow1byte(UR_MODULE_NUMBER_ADDR, settings.moduleNum);
      FIFOaddToBuffer(&TXfifo, settings.moduleNum);
      respondOKnoPayload();
    }
    
    /*  */
    else if(ch == CMD_BALANCE_START)
    {
      twoByteParameter(&settings.voltBalanceStart);
    }
    
    /*  */
    else if(ch == CMD_BALANCE_STOP)
    {
      twoByteParameter(&settings.voltBalanceStop);
    }
    
    /*  */
    else if(ch == CMD_BALANCE_HYSTERESIS)
    {
      twoByteParameter(&settings.balanceHysteresis);
    }
    
    /*  */
    else if(ch == CMD_RESET_SETTINGS)
    {
      eeprom_read_block(&settings, &defaultSettings, sizeof(tSettings));
      settings.moduleNum = readUserRow(UR_MODULE_NUMBER_ADDR);
      eeprom_write_block(&settings, &storedSettings, sizeof(tSettings));
      setMeasureInterval(settings.measureInterval);
      respondOKnoPayload();
    }
    
    /* Take module to sleep */
    else if(ch == CMD_SAVE_SETTINGS)
    {
      eeprom_write_block(&settings, &storedSettings, sizeof(tSettings));
      respondOKnoPayload();
    }
    
    /* Respond with temperature reading of processors internal sensor. */
    else if(ch == CMD_INT_TEMPERATURE)
    {
      //respondOK(0x02);
      respondTwoBytes(settings.intTemp);
    }
    
    /*  */
    else if(ch == CMD_TARGET_VOLTAGE)
    {
      twoByteParameter(&settings.targetVoltage); 
    }
    
    /* Respond with the cell voltage */
    else if(ch == CMD_CELL_VOLTAGE)
    {
      while(!settings.cellVoltage);
      //respondOK(0x02);
      respondTwoBytes(settings.cellVoltage);
    }
    
    /* Do nothing, just sync for all the modules to wake up. */
    else if(ch == CMD_WAKE_UP_SYNC)
    {
      settings.sleepRequested = 0x00;
      respondOKnoPayload();
    }
    
#ifdef USE_I2C
    
    /*  */
    else if(ch == CMD_TWI_INIT)
    {
      if(settings.moduleNum == readSave())
      {
        respondOKnoPayload();
        /* 100kHz @ 3.33MHz per. clk. */
        TWI0_MBAUD = 10;
        /* Enable TWI as master without any interrupts. */
        TWI0_MCTRLA = TWI_ENABLE_bm;
        /* Writing 0x1 to the BUSSTATE bits forces the bus state logic into its Idle state. */
        TWI0_MSTATUS = TWI_BUSSTATE_IDLE_gc;
        emptyOutgoingBuffer();
      }
    }
    
    /*  */
    else if(ch == CMD_TWI_STATUS)
    {
      if(settings.moduleNum == readSave())
      {
        respondOK(0x01);
        respond(TWI0_MSTATUS);
        emptyOutgoingBuffer();
      }
    }
    
    /*  */
    else if(ch == CMD_TWI_START)
    {
      if(settings.moduleNum == readSave())
      {
        uint8_t address = readSave();
        uint8_t rdWr = readSave();
        if(rdWr > 0x01) respond(ERROR_SEQUENCE);
        else
        {
          respondOK(0x01);
          /* Send START condition */
          TWI0_MADDR = (uint8_t)(address << 1) | (rdWr & 0x01);
          /* Wait for write or read interrupt flag */
          while(!(TWI0_MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));
          /* Error on bus, return 0x00. */
          if(TWI0_MSTATUS & TWI_ARBLOST_bm) respond(0x00);
          /* Return true if slave gave an ACK */
          respond(!(TWI0_MSTATUS & TWI_RXACK_bm));
        }
        emptyOutgoingBuffer();
      }
      else
      {
        readSave();
        readSave();
      }      
    }
    
    /*  */
    else if(ch == CMD_TWI_STOP)
    {
      if(settings.moduleNum == readSave())
      {
        respondOKnoPayload();
        /* Send STOP */
        TWI0_MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
        emptyOutgoingBuffer();
      }
    }
    
    /*  */
    else if(ch == CMD_TWI_WRITE)
    {      
      if(settings.moduleNum == readSave())
      {
        uint8_t data = readSave();
        respondOK(0x01);
        /* Wait for write complete interrupt flag */
        while(!(TWI0_MSTATUS & TWI_WIF_bm));
        /* Copy data to master data register. */
        TWI0_MDATA = data;
        /* Transmit Data */
        TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
        /* Return true if slave gave an ACK */
        respond(!(TWI0_MSTATUS & TWI_RXACK_bm));
        emptyOutgoingBuffer();
      }
      else
      {
        readSave();
      }
    }
    
    /*  */
    else if(ch == CMD_TWI_READ)
    {
      if(settings.moduleNum == readSave())
      {
        uint8_t nack = readSave();
        if(nack > 0x01) respond(ERROR_SEQUENCE);
        else
        {
          respondOK(0x01);
          /* Wait for read complete interrupt flag */
          while(!(TWI0_MSTATUS & TWI_RIF_bm));
          /* Copy data from master data register. */
          uint8_t data = TWI0_MDATA;
          /* Last byte to be read, send NACK. */
          //if(!ack) TWI0_MCTRLB = TWI_ACKACT_bm | TWI_MCMD_RECVTRANS_gc;
          /* Execute Acknowledge Action succeeded by a byte read operation. */
          //else TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
          TWI0_MCTRLB = (nack << TWI_ACKACT_bp) | TWI_MCMD_RECVTRANS_gc;
          /* Return received data byte */
          respond(data);
        }        
        emptyOutgoingBuffer();
      }
      else
      {
        readSave();
      }
    }
    
#endif // USE_I2C
    
    /* Unknown command, respond with error code. */
    else
    {
      respond(ERROR_UNKNOWN_COMMAND);
      emptyOutgoingBuffer();
    }
    
    forwardBufferIfNotBot();
  }
} /*** end of main ***/


void respondOK(uint8_t respSize)
{
  respond(RESP_OK);
  respond(respSize);
}


void respondOKnoPayload(void)
{
  respond(RESP_OK);
  respond(0x00);
}


void emptyOutgoingBuffer(void)
{
  FIFOflush(&TXfifo);
} 


void setMeasureInterval(uint16_t interval)
{
  /* Reset counter in case new interval is shorter than the last one. */
  RTC_CNT = 0x0000;
  RTC_CMP = interval;
  /* Interval should be limited to 60.000 (1min), to ensure RTC_PER doesn't overflows. */
  RTC_PER = interval + 100;
}


void createAlarm(uint8_t code)
{
  FIFOaddToBuffer(&alarmQ, code);
}


void emptyAlarmQ(void)
{
  if(FIFOavailable(&alarmQ))
  {
    forward(CMD_ALARM_CODES);
    forward(settings.moduleNum);
    forward(FIFOavailable(&alarmQ));
    do 
    {
      forward(FIFOread(&alarmQ));
    }
    while(FIFOavailable(&alarmQ));
  }
}


void ADCset1V5refMuxIntRef(void)
{
  VREF_CTRLA = VREF_ADC0REFSEL_1V5_gc | VREF_DAC0REFSEL_4V34_gc;
  ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV2_gc;
  ADC0_MUXPOS = ADC_MUXPOS_INTREF_gc;
}


void ADCtriggers(uint8_t state)
{
  if(!state)
  {
    /* Disable triggers. */
    ADC0_INTCTRL = 0x00;
    ADC0_EVCTRL = 0x00;
  }
  else
  {
    /* Enable triggers. */
    ADC0_EVCTRL = ADC_STARTEI_bm;
    ADC0_INTCTRL = ADC_RESRDY_bm;
  }
}


/************************************************************************************//**
** \brief     Periodic task
**
****************************************************************************************/
void task(tSettings *p)
{
  static uint8_t overvoltageCondition = 0x00;
  static uint8_t chargedCondition = 0x00;
  static uint8_t emptyCondition = 0x00;
  static uint8_t undervoltageCondition = 0x00;
  static uint8_t overTempCondition = 0x00;
  static uint8_t balancingCondition = 0x00;
  
  /* Disable triggers. */
  ADCtriggers(0);
  
  /* Sample two external inputs. */
  /* Select external input number 1. */
  ADC0_MUXPOS = ADC_MUXPOS_AIN1_gc;
  /* wait for things to settle down */
  takeADCreading();
  /* Take a reading */
  p->ADCextInput[0] =  ((uint32_t)(takeADCreading() >> 2) * p->cellVoltage) / 4096;
  
  /* Select external input number 2. */
  ADC0_MUXPOS = ADC_MUXPOS_AIN2_gc;
  /* wait for things to settle down */
  takeADCreading();
  /* Take a reading */
  p->ADCextInput[1] =  ((uint32_t)(takeADCreading() >> 2) * p->cellVoltage) / 4096;
  
  /* Sample internal temperature sensor. */
  /* Switch to 1.1V reference */
  VREF_CTRLA = VREF_ADC0REFSEL_1V1_gc | VREF_DAC0REFSEL_4V34_gc;
  /* Switch to internal reference. */
  ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV2_gc;
  /* Select internal temperature sensor input. */
  ADC0_MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
  /* wait for things to settle down */
  takeADCreading();
  /* ADC conversion result with 1.1 V internal reference */
  /* SIGROW_TEMPSENSE1 = sigrow_offset */
  uint32_t temp = (takeADCreading() >> 4) - SIGROW_TEMPSENSE1;
  /* Result might overflow 16 bit variable (10bit+8bit) */
  /* SIGROW_TEMPSENSE0 = sigrow_gain */
  temp *= SIGROW_TEMPSENSE0;
  /* Add 1/2 to get correct rounding on division below */
  temp += 0x80;
  /* Divide result to get Kelvin */
  temp >>= 8;
  /* Copy result, temperature is in kelvins! */
  p->intTemp = temp;
  
  /* ADC init */
  ADCset1V5refMuxIntRef();
  takeADCreading();
  /* Enable triggers. */
  ADCtriggers(1);
  
  
  if((p->cellVoltage > p->voltLimitHigh) || overvoltageCondition)
  {
    if(!overvoltageCondition) createAlarm(ALARM_OVERVOLTAGE);
    overvoltageCondition = (p->cellVoltage > p->voltLimitHighRecover);
  }
  
  if((p->cellVoltage > p->voltageCharged) || chargedCondition)
  {
    if(!chargedCondition) createAlarm(ALARM_CELL_CHARGED);
    chargedCondition = (p->cellVoltage > p->voltageChargedRecover);
  }
  
  if((p->cellVoltage < p->voltageEmpty) || emptyCondition)
  {
    if(!emptyCondition) createAlarm(ALARM_CELL_EMPTY);
    emptyCondition = (p->cellVoltage < p->voltageEmptyRecover);
  }
  
  if((p->cellVoltage < p->voltLimitLow) || undervoltageCondition)
  {
    if(!undervoltageCondition) createAlarm(ALARM_UNDERVOLTAGE);
    undervoltageCondition = (p->cellVoltage < p->voltLimitLowRecover);
  }
  
  if((p->intTemp > p->maxTempBMS) || overTempCondition)
  {
    if(!overTempCondition) createAlarm(ALARM_OVER_TEMP);
    overTempCondition = (p->intTemp > p->maxTempBMSRecover);
  }
  
  if((p->cellVoltage > p->voltBalanceStart) && (p->cellVoltage < p->voltBalanceStop))
  {
    if((p->cellVoltage > (p->targetVoltage + p->balanceHysteresis)) || balancingCondition)
    { 
      balancingCondition = (p->cellVoltage > p->targetVoltage);
      
      if(!overTempCondition && !p->sleepRequested)
      //if(!p->sleepRequested)
      {
        TCA0_SINGLE_CMP0 = p->balanceCurrent;
      }
      else
      {
        TCA0_SINGLE_CMP0 = 0x00;
      }
    }
    else
    {
      balancingCondition = 0x00;
      TCA0_SINGLE_CMP0 = 0x00;
    }
  }
  else
  {
    balancingCondition = 0x00;
    TCA0_SINGLE_CMP0 = 0x00;
  }
  
  /* Send all the alarms and hope new module is not waiting for parameters. */
  emptyAlarmQ();
  
  
  if(undervoltageCondition)
  {
    goToSleep(SLEEP_MODE_PWR_DOWN);
  }
  else if(emptyCondition || p->sleepRequested)
  {
    goToSleep(SLEEP_MODE_STANDBY);
    /* This is in case CMD_CELL_VOLTAGE woke-up the processor.
     * Response wont be given until cell voltage is measured.
     */
    p->cellVoltage = 0x00;
  }
  
  asm("wdr");
    
} /*** end of task ***/


/************************************************************************************//**
** \brief     Take ADC reading
** \return    CRCSCAN_STATUS CRC check status
**
****************************************************************************************/
uint16_t takeADCreading(void)
{
  /* Start conversion. */
  ADC0_COMMAND = ADC_STCONV_bm;
  /* Wait for it to finish. */
  while(!(ADC0_INTFLAGS & ADC_RESRDY_bm));
  /* Return reading. */
  return ADC0_RES;
} /*** end of takeADCreading ***/


/************************************************************************************//**
** \brief     Read and save 2 byte parameter
** \param     parameter Pointer to the local parameter variable
**
****************************************************************************************/
void twoByteParameter(volatile uint16_t *parameter)
{
  uint8_t rdWr = readSave();
  if(rdWr == 'w')
  {
    *parameter = readSave();
    *parameter |= readSave() << 8;
    respondOKnoPayload();
  }
  else if(rdWr == 'r')
  {
    //respondOK(0x02);
    respondTwoBytes(*parameter);
  }
  else
  {
    emptyOutgoingBuffer();
    respond(ERROR_SEQUENCE);
  }
} /*** end of twoByteParameter ***/


/************************************************************************************//**
** \brief     Read and save byte
** \return    return read byte
**
****************************************************************************************/
uint8_t readSave(void)
{
  uint8_t ch = readWait();
  FIFOaddToBuffer(&TXfifo, ch);
  return ch;
} /*** end of readSave ***/


/************************************************************************************//**
** \brief     Wait for an incoming byte
** \return    return read byte
**
****************************************************************************************/
uint8_t readWait(void)
{
  do 
  {
    if((RTC_INTFLAGS & RTC_CMP_bm))
    {
      /* Reset counter and set interval */
      setMeasureInterval(settings.measureInterval);
      /* Clear the int. flag. */
      RTC_INTFLAGS = RTC_CMP_bm;
      /* Do the work */
      task(&settings);
    }    
  }
  while(!FIFOavailable(&RXfifo));
  /* Reset counter and set interval */
  setMeasureInterval(settings.measureInterval);
  /* Now return the received byte. */
  return FIFOread(&RXfifo);
} /*** end of readSave ***/


/************************************************************************************//**
** \brief     Put processor to sleep
**
****************************************************************************************/
void goToSleep(uint8_t mode)
{
  setMeasureInterval(25);
  while(!(USART0_STATUS & USART_TXCIF_bm));
  /* go to sleep */
  PORTA_PIN3CTRL = 0x01;
  SLPCTRL_CTRLA = mode | SLPCTRL_SEN_bm;
  AC0_CTRLA |= AC_OUTEN_bm;
  asm("sleep");
  /* Clear the int. flag. */
  RTC_INTFLAGS = RTC_CMP_bm;
} /*** end of goToSleep ***/


/************************************************************************************//**
** \brief     Only forward buffer if it's not the bottom module
**
****************************************************************************************/
void forwardBufferIfNotBot(void)
{
  if(!settings.moduleNum)
  {
    emptyOutgoingBuffer();
  }
  else
  {
    while(FIFOavailable(&TXfifo))
    {
      forward(FIFOread(&TXfifo));
    }
  }
} /*** end of forwardBufferIfNotBot ***/


/************************************************************************************//**
** \brief     Write 1 byte to user row
** \param     addr Pointer to the user row position
** \param     val Value to save
**
****************************************************************************************/
void writeUserRow1byte(volatile uint8_t *addr, uint8_t val)
{
  *addr = val;
  _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
  while(NVMCTRL_STATUS & NVMCTRL_EEBUSY_bm);
} /*** end of writeUserRow1byte ***/


/************************************************************************************//**
** \brief     Configure watchdog timer
** \param     x Watchdog control register A setting
**
****************************************************************************************/
void watchdogConfig(uint8_t x)
{
  while(WDT_STATUS & WDT_SYNCBUSY_bm);
  _PROTECTED_WRITE(WDT_CTRLA, x);
} /*** end of watchdogConfig ***/


/************************************************************************************//**
** \brief     Reset processor for sequence error
**
****************************************************************************************/
void resetProcessor(void)
{
  while(!(USART0_STATUS & USART_TXCIF_bm));
  watchdogConfig(WDT_PERIOD_8CLK_gc);
  for(;;);
} /*** end of resetProcessor ***/


/************************************************************************************//**
** \brief     Respond with single byte
** \param     data Byte to respond with
**
****************************************************************************************/
void respond(uint8_t data)
{
  volatile uint8_t parityBit;
  
  if(!settings.moduleNum)
  {
    //parityBit = __builtin_parity(data);
    parityBit = data;
    parityBit ^= parityBit >> 4;
    parityBit ^= parityBit >> 2;
    parityBit ^= parityBit >> 1;
  }
  
  while(!(USART0_STATUS & USART_DREIF_bm));
  
  if(!settings.moduleNum) USART0_TXDATAH = parityBit;
  else USART0_TXDATAH = 0x00;
  
  USART0_TXDATAL = data;
} /*** end of respond ***/


/************************************************************************************//**
** \brief     Respond with two bytes
** \param     data Data to respond with
**
****************************************************************************************/
void respondTwoBytes(uint16_t data)
{
  respondOK(0x02);
  respond((uint8_t)data);
  respond((uint8_t)(data >> 8));
} /*** end of respond ***/


/************************************************************************************//**
** \brief     Forward single byte
** \param     data Byte to forward
**
****************************************************************************************/
void forward(uint8_t data)
{
  while(!(USART0_STATUS & USART_DREIF_bm));
  USART0_TXDATAH = 0x01;
  USART0_TXDATAL = data;
} /*** end of forward ***/


/************************************************************************************//**
** \brief     Interrupt service routine for Analog Comparator
**
****************************************************************************************/
ISR(AC0_AC_vect)
{
  /* Clear int. flag */
  AC0_STATUS = AC_CMP_bm;
  /* Invert pin input */
  if(!(AC0_STATUS & AC_STATE_bm)) PORTA_PIN7CTRL = PORT_INVEN_bm;
  else PORTA_PIN7CTRL = 0x00;
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for UART receive
**
****************************************************************************************/
ISR(USART0_RXC_vect)
{
  volatile uint8_t status = USART0_RXDATAH;
  volatile uint8_t data   = USART0_RXDATAL;
  
  if(status & (USART_FERR_bm | USART_PERR_bm))
  {
    createAlarm(ALARM_FRAME_PARITY);
    /* Send all the alarms and hope new module is not waiting for parameters. */
    emptyAlarmQ();
    /* Something went wrong, reset itself. */
    resetProcessor();
  }
  else if(status & USART_RXCIF_bm)
  {
    if((status & 0x01) | settings.topModule)
    {
      FIFOaddToBuffer(&RXfifo, data);
    }
    else
    {
      respond(data);
    }
  }
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for ADC result ready
**
****************************************************************************************/
ISR(ADC0_RESRDY_vect)
{
  settings.cellVoltage = ((uint32_t)settings.vrefCalibrated * 4096) / (ADC0_RES >> 2);
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for PORT
**
****************************************************************************************/
ISR(PORTA_PORT_vect)
{
  AC0_CTRLA &= ~AC_OUTEN_bm;
  PORTA_PIN3CTRL = 0x00;
  PORTA_INTFLAGS = PORT_INT3_bm;
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for Real Time Counter CNT
**
****************************************************************************************/
ISR(RTC_CNT_vect)
{
  /* Clear the int. flag. */
  RTC_INTFLAGS = RTC_OVF_bm;
  /* If sleep was not requested an error has occurred. */
  if(!settings.sleepRequested) createAlarm(ALARM_RTC_OVERFLOW);
  /* Send all the alarms and hope we did not interrupt command transfer, because this
   * bytes will be threated as parameters for interrupted command.
   */
  emptyAlarmQ();
  /* Something went wrong, reset itself. */
  resetProcessor();
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for Real Time Counter PIT
**
****************************************************************************************/
ISR(RTC_PIT_vect)
{
  AC0_CTRLA &= ~AC_OUTEN_bm;
  PORTA_PIN3CTRL = 0x00;
  RTC_PITINTFLAGS = RTC_PI_bm;
} /*** end of ISR ***/


/************************************ end of main.c ************************************/