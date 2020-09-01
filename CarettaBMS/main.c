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
****************************************************************************************/


/****************************************************************************************
* Include files
****************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "CarettaBMS_CMD.h"
#include "command.h"
#include "FIFO.h"


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define F_CPU (3333333333)

#define RX_BUFFER_SIZE 16
#define TX_BUFFER_SIZE 16

#define readUserRow(x) (*x)
#define msToCounterTop(x) (x * (F_CPU / 256000))

#define UR_BOOTLOADER_CONDITION_ADDR ((volatile uint8_t  *)(&USERROW_USERROW0 + 31))
#define UR_MODULE_NUMBER_ADDR        ((volatile uint8_t  *)(&USERROW_USERROW0 + 30))
#define UR_VREF_CALIBRATION_ADDR     ((volatile uint16_t *)(&USERROW_USERROW0 + 28))
#define UR_BALANCE_CURRENT_ADDR      ((volatile uint16_t *)(&USERROW_USERROW0 + 26))
#define UR_MAX_TEMPERATURE_BMS_ADDR  ((volatile uint8_t  *)(&USERROW_USERROW0 + 25))
#define UR_MAX_TEMPERATURE_BAT_ADDR  ((volatile uint8_t  *)(&USERROW_USERROW0 + 24))
#define UR_HIGH_VOLT_LIMIT_ADDR      ((volatile uint16_t *)(&USERROW_USERROW0 + 22))
#define UR_CHARGED_VOLTAGE_ADDR      ((volatile uint16_t *)(&USERROW_USERROW0 + 20))
#define UR_EMPTY_VOLTAGE_ADDR        ((volatile uint16_t *)(&USERROW_USERROW0 + 18))
#define UR_LOW_VOLT_LIMIT_ADDR       ((volatile uint16_t *)(&USERROW_USERROW0 + 16))
#define UR_EXT_TEMP_CAL_ADDR         ((volatile uint16_t *)(&USERROW_USERROW0 + 14))
#define UR_MEASURE_INTERVAL_ADDR     ((volatile uint16_t *)(&USERROW_USERROW0 + 12))
#define UR_BALANCE_START_ADDR        ((volatile uint16_t *)(&USERROW_USERROW0 + 10))
#define UR_BALANCE_STOP_ADDR         ((volatile uint16_t *)(&USERROW_USERROW0 + 8))
#define UR_BALANCE_HYSTERESIS_ADDR   ((volatile uint16_t *)(&USERROW_USERROW0 + 6))
#define UR_RESET_SETTINGS_ADDR       ((volatile uint8_t  *)(&USERROW_USERROW0 + 5))

#define RESP_OK                     (uint8_t)(0x0F)
#define RESP_FRAME_PARITY_ERROR     (uint8_t)(0xFE)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void task(void);
void syncResponse(void);
void balanceCell(void);
void setBalancingCurrent(uint16_t current);
void watchdogConfig(uint8_t x);
void goToBootloader(void);
void goToSleep(void);
void calcVoltageReading(void);
void measureTemperature(void);
uint16_t takeADCreading(void);
uint8_t readWait(void);
void respondIfBot(uint8_t ch);
void respondIfNotBot(uint8_t ch);
void respond(uint8_t data);
void forwardBufferIfNotBot(void);
void forward(uint8_t data);
uint8_t readSave(void);
void sequenceError(void);
void readSaveMultiple(uint8_t n);
void save2byteParameter(volatile uint16_t *parameter, volatile uint16_t *addr);
void writeUserRow1byte(volatile uint8_t *addr, uint8_t val);
void writeUserRow2bytes(volatile uint16_t *addr, uint16_t val);
void writePageSPM(void);
/* tmp */
void printNumber(uint16_t num);


/****************************************************************************************
* Local data declaration
****************************************************************************************/
FIFO_t RXfifo;
uint8_t RXbuffer[RX_BUFFER_SIZE];

FIFO_t TXfifo;
uint8_t TXbuffer[TX_BUFFER_SIZE];

volatile uint8_t waitForParameters = 0x00;
volatile uint8_t balancingStarted = 0x00;
volatile uint16_t targetVoltage = 5000;
volatile uint16_t ADCres = 0x00;

/* Parameters */
volatile uint8_t moduleNum = 0xFF;
volatile uint32_t cellVoltage = 4200;
volatile uint16_t vrefCalibrated = 1500;
volatile uint16_t balanceCurrent = 100;
volatile uint8_t maxTempBMS = 70;
volatile uint8_t maxTempBAT = 70;
volatile uint16_t voltLimitHigh = 4300;
volatile uint16_t voltageCharged = 4200;
volatile uint16_t voltageEmpty = 3200;
volatile uint16_t voltLimitLow = 2500;
volatile uint16_t extTempCal = 0;
volatile uint16_t measureInterval = 100;
volatile uint16_t voltBalanceStart = 3600;
volatile uint16_t voltBalanceStop = 4200;
volatile uint16_t balanceHysteresis = 50;


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
  VREF_CTRLA = VREF_ADC0REFSEL_1V5_gc | VREF_DAC0REFSEL_4V34_gc;
  
  /* AC init */
  AC0_MUXCTRLA = AC_MUXNEG_VREF_gc | AC_INVERT_bm;
  AC0_CTRLA = AC_RUNSTDBY_bm | AC_HYSMODE_50mV_gc | AC_ENABLE_bm;
  AC0_INTCTRL = AC_CMP_bm;
  
  /* USART init */
  USART0_CTRLA = USART_RXCIE_bm; //USART_LBME_bm
  USART0_CTRLB = USART_RXEN_bm | USART_TXEN_bm;
  USART0_CTRLC |= USART_PMODE_ODD_gc | USART_CHSIZE_9BITL_gc;
  USART0_BAUD = 1389;
  
  /* ADC init */
  ADC0_CTRLA = ADC_ENABLE_bm;
  ADC0_CTRLB = 0x04; //ACC16
  ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV2_gc;
  ADC0_CTRLD = ADC_INITDLY_DLY32_gc | ADC_ASDV_ASVON_gc | 0x02;
  ADC0_MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0_EVCTRL = ADC_STARTEI_bm;
  ADC0_INTCTRL = ADC_RESRDY_bm;
  
  /* TCA init */
  TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
  TCA0_SINGLE_PER = 102;
  TCA0_SINGLE_CMP0 = 0;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm;
  
  /* TCB init */
  TCB0_CCMP = 0x00FF;
  TCB0_CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
  
  /* RTC init */
  RTC_INTCTRL = RTC_CMP_bm; //RTC_OVF_bm
  RTC_CLKSEL = RTC_CLKSEL_INT1K_gc;
  //RTC_PER = 0x00;
  //RTC_CMP = 0x00;
  //RTC_CTRLA = RTC_RUNSTDBY_bm | RTC_PRESCALER0_bm | RTC_RTCEN_bm;
  RTC_PITINTCTRL = 0x01;
  //RTC_PITCTRLA = RTC_PERIOD_CYC8_gc | RTC_PITEN_bm;
  
  /* EVSYS init */
  EVSYS_SYNCCH0 = EVSYS_SYNCCH0_TCA0_CMP0_gc;
  EVSYS_ASYNCUSER1 = EVSYS_ASYNCUSER1_SYNCCH0_gc;
  
  /* Read user row */
  moduleNum = 0x00;
  //moduleNum = readUserRow(UR_MODULE_NUMBER_pos);
  
  /* FIFO init */
  FIFOinit(&RXfifo, RXbuffer, RX_BUFFER_SIZE);
  FIFOinit(&TXfifo, TXbuffer, TX_BUFFER_SIZE);
  
  writeUserRow1byte(UR_BOOTLOADER_CONDITION_ADDR, 0xEA);
  
  /* WDT init */
  //watchdogConfig(WDT_PERIOD_1KCLK_gc);
  
  sei();
  
  while (1) 
  {
    uint8_t ch = readSave();
    
    /*  */
    if(ch == CMD_ALARM_INTERVAL)
    {
      
    }
    
    /*  */
    else if(ch == CMD_ALARM_CONT_SINGLE)
    {
      
    }
    
    /*  */
    else if(ch == CMD_VOLTAGE_CAL)
    {
      if(moduleNum == readSave())
      {
        save2byteParameter(&vrefCalibrated, UR_VREF_CALIBRATION_ADDR);
        FIFOflush(&TXfifo);
      }
      else
      {
        readSaveMultiple(2);
      }
    }
    
    /*  */
    else if(ch == CMD_BALANCE_CURRENT)
    {
      save2byteParameter(&balanceCurrent, UR_BALANCE_CURRENT_ADDR);
    }
    
    else if(ch == CMD_EXT_TEMPERATURE)
    {
      
    }
    
    /*  */
    else if(ch == CMD_MAX_TEMP_BMS)
    {
      maxTempBMS = readSave();
      writeUserRow1byte(UR_MAX_TEMPERATURE_BMS_ADDR, maxTempBMS);
      respond(RESP_OK);
    }
    
    /*  */
    else if(ch == CMD_MAX_TEMP_BAT)
    {
      maxTempBAT = readSave();
      writeUserRow1byte(UR_MAX_TEMPERATURE_BAT_ADDR, maxTempBAT);
      respond(RESP_OK);
    }
    
    /*  */
    else if(ch == CMD_HIGH_VOLT_LIMIT)
    {
      save2byteParameter(&voltLimitHigh, UR_HIGH_VOLT_LIMIT_ADDR);
    }
    
    /*  */
    else if(ch == CMD_CHARGED_VOLTAGE)
    {
      save2byteParameter(&voltageCharged, UR_CHARGED_VOLTAGE_ADDR);
    }
    
    /*  */
    else if(ch == CMD_EMPTY_VOLTAGE)
    {
      save2byteParameter(&voltageEmpty, UR_EMPTY_VOLTAGE_ADDR);
    }
    
    /*  */
    else if(ch == CMD_LOW_VOLT_LIMIT)
    {
      save2byteParameter(&voltLimitLow, UR_LOW_VOLT_LIMIT_ADDR);
    }
    
    /*  */
    else if(ch == CMD_EXT_TEMP_CAL)
    {
      save2byteParameter(&extTempCal, UR_EXT_TEMP_CAL_ADDR);
    }
    
    /*  */
    else if(ch == CMD_MEASURE_INTERVAL)
    {
      save2byteParameter(&measureInterval, UR_MEASURE_INTERVAL_ADDR);
    }
    
    /*  */
    else if(ch == CMD_NMBR_OF_MODULES)
    {
      moduleNum = readWait() - 1;
      writeUserRow1byte(UR_MODULE_NUMBER_ADDR, moduleNum);
      FIFOaddToBuffer(&TXfifo, moduleNum);
    }
    
    /*  */
    else if(ch == CMD_BALANCE_START)
    {
      save2byteParameter(&voltBalanceStart, UR_BALANCE_START_ADDR);
    }
    
    /*  */
    else if(ch == CMD_BALANCE_STOP)
    {
      save2byteParameter(&voltBalanceStop, UR_BALANCE_STOP_ADDR);
    }
    
    /*  */
    else if(ch == CMD_BALANCE_HYSTERESIS)
    {
      save2byteParameter(&balanceHysteresis, UR_BALANCE_HYSTERESIS_ADDR);
    }
    
    /*  */
    else if(ch == CMD_RESET_SETTINGS)
    {
      writeUserRow1byte(UR_RESET_SETTINGS_ADDR, 0x01);
      respond(RESP_OK);
    }
    
    /*  */
    else if(ch == CMD_GOTO_SLEEP)
    {
      goToSleep();
    }
    
    /*  */
    else if(ch == CMD_INT_TEMPERATURE)
    {
      measureTemperature();
    }
    
    /*  */
    else if(ch == CMD_TARGET_VOLTAGE)
    {
      
    }
    
    /*  */
    else if(ch == CMD_CELL_VOLTAGE)
    {
      volatile uint16_t val = cellVoltage;
      respond((uint8_t)val);
      respond((uint8_t)(val >> 8));
    }
    
    /*  */
    else if(ch == CMD_WAKE_UP_SYNC)
    {
      
    }
    
    /*  */
    else if(ch == CMD_EXTRA_0)
    {
      
    }
    
    /*  */
    else if(ch == CMD_EXTRA_1)
    {
      
    }
    
    /*  */
    else if(ch == CMD_EXTRA_2)
    {
      
    }
    
    /*  */
    else if(ch == Cmnd_STK_GET_SYNC)
    {
      goToBootloader();
    }
    
    /*  */
    else
    {
      respond(0xCE);
      FIFOflush(&TXfifo);
    }
    
    forwardBufferIfNotBot();
  }
} /*** end of main ***/


/************************************************************************************//**
** \brief     Respond with a number in ASCII
** \param     num Number to send
**
****************************************************************************************/
void printNumber(uint16_t num)
{
  uint8_t leadingZero = 1;
  for(uint16_t decade = 10000; decade > 0; decade /= 10)
  {
    uint8_t digit = num/decade;
    if((!leadingZero) || (digit != 0))
    {
      leadingZero = 0;
      respond(digit + '0');
    }
    num %= decade;
  }
  respond('\r');
  respond('\n');
} /*** end of printNumber ***/


/************************************************************************************//**
** \brief     Periodic task
**
****************************************************************************************/
void task(void)
{
  calcVoltageReading();
  balanceCell();
  measureTemperature();
  asm("wdr");
  //if("out of time") respond(0xEE);
} /*** end of task ***/


/************************************************************************************//**
** \brief     Set balancing current
** \param     current Balancing current in percentage
**
****************************************************************************************/
void setBalancingCurrent(uint16_t current)
{
  
} /*** end of setBalancingCurrent ***/


/************************************************************************************//**
** \brief     Balance cell voltage
**
****************************************************************************************/
void balanceCell(void)
{
  if((cellVoltage > voltBalanceStart) && (cellVoltage < voltBalanceStop))
  {
    if(cellVoltage <= targetVoltage)
    {
      balancingStarted = 0x00;
      setBalancingCurrent(0x0000);
    }
    else if((cellVoltage > (targetVoltage + balanceHysteresis)) | balancingStarted)
    {
      balancingStarted = 0x01;
      setBalancingCurrent(balanceCurrent);
    }
  }
  else
  {
    balancingStarted = 0x00;
    setBalancingCurrent(0x0000);
  }
} /*** end of balanceCell ***/


/************************************************************************************//**
** \brief     Calculate cell voltage from ADC result
**
****************************************************************************************/
void calcVoltageReading(void)
{
  volatile uint16_t res = ADCres;
  cellVoltage = ((uint32_t)vrefCalibrated * 4096) / (res >> 2);
} /*** end of calcVoltageReading ***/


/************************************************************************************//**
** \brief     Take a temperature reading
**
****************************************************************************************/
void measureTemperature(void)
{
  /* Disable timer. */
  TCB0_CTRLA = 0x00;
  /* ADC init */
  ADC0_INTCTRL = 0x00;
  ADC0_EVCTRL = 0x00;
  /* VREF init */
  VREF_CTRLA = VREF_ADC0REFSEL_1V1_gc | VREF_DAC0REFSEL_4V34_gc;
  /* Switch to internal reference. */
  ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV2_gc;
  /* Select internal temperature sensor input. */
  ADC0_MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
  takeADCreading();
  
  int8_t sigrow_offset = *(int8_t *)(&SIGROW_TEMPSENSE1);
  uint8_t sigrow_gain = *(uint8_t *)(&SIGROW_TEMPSENSE0);
  uint16_t adc_reading =  takeADCreading() >> 4;
  uint32_t temp = adc_reading - sigrow_offset;
  temp *= sigrow_gain;
  temp += 0x80;
  temp >>= 8;
  uint16_t temperature_in_K = temp;
  respond(temperature_in_K - 273);
  
  /* VREF init */
  VREF_CTRLA = VREF_ADC0REFSEL_1V5_gc | VREF_DAC0REFSEL_4V34_gc;
  /* ADC reinit */
  ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV2_gc;
  ADC0_MUXPOS = ADC_MUXPOS_INTREF_gc;
  takeADCreading();
  /* Enable triggers. */
  ADC0_EVCTRL = ADC_STARTEI_bm;
  ADC0_INTCTRL = ADC_RESRDY_bm;
  /* Enable timer. */
  TCB0_CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
} /*** end of measureTemperature ***/


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
** \brief     Read and save 2 byte parameter to user row
** \param     parameter Pointer to the local parameter variable
** \param     addr Parameter address
**
****************************************************************************************/
void save2byteParameter(volatile uint16_t *parameter, volatile uint16_t *addr)
{
  *parameter = readSave();
  *parameter |= readSave() << 8;
  writeUserRow2bytes(addr, *parameter);
  respond(RESP_OK);
} /*** end of save2byteParameter ***/


/************************************************************************************//**
** \brief     Read and save multiple bytes
** \param     n Number ob bytes to read
**
****************************************************************************************/
void readSaveMultiple(uint8_t n)
{
  while(n--) readSave();
} /*** end of readSaveMultiple ***/


/************************************************************************************//**
** \brief     Read and save byte
** \return    Retun read byte
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
** \return    Retun first byte from buffer
**
****************************************************************************************/
uint8_t readWait(void)
{
  while(!FIFOavailable(&RXfifo));
  return FIFOread(&RXfifo);
} /*** end of readWait ***/


/************************************************************************************//**
** \brief     Go to bootloader
**
****************************************************************************************/
void goToBootloader(void)
{
  syncResponse();
  writeUserRow1byte(UR_BOOTLOADER_CONDITION_ADDR, 0xEB);
  while(!(USART0_STATUS & USART_TXCIF_bm));
  _PROTECTED_WRITE(RSTCTRL_SWRR, RSTCTRL_SWRE_bm);
} /*** end of goToBootloader ***/


/************************************************************************************//**
** \brief     Put processor to sleep
**
****************************************************************************************/
void goToSleep(void)
{
  /* go to sleep */
  PORTA_PIN3CTRL = 0x01;
  SLPCTRL_CTRLA = (0x01 << 1) | SLPCTRL_SEN_bm;
  AC0_CTRLA |= AC_OUTEN_bm;
  asm("sleep");
} /*** end of goToSleep ***/


/************************************************************************************//**
** \brief     Only respond if it's not the bottom module
** \param     ch Byte to respond with
**
****************************************************************************************/
void respondIfNotBot(uint8_t ch)
{
  if(moduleNum) respond(ch);
} /*** end of respondIfNotBot ***/


/************************************************************************************//**
** \brief     Only respond if it's the bottom module
** \param     ch Byte to respond with
**
****************************************************************************************/
void respondIfBot(uint8_t ch)
{
  if(!moduleNum) respond(ch);
} /*** end of respondIfBot ***/


/************************************************************************************//**
** \brief     Only forward buffer if it's not the bottom module
**
****************************************************************************************/
void forwardBufferIfNotBot(void)
{
  waitForParameters = 0x00;
  
  if(moduleNum)
  {
    while(FIFOavailable(&TXfifo))
    {
      forward(FIFOread(&TXfifo));
    }
  }
  else
  {
    FIFOflush(&TXfifo);
  }
} /*** end of forwardBufferIfNotBot ***/


/************************************************************************************//**
** \brief     Write page of self programming memory
**
****************************************************************************************/
void writePageSPM(void)
{
  _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
  while(NVMCTRL_STATUS & NVMCTRL_FBUSY_bm);
} /*** end of writePageSPM ***/


/************************************************************************************//**
** \brief     Write 1 byte to user row
** \param     addr Pointer to the user row position
** \param     val Value to save
**
****************************************************************************************/
void writeUserRow1byte(volatile uint8_t *addr, uint8_t val)
{
  *addr = val;
  writePageSPM();
} /*** end of writeUserRow1byte ***/


/************************************************************************************//**
** \brief     Write 2 bytes to user row
** \param     addr Pointer to the user row position
** \param     val Value to save
**
****************************************************************************************/
void writeUserRow2bytes(volatile uint16_t *addr, uint16_t val)
{
  *addr = val;
  writePageSPM();
} /*** end of writeUserRow2bytes ***/


/************************************************************************************//**
** \brief     Sync command response
**
****************************************************************************************/
void syncResponse(void)
{
  if(readSave() != Sync_CRC_EOP) sequenceError();
  
  respondIfBot(Resp_STK_INSYNC);
  respondIfBot(Resp_STK_OK);
  
  forwardBufferIfNotBot();
} /*** end of syncResponse ***/


/************************************************************************************//**
** \brief     Configure watchdog timer
** \param     x Watchdog control register A setting
**
****************************************************************************************/
void watchdogConfig(uint8_t x)
{
  _PROTECTED_WRITE(WDT_CTRLA, x);
} /*** end of watchdogConfig ***/


/************************************************************************************//**
** \brief     Reset processor for sequence error
**
****************************************************************************************/
void sequenceError(void)
{
  watchdogConfig(WDT_PERIOD_8CLK_gc);
  while(1);
} /*** end of sequenceError ***/


/************************************************************************************//**
** \brief     Respond with single byte
** \param     data Byte to respond with
**
****************************************************************************************/
void respond(uint8_t data)
{
  volatile uint8_t parityBit;
  
  if(!moduleNum)
  {
    //parityBit = __builtin_parity(data);
    parityBit = data;
    parityBit ^= parityBit >> 4;
    parityBit ^= parityBit >> 2;
    parityBit ^= parityBit >> 1;
  }
  
  while(!(USART0_STATUS & USART_DREIF_bm));
  
  if(!moduleNum) USART0_TXDATAH = parityBit;
  else USART0_TXDATAH = 0x01;
  
  USART0_TXDATAL = data;
} /*** end of respond ***/


/************************************************************************************//**
** \brief     Forward sinle byte
** \param     data Byte to forward
**
****************************************************************************************/
void forward(uint8_t data)
{
  while(!(USART0_STATUS & USART_DREIF_bm));
  USART0_TXDATAH = 0x00;
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
  PORTA_PIN7CTRL = (~AC0_STATUS << (PORT_INVEN_bp - AC_STATE_bp)) & PORT_INVEN_bm;
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
    USART0_STATUS = USART_TXCIF_bm;
    while(!(USART0_STATUS & USART_DREIF_bm));
    USART0_TXDATAH = 0x01;
    USART0_TXDATAL = 0xFE;
    while(!(USART0_STATUS & USART_TXCIF_bm));
    sequenceError();
  }
  else if(status & USART_RXCIF_bm)
  {
    if((!(status & 0x01)) | waitForParameters)
    {
      waitForParameters = 0x01;
      *(RXfifo.head) = data;
      if(RXfifo.head < (RXbuffer + (RX_BUFFER_SIZE - 1))) RXfifo.head++;
      else  RXfifo.head = RXbuffer;
    }
    else
    {
      volatile uint8_t parityBit;
      if(!moduleNum)
      {
        parityBit = data;
        parityBit ^= parityBit >> 4;
        parityBit ^= parityBit >> 2;
        parityBit ^= parityBit >> 1;
      }
      
      while(!(USART0_STATUS & USART_DREIF_bm));
      
      if(!moduleNum) USART0_TXDATAH = parityBit;
      else USART0_TXDATAH = 0x01;
      
      USART0_TXDATAL = data;
    }
    asm ("wdr");
  }
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for ADC result ready
**
****************************************************************************************/
ISR(ADC0_RESRDY_vect)
{
  ADCres = ADC0_RES;
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
  
} /*** end of ISR ***/


/************************************************************************************//**
** \brief     Interrupt service routine for Real Time Counter PIT
**
****************************************************************************************/
ISR(RTC_PIT_vect)
{
  
} /*** end of ISR ***/


/************************************ end of main.c ************************************/