---
filename: 05_devGuide
title: Dev guide
layout: main
---
# Cell module source code simplified:

## State machine
```C
/* Main program entry point. */
int main(void)
{  
  /* CPUINT init */
  /* PORT init */
  /* VREF init */
  /* ADC init */
  /* AC init */
  /* USART init */
  /* TCA init */
  /* EVSYS init */
  
  /* Load settings from the EEPROM and USERROW*/
  /* Init. global variables. */
  
  /* RTC init */
  
  /* FIFO buffers init */
  
  /* Update bootloader flag, set to 0xEA to stay in the application. */
  
  /* WDT init */
  
  /* Main state machine */
  for(;;) 
  {
    /* Read new command byte. */
    uint8_t ch = readSave();
    
    if(ch == CMD_ALARM_CODES)
    {
      /* Handle command */
    }
    else if(ch == CMD_GOTO_BOOTLOADER)
    {
      /* Handle command */
    }
    else
    {
      /* Unknown command, respond with error code. */
      /* Delete received command, we dont want to forwad bad commands. */
    }
    
    /* Forward command, no need if this is the last module. */
  }
}
```

```C
uint8_t readSave(void)
{
  /* Wait for a new byte received. */
  uint8_t ch = readWait();
  /* Save byte (ch) to the command buffer. */
  /* Return received byte (ch). */
  return ch;
}
```

```C
uint8_t readWait(void)
{
  /* Do the work while no new data was received. */
  do 
  {
    /* Wait for the real time conter compare event; every  */
    if((RTC_INTFLAGS & RTC_CMP_bm))
    {
      /* Do the work */
      task(&settings);
    }
  }
  while(!FIFOavailable(&RXfifo));
  
  /* New data received, now return the received data. */
  return FIFOread(&RXfifo);
}
```

Task function performs all the main operations of the cell module BMS.
```C
void task(tSettings *p)
{
  /* Sample two external inputs. */
  /* Sample internal temperature sensor. */
  
  /* Check for overvoltage condition */
  /* Check for charged condition */
  /* Check for empty condition */
  /* Check for undervoltage condition */
  /* Check for overtemperature condition */
  /* Balance cell, if needed apply balancing current. */
  
  /* Send all the alarms, if any occurred. */
  
  /* If undervoltage condition go to sleep.
   * else if empty condition or sleep requested go to standby mode.
   */
  
  /* Call watchdog timer. */
}
```

All the received commands are saved in a buffer. Once the command has been
handled buffer with the command is forwarded to the next module. But if this
is the bottom module commands don't need to be forwarded.

```C
void forwardBufferIfNotBot(void)
{
  /* Is this the bottom module? */
  if(!settings.moduleNum)
  {
    /* Delete received command, we dont want to forwad the command. */
  }
  else
  {
    /* Forward received command. */
  }
}
```


## Communication
```C
void respond(uint8_t data)
{
  /* Is this the bottom module? */
  if(!settings.moduleNum)
  {
    /* Calculate parity bit. */
  }
  
  /* Wait for UART to be ready to send data. */
  
  /* Is this the bottom module? */
  if(!settings.moduleNum)
  { 
    /* Set ninth data bit as parity bit, this converts from 9O1 to 8E2. */
  }
  else
  {
    /* Set ninth data bit to zero, this indicates data is a response. */
  }
  
  /* Send data */
}
```

```C
void forward(uint8_t data)
{
  /* Wait for UART to be ready to send data. */
  /* Set ninth data bit to one, this indicates data is a command. */
  /* Send data */
}
```

```C
ISR(AC0_AC_vect)
{
  /* Clear int. flag */
  /* Set invert enable signal depending on AC state to trigger UART receiver. */
}
```

```C
ISR(USART0_RXC_vect)
{
  /* Read UART status registers. */
  
  /* Has frame or parity error occurred? */
  if(status & (USART_FERR_bm | USART_PERR_bm))
  {
    /* Create frame, parity alarm. */
    /* Send all the alarms. */
    /* Something went wrong, reset itself. */
  }
  /* Has receive complete interrupt occurred? */
  else if(status & USART_RXCIF_bm)
  {
    /* Is this a command? */
    if((status & 0x01) | settings.topModule)
    {
      /* Save the command, so it can be handled outside the ISR. */
    }
    /* This is a respone(not a command). */
    else
    {
      /* Respond with the data. */
    }
  }
}
```


## Voltage measurement
```C
ISR(ADC0_RESRDY_vect)
{
  /* Convert new ADC sample to voltage measurement. */
}
```


## Sleep
```C
void goToSleep(uint8_t mode)
{
  /* Make measure interval short, so we measure as soon as we wake up. */
  /* Wait for any outgoing transmitions to finish. */
  /* Enable PIN1 interrupt (wake up source). */
  /* Enable sleep mode. */
  /* Connect AC output to the PIN3 (UART signal change triggers PIN3 interrupt.)*/
  /* Go to sleep. */
  /* Clear the real time counter compare int. flag (wake up source). */
}
```

```C
ISR(PORTA_PORT_vect)
{
  /* Disconnect AC output to the PIN3 (needs to be done as soon as possible). */
  /* Disable PIN3 interrupt (wake up source). */
  /* Clear int. flag. */
}
```

```C
ISR(RTC_PIT_vect)
{
  /* Disconnect AC output to the PIN3 (needs to be done as soon as possible). */
  /* Disable PIN3 interrupt (wake up source). */
  /* Clear int. flag. */
}
```

```C
ISR(RTC_CNT_vect)
{
  /* Clear the int. flag. */
  /* If sleep was not requested an error has occurred, create alarm. */
  /* Send all the alarms */
  /* Something went wrong, reset itself. */
}
```
