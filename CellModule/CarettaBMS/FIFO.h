/************************************************************************************//**
* \file       FIFO.c
* \brief      This module implements a FIFO buffer.
* \ingroup    CarettaBMSFIFO
****************************************************************************************/
/************************************************************************************//**
* \defgroup   CarettaBMSFIFO FIFO buffer module
* \brief      This module implements a FIFO buffer.
* \ingroup    CarettaBMS
****************************************************************************************/
#ifndef FIFO_H_
#define FIFO_H_


/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef struct
{
  uint8_t * buffer;
  uint16_t size;
  uint8_t * head;
  uint8_t * tail;
}FIFO_t;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void FIFOinit(FIFO_t *p, uint8_t *buffer, uint16_t size);
void FIFOflush(FIFO_t *p);
uint8_t FIFOaddToBuffer(FIFO_t *p, uint8_t data);
uint16_t FIFOavailable(FIFO_t *p);
uint8_t FIFOread(FIFO_t *p);
uint8_t FIFOpeek(FIFO_t *p);


#endif /* FIFO_H_ */
/************************************ end of FIFO.h ************************************/