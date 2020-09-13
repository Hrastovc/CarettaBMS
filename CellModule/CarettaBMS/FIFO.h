/************************************************************************************//**
* \file       FIFO.c
* \brief      This module implements a FIFO buffer.
* \ingroup    CarettaBMSFIFO
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