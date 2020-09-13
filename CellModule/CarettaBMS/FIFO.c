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

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include "FIFO.h"


/****************************************************************************************
* Function prototypes
****************************************************************************************/
uint8_t FIFOincreaseHead(FIFO_t *p);
uint8_t FIFOincreaseTail(FIFO_t *p);


/****************************************************************************************
*                              F I F O   U T I L I T I E S
****************************************************************************************/
/************************************************************************************//**
** \brief     FIFOinit
** \param     p Pointer to the FIFO structure
** \param     buffer Pointer to the buffer
** \param     size Size of the buffer
**
****************************************************************************************/
void FIFOinit(FIFO_t *p, uint8_t *buffer, uint16_t size)
{
  p->buffer = buffer;
  p->size = size;
  p->head = buffer;
  p->tail = buffer;
} /*** end of FIFOinit ***/


/************************************************************************************//**
** \brief     FIFOflush
** \param     p Pointer to the FIFO structure
**
****************************************************************************************/
void FIFOflush(FIFO_t *p)
{
  p->tail = p->head;
} /*** end of FIFOflush ***/


/************************************************************************************//**
** \brief     FIFOincreaseHead
** \param     p Pointer to the FIFO structure
** \return    True if successful, false otherwise.
**
****************************************************************************************/
uint8_t FIFOincreaseHead(FIFO_t *p)
{
  if( p->head < (p->buffer + (p->size - 1)) ) p->head++;
  else  p->head = p->buffer;
  return (p->head != p->tail);
} /*** end of FIFOincreaseHead ***/


/************************************************************************************//**
** \brief     FIFOincreaseTail
** \param     p Pointer to the FIFO structure
** \return    True if successful, false otherwise.
**
****************************************************************************************/
uint8_t FIFOincreaseTail(FIFO_t *p)
{
  if( p->tail < (p->buffer + (p->size - 1)) ) p->tail++;
  else  p->tail = p->buffer;
  return (p->head != p->tail);
} /*** end of FIFOincreaseTail ***/


/************************************************************************************//**
** \brief     FIFOaddToBuffer
** \param     p Pointer to the FIFO structure
** \param     data Byte to be stored
** \return    True if successful, false otherwise.
**
****************************************************************************************/
uint8_t FIFOaddToBuffer(FIFO_t *p, uint8_t data)
{
  *(p->head) = data;
  FIFOincreaseHead(p);
  return (p->head != p->tail);
} /*** end of FIFOaddToBuffer ***/


/************************************************************************************//**
** \brief     FIFOavailable
** \param     p Pointer to the FIFO structure
** \return    size Size of unread data in buffer
**
****************************************************************************************/
uint16_t FIFOavailable(FIFO_t *p)
{
  if(p->head >= p->tail) return (p->head - p->tail);
  else return (p->size - (p->tail - p->head));
} /*** end of FIFOavailable ***/


/************************************************************************************//**
** \brief     FIFOread
** \param     p Pointer to the FIFO structure
** \return    data First unread byte from buffer
**
****************************************************************************************/
uint8_t FIFOread(FIFO_t *p)
{
  uint8_t data = *(p->tail);
  FIFOincreaseTail(p);
  return data;
} /*** end of FIFOread ***/


/************************************************************************************//**
** \brief     FIFOpeek
** \param     p Pointer to the FIFO structure
** \return    *(p->tail) First unread byte from buffer
**
****************************************************************************************/
uint8_t FIFOpeek(FIFO_t *p)
{
  return *(p->tail);
} /*** end of FIFOpeek ***/


/************************************ end of FIFO.h ************************************/