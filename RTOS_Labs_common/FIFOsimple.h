// FIFOsimple.h
// Runs on any LM3Sxxx
// Provide functions that initialize a FIFO, put data in, get data out,
// and return the current size.  The file includes a transmit and receive
// FIFO using index and pointer implementations.  Implementations provided
// here are equivalent to using macros from FIFO.h.

// Daniel Valvano
// May 2, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
      Programs 3.7, 3.8., 3.9 and 3.10 in Section 3.7

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#ifndef __FIFOSIMPLE_H__
#define __FIFOSIMPLE_H__

// Transmit FIFO
// can hold 0 to TXFIFOSIZE elements
#define TXFIFOSIZE 1024 // must be a power of 2
#define TXFIFOSUCCESS 1
#define TXFIFOFAIL    0
typedef char txDataType;

// initialize FIFO
void TxFifo_Init(void);
// add element to end of FIFO
// return TXFIFOSUCCESS if successful
int TxFifo_Put(txDataType data);
// remove element from front of FIFO
// return TXFIFOSUCCESS if successful
int TxFifo_Get(txDataType *datapt);
// number of elements in FIFO
// 0 to TXFIFOSIZE-1
uint32_t TxFifo_Size(void);

// Receive FIFO
// can hold 0 to RXFIFOSIZE-1 elements
#define RXFIFOSIZE 1024 // can be any size
#define RXFIFOSUCCESS 1
#define RXFIFOFAIL    0

typedef char rxDataType;

// initialize FIFO
void RxFifo_Init(void);
// add element to end of FIFO
// return RXFIFOSUCCESS if successful
int RxFifo_Put(rxDataType data);
// remove element from front of FIFO
// return RXFIFOSUCCESS if successful
int RxFifo_Get(rxDataType *datapt);
// number of elements in FIFO
// 0 to RXFIFOSIZE-1
uint32_t RxFifo_Size(void);


#endif //  __FIFOSIMPLE_H__
