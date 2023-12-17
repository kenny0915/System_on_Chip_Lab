#ifndef __COMBINED_H__
#define __COMBINED_H__

#include <stdint.h>
#include <stdbool.h>

#define N      11
#define SIZE_M 4
#define SIZE_Q 10

/*----- FIR -----*/
int taps[N] = {0,-10,-9,23,56,63,56,23,-9,-10,0};
int inputbuffer[N];
int inputsignal[N] = {1,2,3,4,5,6,7,8,9,10,11};
int outputsignal[N];

/*----- Matrix Multiplication -----*/
int A[SIZE_M*SIZE_M] = {0, 1, 2, 3,
                    0, 1, 2, 3,
                    0, 1, 2, 3,
                    0, 1, 2, 3,
		   };
int B[SIZE_M*SIZE_M] = {1, 2, 3, 4,
                    5, 6, 7, 8,
                    9, 10, 11, 12,
                   13, 14, 15, 16,
		   };
int result[SIZE_M*SIZE_M];

/*----- Quick Sort -----*/
int Q[SIZE_Q] = {893, 40, 3233, 4267, 2669, 2541, 9073, 6023, 5681, 4622};


/*----- UART Comfiguration Registers -----*/
/*
#define reg_rx_data          (*(volatile uint32_t*)0x30000000)
#define reg_tx_data          (*(volatile uint32_t*)0x30000004)
#define reg_uart_stat        (*(volatile uint32_t*)0x30000008)
#define reg_uart_clkdiv      (*(volatile uint32_t*)0x3000000c)
*/


#endif

