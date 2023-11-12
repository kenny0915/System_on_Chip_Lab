#ifndef __FIR_H__
#define __FIR_H__

#define fir_ap_ctrl 0x30000000 // ap_control
#define fir_len     0x30000010 // data length

#define fir_coeff   0x30000040 // Load into TapRAM

#define fir_x_in    0x30000080 // Load X into DataRAM
#define fir_y_out   0x30000084 // Take the output Y

#define checkbits   0x2600000C // MPRJ I/O

#define N 11
#define data_length N

int taps[11] = {0,-10,-9,23,56,63,56,23,-9,-10,0};
int inputbuffer[N];
int inputsignal[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
int outputsignal[N];

#define adr_ofst(target, offset) (target + offset)
#define wb_write(target, data)   (*(volatile uint32_t*)(target)) = data // wishbone write
#define wb_read(target)   	 (*(volatile uint32_t*)(target))        // wishbone read 



#endif
