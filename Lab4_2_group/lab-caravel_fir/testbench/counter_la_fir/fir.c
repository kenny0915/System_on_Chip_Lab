#include "fir.h"
#include <stdint.h>

void __attribute__ ( ( section ( ".mprjram" ) ) ) initfir() {
	
	// program data length
	wb_write(fir_len, data_length);
	wb_write(checkbits, 0x00DD0000); // Let TB know we finished programming data length

	// program coefficient
	for (uint32_t i = 0; i < 11; i++) {
		wb_write(adr_ofst(fir_coeff, 4*i), taps[i]);
	}
	wb_write(checkbits, 0x00CC0000); // Let TB know we finished programming coefficient
	
	// Check the correctness
	for (uint32_t j = 0; j < 11; j++) {
		int32_t register tmp = wb_read(adr_ofst(fir_coeff, 4*j));
		wb_write(checkbits, tmp << 16); // send output to MPRJ I/O and let TB check it
	}

}

// Referred to the source code of lab2-FIR
int* __attribute__ ( ( section ( ".mprjram" ) ) ) fir() {
	// initialization
	initfir();
	
	// StartMark
	wb_write(checkbits, 0x00A50000);

	// ap_start
	wb_write(fir_ap_ctrl, 0x1);

	uint8_t register t = 0;
	uint8_t register  x = 0;
	int8_t register y = 0;
	while (t < data_length) {
		// check ap_ctrl[4] (ss_tready is asserted)
		if (wb_read(fir_ap_ctrl) == 0x10 /* && x < data_length */) {
			wb_write(fir_x_in, x); // write X into fir
			x++;
		}
		// check ap_ctrl[5] (sm_tvalid is asserted)
		if (wb_read(fir_ap_ctrl) == 0x20) {
			y = wb_read(fir_y_out);  // read Y from fir
			outputsignal[t] = y;
			t++;
		}
	}
	// let TB check the final Y by using MPRJ[31:24]
	// and send the EndMark 5A signal at MPRJ[23:16]
	wb_write(checkbits,  outputsignal[N-1] << 24 |  0x005A0000);



	wb_write(checkbits, 0x00A50000);

	while (t < data_length) {
                // check ap_ctrl[4] (ss_tready is asserted)
                if (wb_read(fir_ap_ctrl) == 0x10 /* && x < data_length */) {
                        wb_write(fir_x_in, x); // write X into fir
                        x++;
                }
                // check ap_ctrl[5] (sm_tvalid is asserted)
                if (wb_read(fir_ap_ctrl) == 0x20) {
                        y = wb_read(fir_y_out);  // read Y from fir
                        outputsignal[t] = y;
                        t++;
                }
        }
        // let TB check the final Y by using MPRJ[31:24]
        // and send the EndMark 5A signal at MPRJ[23:16]
        wb_write(checkbits,  outputsignal[N-1] << 24 |  0x005A0000);
	
	
	wb_write(checkbits, 0x00A50000);

	while (t < data_length) {
                // check ap_ctrl[4] (ss_tready is asserted)
                if (wb_read(fir_ap_ctrl) == 0x10 /* && x < data_length */) {
                        wb_write(fir_x_in, x); // write X into fir
                        x++;
                }
                // check ap_ctrl[5] (sm_tvalid is asserted)
                if (wb_read(fir_ap_ctrl) == 0x20) {
                        y = wb_read(fir_y_out);  // read Y from fir
                        outputsignal[t] = y;
                        t++;
                }
        }
        // let TB check the final Y by using MPRJ[31:24]
        // and send the EndMark 5A signal at MPRJ[23:16]
        wb_write(checkbits,  outputsignal[N-1] << 24 |  0x005A0000);

	return outputsignal;
}
