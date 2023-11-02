#include "fir.h"

void __attribute__ ( ( section ( ".mprjram" ) ) ) initfir() {
	//initial your fir
	for(int i=0; i<N; i++){
		inputbuffer[i] = 0;
		outputsignal[i] = 0;
	}
}

int* __attribute__ ( ( section ( ".mprjram" ) ) ) fir(){
	initfir();
	//write down your fir
	for (int t = 0; t < N; t++) {
        	// shifting ram value
        	for (int j = N - 1; j > 0; j--) {
            		inputbuffer[j] = inputbuffer[j - 1];
        	}
		// assign value
        	inputbuffer[0] = inputsignal[t];
		// FIR calculation
		for(int i=0; i<N; i++){		
			outputsignal[t] += taps[i]*inputbuffer[i];
		}
	}
	return outputsignal;
}
		
