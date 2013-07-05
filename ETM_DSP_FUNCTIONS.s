
.ifdef __dsPIC30F
        .include "p30fxxxx.inc"
.endif
.ifdef __dsPIC33F
        .include "p33Fxxxx.inc"
.endif

.section .nbss, bss, near    
	_saturation_scale13Q3_count:		.space 2
	.global _saturation_scale13Q3_count
	_saturation_makescale13Q3_count:	.space 2
	.global _saturation_makescale13Q3_count
	_saturation_scale16_count:	.space 2
	.global _saturation_scale16_count
.text



	
        .global  _AverageADC128
        .text
_AverageADC128:
	mov		W0, W4		; move source address
	CLR		A		; 0 40 bit Acc

ACC_S:	REPEAT	#127			; add em all up
	ADD		[W4++], #4, A ; signed 16 add to ACCA (right shift 4 bits)
	                                      ; The data that we want is now stored in the 15 LSB of ACCAH and the 1 MSB of ACCAL
	                                      ; If we shift the data left one bit and call SAC.R the data will be bashed because
	                                      ; The accumulator will be signed.  There for we must work around this little problem 
	SAC		A, W0	      ; Move ACCAH to W0
	SL		W0, #1, W0    ; Shift W0 left by one bit.  
 	SFTAC           A, #-1        ; Shift Accumulator left by one bit.
	SAC.R           A, W1         ; Move ACCAH to W1
	AND             #0x0001, W1   ; W1 &= 0x0001
	IOR             W0, W1, W0    ; WO = WO | W1
	return



	
	
        .global  _AverageADC16
        .text
_AverageADC128:
	mov		W0, W4		; move source address
	CLR		A		; 0 40 bit Acc

ACC_S:	REPEAT	#15			; add em all up
	ADD		[W4++], #5, A ; signed 16 add to ACCA (right shift 5 bits)
	                                      ; The data that we want is now stored in the 15 LSB of ACCAH and the 1 MSB of ACCAL
	                                      ; If we shift the data left one bit and call SAC.R the data will be bashed because
	                                      ; The accumulator will be signed.  There for we must work around this little problem 
	SAC		A, W0	      ; Move ACCAH to W0
	SL		W0, #1, W0    ; Shift W0 left by one bit.  
 	SFTAC           A, #-1        ; Shift Accumulator left by one bit.
	SAC.R           A, W1         ; Move ACCAH to W1
	AND             #0x0001, W1   ; W1 &= 0x0001
	IOR             W0, W1, W0    ; W0 = W0 | W1
	return


	



	;; ----------------------------------------------------------

	
	.global  _Scale13Q3
	;; uses and does not restore W0->W3
	.text
_Scale13Q3:
	;; Value is stored in w0
	;; Scale is stored in w1

	MUL.UU		W0,W1, W2 		; Multiply W0 by W1 and store in W2:W3, MSW is stored in W3
	MUL.UU		W3,#8,W0		; Multiply W3 by 8 (shift left 3 bits) and store the results in W0:W1 - W0 is the result we care about
						; If W1 is not Zero, then there was an overflow
	CP0		W1
	BRA		Z, _scale13q3_no_overflow
_scale13q3_overflow:
	;; There was an overflow in the shift opertion
	;; Increment the overflow counter and set the result to 0xFFFF
	MOV		#0xFFFF, W0
	INC		_saturation_scale13Q3_count
_scale13q3_no_overflow:	
	;; OR together W0, W1 into W0 to give the final results
	LSR		W2, #13, W1		; Take the 3 MSbits of W2 and store then as the 3 LSB of W1
	IOR		W0, W1, W0		; Add W1 to W0 (using bitwise or in this case)
	RETURN



	;; DPARKER need to write _ReverseScale13Q3  (see A34760 project serial.c)



	;; ---------------------------------------------------------------------------

        .global  _MakeScale13Q3	
	.text
_MakeScale13Q3:
	;; num is scored in W0
	;; den is stored in W1
	MOV 		W1,W4
	;; Left shift W0 by 13 bits, and store the results in W2&W3
	LSR		W0, #3, W3 		; This is the high word of the 32 bit results
	SL		W0, #13,W2		; This is the low word of the 32 bit results
	REPEAT		#17
	DIV.UD		W2,W4			; Store the results in W0,W1
	;; Error Check for Overflow
	BRA 		NOV, _makescale13q3_no_overflow
_makescale13q3_overflow:
	;; There was an overflow in the multiplication
	;; Increment the overflow counter and set the result to 0xFFFF
	INC		_saturation_makescale13Q3_count
	MOV		#0xFFFF, W0
_makescale13q3_no_overflow:	
	RETURN





	;; -------------------------------------------------------------------------
	
	.global  _Scale16Bit
	.text
_Scale16Bit:
	;; w0 is value to get scaled
	;; w1 is num
	;; w2 is den

	;; This function has a max scale value of 1, so if num >= den, just return the initial value
	CP 		W1,W2
	BRA		NN, _scale16bit_done

	;; Den > num, so this function makes sense to compute
	MUL.UU		W0,W1,W4 		; Multiply W0 by W1 and store the 32 bit results in W4:W5
	REPEAT		#17
	DIV.UD		W4,W2			; Store the results in W0,W1
	BRA 		NOV, _scale16bit_no_overflow
_scale16bit_overflow:
	;; Dparker it should be impossible to get here because we already checked the num and den
	;; There was an overflow in the multiplication
	;; Increment the overflow counter and set the result to 0xFFFF
	INC		_saturation_scale16_count
	MOV		#0xFFFF, W0
_scale16bit_no_overflow:	


_scale16bit_done:	
	RETURN







	;; ---------------------------------------------------------------------
	
	.global  _RCFilterNTau
	.text
_RCFilterNTau:
	;; w0 previous value
	;; w1 current reading
	;; w2 filter tau bit shift (if this value is greater than 15, funny things may happen)
	
	;; W3 is used to store the multipler
	;; W4:45 are use as the multiplication results register

	;;  If N < 16, keep going, otherwise set N = 15
	CP 		W2, #16
	BRA 		LT, _RCFilterNTauOK
	MOV		#15, W2
_RCFilterNTauOK:	

	;; If the previous value is zero, then we want to start filtering not from zero, but from the current value
	;; This will make the response at startup much faster
	CP0		W0
	BRA		NZ, _RCFilterNTau_previous_value_not_zero
	MOV		W1, W0
_RCFilterNTau_previous_value_not_zero:	

	;; First Compute the multiplier from N (tau bit shift)
	MOV		#1, W3
	SL		W3, W2, W3 		
	DEC		W3, W3				; W3 now contains 2^N -1
	
	MUL.UU		W3, W0, W4 			; Store results in W4, W5, High Word is W5
	ADD		W1, W4, W4			
        ADDC		#0, W5	                        ; Add W1 to W4:W5

	;; Now need to shift W4:W5 right by N bits and store results in W0
	LSR		W4, W2, W4 			; Shit W4 Right by N bits	
	SUBR		W2, #16, W2					
	SL		W5, W2, W5 			; Shift W5 Left by 16-N bits
	IOR		W4, W5, W0			


	;; Need to provide a little more math to help to filtered value stabilize
	CP		W1, W0
	BRA		Z, _RCFilterNTau_AdjustDone
	BRA		LT, _RCFilterNTau_AdjustNegative
	INC2		W0, W0				; Increment W0 by 2 (it will be decremented 1 by the next command)

_RCFilterNTau_AdjustNegative:	
	DEC		W0, W0

_RCFilterNTau_AdjustDone:		
	RETURN







	;; DPARKER SetPowerSupplyTarget function takes a LONG time to execute, move this over to a better power supply function
	
	
				;--------End of All Code Sections ---------------------------------------------
.end                               ;End of program code in this file



