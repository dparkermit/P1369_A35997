        .include "p30f6014A.inc"


.section .nbss, bss, near    
	;; 	_spi2_state:				.space 2
	;; 	.global _spi2_state
	;; 	_ad7686_result_index:			.space 2
	;; 	.global _ad7686_result_index
.text	

	
	.global  __SPI2Interrupt
__SPI2Interrupt:	
	;; Handles the SPI2 Interrupt

	;; W0 will hold the address of the array to add data to
	;; W1 will hold spi2_state & then the position in the array to add data to
	;; W2 will hold data

	
	push.s
	PUSH 	CORCON
	PUSH	SR
	lnk 	#0x4
	bclr.b	0x0087, #2
	bset	LATB, #12 			; Set PIN_TEST_POINT_27  DPARKER - Remove this line for productions System  
	
	MOV	_spi2_state, W1			;
	BRA	W1

	;; Location 0
	BRA	_spi2interrupt_read_b_and_start_conversion
	;; location 1
	BRA	_spi2interrupt_start_a_transfer
	;; location 2
	BRA	_spi2interrupt_read_a_and_start_b_transfer

_spi2interrupt_read_b_and_start_conversion:	
	BSET	LATC, #3			; PIN_AD7686_2_CS = 1;
	BCLR 	LATC, #2			; PIN_AD7686_CONVERT = 0;

	MOV	_fwd_det_b_pointer, W0		; Next Three lines W0 = Address of forward_power_detector_B[ad7686_result_index]  
	MOV	_ad7686_result_index, W1	; 
	ADD	W0, W1, W0			;
	ADD	W0, W1, W0			;
	
	MOV	SPI2BUF, W2			; forward_power_detector_B[ad7686_result_index] = SPI2BUF
	MOV	W2, [W0]			; 

	BSET 	LATC, #2			; PIN_AD7686_CONVERT = 1;
	
	MOV	W2, SPI2BUF			; Start the next SPI2 Process by writing Dummy Value to SPI2BUF
	
	MOV 	#1, W2
	MOV	W2, _spi2_state			;

	INC	W1, W1				; Increment ad7686_result_index circular buffer index
	AND	#0x00F, W1				
	MOV 	W1, _ad7686_result_index
	
	BRA	_spi2interrupt_done
_spi2interrupt_start_a_transfer:
	MOV	SPI2BUF, W2	                ; Dummy read of SPI2BUF

	BCLR	LATC, #4	      		; PIN_AD7686_1_CS = 0

	MOV	W2, SPI2BUF 			; Dummy write to SPI2BUF

	MOV	#2, W2
	MOV	W2, _spi2_state			;

	BRA	_spi2interrupt_done

_spi2interrupt_read_a_and_start_b_transfer:
	BSET 	LATC, #4			; PIN_AD7686_1_CS = 1

	MOV	_fwd_det_a_pointer, W0		; Next Three lines W0 = Address of forward_power_detector_A[ad7686_result_index]  
	MOV	_ad7686_result_index, W1	; 
	ADD	W0, W1, W0			;
	ADD	W0, W1, W0			;

	MOV	SPI2BUF, W2			; forward_power_detector_B[ad7686_result_index] = SPI2BUF
	MOV	W2, [W0]			; 

	BCLR	LATC, #3			; PIN_AD7686_2_CS = 0;

	MOV	W2, SPI2BUF 			; Dummy write to SPI2B

	MOV	#0, W2
	MOV	W2, _spi2_state			;

_spi2interrupt_done:	

	bclr	LATB, #12 			; Set PIN_TEST_POINT_27  DPARKER remove this line for production system
	ulnk
	POP	SR
	POP	CORCON
	pop.s
	retfie
	














