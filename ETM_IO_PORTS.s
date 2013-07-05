.ifdef __dsPIC30F
        .include "p30fxxxx.inc"
.endif
.ifdef __dsPIC33F
        .include "p33Fxxxx.inc"
.endif

	
.ifdef __30F6014A
	.set __PORTA_ACTIVE, 1	
	.set __PORTB_ACTIVE, 1
	.set __PORTC_ACTIVE, 1
	.set __PORTD_ACTIVE, 1
	.set __PORTF_ACTIVE, 1
	.set __PORTG_ACTIVE, 1	
.endif



	
        .global  _ETMSetPin
        .text
_ETMSetPin:
	;; Port should be stored in W1 after function call
	;; Pin should be stored in W0 after function call
			
	        BRA            	W1
	
; _branchPortA:	
	.ifdef __PORTA_ACTIVE
		IOR             LATA		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN

; _branchPortB:
	.ifdef __PORTB_ACTIVE
		IOR             LATB		; Note that the Mask must be stored in W0
	.else
		NOP
	.endif
		RETURN
; _branchPortC:
	.ifdef __PORTC_ACTIVE
		IOR             LATC		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortD:
	.ifdef __PORTD_ACTIVE
		IOR             LATD		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortE:
	.ifdef __PORTE_ACTIVE
		IOR             LATE		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN
; _branchPortF:
	.ifdef __PORTF_ACTIVE
		IOR             LATF		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortG:
	.ifdef __PORTG_ACTIVE
		IOR             LATG		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
	
; _branchPortNone:
		NOP
		RETURN

		
        .global  _ETMClearPin
        .text
_ETMClearPin:
	;; Port should be stored in W1 after function call
	;; Pin should be stored in W0 after function call


		COM		W0,W0
	        BRA            	W1
	
; _branchPortA:	
	.ifdef __PORTA_ACTIVE
		AND             LATA		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN

; _branchPortB:
	.ifdef __PORTB_ACTIVE
		AND             LATB		; Note that the Mask must be stored in W0
	.else
		NOP
	.endif
		RETURN
; _branchPortC:
	.ifdef __PORTC_ACTIVE
		AND             LATC		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortD:
	.ifdef __PORTD_ACTIVE
		AND             LATD		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortE:
	.ifdef __PORTE_ACTIVE
		AND             LATE		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN
; _branchPortF:
	.ifdef __PORTF_ACTIVE
		AND             LATF		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortG:
	.ifdef __PORTG_ACTIVE
		AND             LATG		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN

; _branchPortNone:
		NOP
		RETURN






	
	
        .global  _ETMPinTrisInput
        .text
_ETMPinTrisInput:
	;; Port should be stored in W1 after function call
	;; Pin should be stored in W0 after function call
			
	        BRA            	W1
	
; _branchPortA:	
	.ifdef __PORTA_ACTIVE
		IOR             TRISA		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN

; _branchPortB:
	.ifdef __PORTB_ACTIVE
		IOR             TRISB		; Note that the Mask must be stored in W0
	.else
		NOP
	.endif
		RETURN
; _branchPortC:
	.ifdef __PORTC_ACTIVE
		IOR             TRISC		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortD:
	.ifdef __PORTD_ACTIVE
		IOR             TRISD		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortE:
	.ifdef __PORTE_ACTIVE
		IOR             TRISE		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN
; _branchPortF:
	.ifdef __PORTF_ACTIVE
		IOR             TRISF		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortG:
	.ifdef __PORTG_ACTIVE
		IOR             TRISG		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN

; _branchPortNone:
		NOP
		RETURN

		
        .global  _ETMPinTrisOutput
        .text
_ETMPinTrisOutput:
	;; Port should be stored in W1 after function call
	;; Pin should be stored in W0 after function call


		COM		W0,W0
	        BRA            	W1
	
; _branchPortA:	
	.ifdef __PORTA_ACTIVE
		AND             TRISA		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN

; _branchPortB:
	.ifdef __PORTB_ACTIVE
		AND             TRISB		; Note that the Mask must be stored in W0
	.else
		NOP
	.endif
		RETURN
; _branchPortC:
	.ifdef __PORTC_ACTIVE
		AND             TRISC		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortD:
	.ifdef __PORTD_ACTIVE
		AND             TRISD		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortE:
	.ifdef __PORTE_ACTIVE
		AND             TRISE		; Note that the Mask must be stored in W0	
	.else	
		NOP
	.endif
		RETURN
; _branchPortF:
	.ifdef __PORTF_ACTIVE
		AND             TRISF		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN
; _branchPortG:
	.ifdef __PORTG_ACTIVE
		AND             TRISG		; Note that the Mask must be stored in W0	
	.else
		NOP
	.endif
		RETURN

; _branchPortNone:
		NOP
		RETURN






;--------End of All Code Sections ---------------------------------------------
        .end                               ;End of program code in this file



