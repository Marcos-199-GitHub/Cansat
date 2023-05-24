
_main:

;Segundo_Round.c,1 :: 		void main() {
;Segundo_Round.c,3 :: 		UART1_Init(9600);
	BSF         BAUDCON+0, 3, 0
	MOVLW       1
	MOVWF       SPBRGH+0 
	MOVLW       160
	MOVWF       SPBRG+0 
	BSF         TXSTA+0, 2, 0
	CALL        _UART1_Init+0, 0
;Segundo_Round.c,4 :: 		while (1){
L_main0:
;Segundo_Round.c,5 :: 		UART1_Write_Text("Funciona");
	MOVLW       ?lstr1_Segundo_Round+0
	MOVWF       FARG_UART1_Write_Text_uart_text+0 
	MOVLW       hi_addr(?lstr1_Segundo_Round+0)
	MOVWF       FARG_UART1_Write_Text_uart_text+1 
	CALL        _UART1_Write_Text+0, 0
;Segundo_Round.c,6 :: 		}
	GOTO        L_main0
;Segundo_Round.c,8 :: 		}
L_end_main:
	GOTO        $+0
; end of _main
