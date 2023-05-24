
_Soft_UART_Write_Text:

;MyProject.c,26 :: 		void Soft_UART_Write_Text(char* udata){
;MyProject.c,27 :: 		int i=0;
	CLRF        Soft_UART_Write_Text_i_L0+0 
	CLRF        Soft_UART_Write_Text_i_L0+1 
;MyProject.c,28 :: 		do{
L_Soft_UART_Write_Text0:
;MyProject.c,30 :: 		i++;
	INFSNZ      Soft_UART_Write_Text_i_L0+0, 1 
	INCF        Soft_UART_Write_Text_i_L0+1, 1 
;MyProject.c,32 :: 		while (udata[i] != 0) ;
	MOVF        Soft_UART_Write_Text_i_L0+0, 0 
	ADDWF       FARG_Soft_UART_Write_Text_udata+0, 0 
	MOVWF       FSR0L+0 
	MOVF        Soft_UART_Write_Text_i_L0+1, 0 
	ADDWFC      FARG_Soft_UART_Write_Text_udata+1, 0 
	MOVWF       FSR0L+1 
	MOVF        POSTINC0+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_Soft_UART_Write_Text0
;MyProject.c,33 :: 		}
L_end_Soft_UART_Write_Text:
	RETURN      0
; end of _Soft_UART_Write_Text

_main:

;MyProject.c,36 :: 		void main() {
;MyProject.c,38 :: 		unsigned char udt = 55;
	MOVLW       55
	MOVWF       main_udt_L0+0 
;MyProject.c,42 :: 		UART1_Init(9600);
	BSF         BAUDCON+0, 3, 0
	MOVLW       1
	MOVWF       SPBRGH+0 
	MOVLW       160
	MOVWF       SPBRG+0 
	BSF         TXSTA+0, 2, 0
	CALL        _UART1_Init+0, 0
;MyProject.c,43 :: 		if (error > 0) {
	MOVF        main_error_L0+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_main3
;MyProject.c,44 :: 		while(1) ;                            // Stop program
L_main4:
	GOTO        L_main4
;MyProject.c,45 :: 		}
L_main3:
;MyProject.c,47 :: 		UART1_Write_Text("Serial correcto\n");
	MOVLW       ?lstr1_MyProject+0
	MOVWF       FARG_UART1_Write_Text_uart_text+0 
	MOVLW       hi_addr(?lstr1_MyProject+0)
	MOVWF       FARG_UART1_Write_Text_uart_text+1 
	CALL        _UART1_Write_Text+0, 0
;MyProject.c,48 :: 		Chip_Select = 1;                       // Deselect DAC
	BSF         RA4_bit+0, BitPos(RA4_bit+0) 
;MyProject.c,49 :: 		Chip_Select_Direction = 0;             // Set CS# pin as Output
	BCF         TRISA4_bit+0, BitPos(TRISA4_bit+0) 
;MyProject.c,50 :: 		Correct_Direction = 0;
	BCF         TRISD7_bit+0, BitPos(TRISD7_bit+0) 
;MyProject.c,51 :: 		Correct = 1;
	BSF         RD7_bit+0, BitPos(RD7_bit+0) 
;MyProject.c,52 :: 		Chip_Select = 1;
	BSF         RA4_bit+0, BitPos(RA4_bit+0) 
;MyProject.c,58 :: 		SPI1_Init();
	CALL        _SPI1_Init+0, 0
;MyProject.c,61 :: 		UART1_Write_Text("SPI iniciado\n");
	MOVLW       ?lstr2_MyProject+0
	MOVWF       FARG_UART1_Write_Text_uart_text+0 
	MOVLW       hi_addr(?lstr2_MyProject+0)
	MOVWF       FARG_UART1_Write_Text_uart_text+1 
	CALL        _UART1_Write_Text+0, 0
;MyProject.c,62 :: 		while (1){
L_main6:
;MyProject.c,64 :: 		Chip_Select = 0;
	BCF         RA4_bit+0, BitPos(RA4_bit+0) 
;MyProject.c,66 :: 		SPI1_Write(0b01011010);
	MOVLW       90
	MOVWF       FARG_SPI1_Write_data_+0 
	CALL        _SPI1_Write+0, 0
;MyProject.c,68 :: 		Delay_ms (10);
	MOVLW       52
	MOVWF       R12, 0
	MOVLW       241
	MOVWF       R13, 0
L_main8:
	DECFSZ      R13, 1, 1
	BRA         L_main8
	DECFSZ      R12, 1, 1
	BRA         L_main8
	NOP
	NOP
;MyProject.c,71 :: 		udt = SPI1_Read(0b01011010);
	MOVLW       90
	MOVWF       FARG_SPI1_Read_buffer+0 
	CALL        _SPI1_Read+0, 0
	MOVF        R0, 0 
	MOVWF       main_udt_L0+0 
;MyProject.c,72 :: 		Delay_ms (500);
	MOVLW       11
	MOVWF       R11, 0
	MOVLW       38
	MOVWF       R12, 0
	MOVLW       93
	MOVWF       R13, 0
L_main9:
	DECFSZ      R13, 1, 1
	BRA         L_main9
	DECFSZ      R12, 1, 1
	BRA         L_main9
	DECFSZ      R11, 1, 1
	BRA         L_main9
	NOP
	NOP
;MyProject.c,73 :: 		Chip_Select = 1;
	BSF         RA4_bit+0, BitPos(RA4_bit+0) 
;MyProject.c,74 :: 		Delay_ms (500);
	MOVLW       11
	MOVWF       R11, 0
	MOVLW       38
	MOVWF       R12, 0
	MOVLW       93
	MOVWF       R13, 0
L_main10:
	DECFSZ      R13, 1, 1
	BRA         L_main10
	DECFSZ      R12, 1, 1
	BRA         L_main10
	DECFSZ      R11, 1, 1
	BRA         L_main10
	NOP
	NOP
;MyProject.c,75 :: 		if (udt != 0b01010101){
	MOVF        main_udt_L0+0, 0 
	XORLW       85
	BTFSC       STATUS+0, 2 
	GOTO        L_main11
;MyProject.c,76 :: 		IntToStr(udt,str)   ;
	MOVF        main_udt_L0+0, 0 
	MOVWF       FARG_IntToStr_input+0 
	MOVLW       0
	MOVWF       FARG_IntToStr_input+1 
	MOVLW       main_str_L0+0
	MOVWF       FARG_IntToStr_output+0 
	MOVLW       hi_addr(main_str_L0+0)
	MOVWF       FARG_IntToStr_output+1 
	CALL        _IntToStr+0, 0
;MyProject.c,79 :: 		UART1_Write_Text("Dato recibido\n");
	MOVLW       ?lstr3_MyProject+0
	MOVWF       FARG_UART1_Write_Text_uart_text+0 
	MOVLW       hi_addr(?lstr3_MyProject+0)
	MOVWF       FARG_UART1_Write_Text_uart_text+1 
	CALL        _UART1_Write_Text+0, 0
;MyProject.c,80 :: 		Correct = 0;
	BCF         RD7_bit+0, BitPos(RD7_bit+0) 
;MyProject.c,82 :: 		UART1_Write(udt)  ;
	MOVF        main_udt_L0+0, 0 
	MOVWF       FARG_UART1_Write_data_+0 
	CALL        _UART1_Write+0, 0
;MyProject.c,84 :: 		UART1_Write_Text(str)  ;
	MOVLW       main_str_L0+0
	MOVWF       FARG_UART1_Write_Text_uart_text+0 
	MOVLW       hi_addr(main_str_L0+0)
	MOVWF       FARG_UART1_Write_Text_uart_text+1 
	CALL        _UART1_Write_Text+0, 0
;MyProject.c,87 :: 		UART1_Write('\n')  ;
	MOVLW       10
	MOVWF       FARG_UART1_Write_data_+0 
	CALL        _UART1_Write+0, 0
;MyProject.c,88 :: 		}
	GOTO        L_main12
L_main11:
;MyProject.c,90 :: 		Correct = 1;
	BSF         RD7_bit+0, BitPos(RD7_bit+0) 
;MyProject.c,92 :: 		UART1_Write_Text("RFM correcto\n");
	MOVLW       ?lstr4_MyProject+0
	MOVWF       FARG_UART1_Write_Text_uart_text+0 
	MOVLW       hi_addr(?lstr4_MyProject+0)
	MOVWF       FARG_UART1_Write_Text_uart_text+1 
	CALL        _UART1_Write_Text+0, 0
;MyProject.c,93 :: 		}
L_main12:
;MyProject.c,94 :: 		}
	GOTO        L_main6
;MyProject.c,95 :: 		}
L_end_main:
	GOTO        $+0
; end of _main
