#line 1 "I:/CanSat/Nueva carpeta/RFM69/Receptor/New/Segundo_Round.c"
void main() {

UART1_Init(9600);
while (1){
UART1_Write_Text("Funciona");
}

}
