void main() {

UART1_Init(9600);
while (1){
UART1_Write_Text("Funciona");
}

}