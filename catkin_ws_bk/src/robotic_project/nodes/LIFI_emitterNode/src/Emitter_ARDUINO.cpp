//Sequencia de trabalho:
    //Recebe sequencia
    //envia para led a sequencia.
bool dadoToPrint[8];
const int saida = 8; //saida digital que se localiza o LED.
tempo = 2;
void setup() {

  pinMode(saida,OUTPUT);
  Serial.begin(38400); //A taxa de bits 9600 se demonstrou baixa em testes.
}
void loop()
{
        recebeDado();
        Print();
}
void recebeDado()
{

    //Recebemos dados e o colocamos no vetor dadoToPrint
}
void Print()
{
  for(int i=0;i<tamanho;i++)
  {
    Serial.print(dadoToPrint[i]);
    digitalWrite(saida,dadoToPrint[i]);
    delay(tempo);
  }
  Serial.print("\t");
  Serial.print(valor);
  Serial.println();
}
