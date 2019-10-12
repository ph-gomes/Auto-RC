int entrada=0;
const int tamanhoDado=4; //Tamanho do dado é o numero de bits que representam um dado.
                         //No nosso caso 4 bits para cada dado (0000,0011,1100,1111).

int tempo=2;             //Tempo/ frequencia de captura de dados, lembrando que o tempo de recepção tem que ser igual ao
                         //tempo de emissão dos dados.
int Sensor;              //Variavel que armazena o valor em decimal recebido pelo sensor
//media movel
int media=520;           //Variavel que armazenará a media gaussiana do sinal.
int contMedia=0;         //Variavel que contem quantos dados foram recebidos (para calcular a media).
int soma=0;              //Variavel que armazena a soma para que seja possivel o calculo da media.

//velocidade
float velo=0;            //Variavel teste que armazena a velocidade de transmissão dos dados
long int DT = millis();  //Variação de tempo para o calculo de velocidade
//Variaveis referentes a sequencia de dado
bool sequencia[tamanhoDado];//Vetor que armazena a sequencia lida.
bool sequenciaAnterior[tamanhoDado];//Vetor que armazena a sequencia anterior lida (para evitar uso desnecessário de processamento).
bool dif = true;        //Variavel que armazena se a sequencia anterior é diferente ou não da atual.
bool identificador;     //Variavel que armazena true quando um identificador é encontrado.
//maxmin
long int tempoMaxMin=millis();
int Maximo=0;
int Minimo=70;

//LOOP
/*
                identificador = procuraIdentificador();
                if(identificador)
                {
                  Sequencia();
                  if(verificaSequencia())
                  {

                    for(int i=0;i<tamanhoDado;i++)
                    {
                      sequenciaAnterior[i] = sequencia[i];

                    }
                    velo = velocidade();

                  }
                }
                MaxMin();
                //Aqui temos a sequencia e todos os dados para que seja possivel a decodificação.
                  Serial.print("Seq:");
                  for(int i=0;i<tamanhoDado;i++)
                    Serial.print(sequenciaAnterior[i]);
                  Serial.print("\tVel:");
                  Serial.print(velo,4);
                  Serial.print("\tMIN:");
                  Serial.print(Minimo);
                  Serial.print("\tMax:");
                  Serial.print(Maximo);
                  Serial.print("\tMMV:");
                  Serial.println(media);

*/
//Função de recepção (Unica que deve obrigatoriamente estar no raspberry).
bool sensor() //Função que recebe os dados do sensor (A SER ALTERADA).
{
  bool resultado;

    //Sensor = analogRead(entrada);//Aqui vamos ler o sensor para que seja possivel executar os calculos

    mediaMovel(Sensor);
    resultado = nivelLogico(Sensor);
    delay(tempo);
    return resultado;
}
//Funções de calculo de resultados
void MaxMin() //função que calcula o maximo e o minimo recebidos pelos sensores
{
  if(Maximo<Sensor)
  {
    Maximo=Sensor;
  }
  if(Minimo>Sensor)
  {
    Minimo = Sensor;
  }
  if(millis() - tempoMaxMin>100)
  {
    Maximo--;
    Minimo++;
    tempoMaxMin=millis();
  }
}
//calculo de velocidade de transmissão
float velocidade()
{
  float resultado = 1.4f;
  long int dt = (millis()-DT);
  int ds = 1;
  resultado = 1000.0*(float)ds/(float)dt;

  DT=millis();
  return resultado;
}
//verifica se a sequencia é boa
bool verificaSequencia()
{
  int contNB=0;
  int contNA=0;
  bool control=true;
  bool resultado=true;
  for(int i=0;i<tamanhoDado;i++)
  {
    if(sequencia[i])
    {
      contNA++; //Nivel alto e nivel baixo (1 e 0) devem ter sempre numeros pares dentro da sequencia (0, 2 ou 4).
    }
    else
    {
      contNB++;
    }
    if(i%2==0)
    {
      if(sequencia[i]!=sequencia[i+1]) //Nesse caso, a posição 0 deve sempre ser igual a posição 1 e
      {                                //a posição 2 deve sempre ser igual a posição 3.
        resultado=false;
      }
    }
  }
  if(contNB%2==1||contNA%2==1||diferenca()==false) // verifica se existe quantidades impares de 0 ou 1 e
  {                                                //se a diferença entre eles e a media é de mais de 5(valor decimal recebido do sensor).
    resultado=false;
  }
  return resultado;
}
//Inicio
bool procuraIdentificador()                         //Função que fica a todo momento procurando o identificador
{
  int resultado;
  if(sensor()==0&&sensor()==1&&sensor()==0&&sensor()==1)//Identificador no caso = 0101
  {
    resultado=true;
  }
  else
  {
    resultado=false;
  }

  return resultado;


}
void Sequencia()    //Função que captura a sequencia após o encontro do identificador
{
  for(int i=0;i<tamanhoDado;i++)
  {
    sequencia[i] = sensor();

  }
}
bool diferenca()    //Função que calcula a diferença entre o valor recebido e a media.
{                   //Isso filtra "lixos" quando o receptor não está diante do emissor.
  bool resultado = true;
  if(abs(Sensor-media)<5)
  {
    resultado=false;
  }
  return resultado;
}

bool nivelLogico(int valor) // função que transforma os valores decimais recebidos pelo sensor em uma codificação
{                           // binaria.
  bool resultado;
  if(valor>media)
  {
    resultado=true;
  }
  else
  {
    resultado=false;
  }
  return resultado;
}
int mediaMovel(int valor)    //Calculo da media movel.
{
  if(contMedia<20)
  {
    soma += valor;
    contMedia++;
  }
  else
  {
    media = soma/20;
    soma=0;
    contMedia=0;
  }
}
