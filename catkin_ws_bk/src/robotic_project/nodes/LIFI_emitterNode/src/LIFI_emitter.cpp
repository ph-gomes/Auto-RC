//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/10 - 18:40
//  filename:   LIFI_emitter.cpp
//
//  author:     Giovani Bernardes Vitor
//              Copyright DEG / UFLA
//
//  version:    $Id: $
//
//  purpose:
//
//////////////////////////////////////////////////////////////////////////////////
#include <LIFI_emitter/LIFI_emitter.h>
 bool dadoToPrint[8];
 bool binary[2];
 bool binaryToCode[4];
 int valor=0;

LIFI_emitter::LIFI_emitter(ros::NodeHandle *rosNode_): rosNode(rosNode_)
{

	ROS_INFO(" LIFI_emitter correctly configured and running!");
}

LIFI_emitter::~LIFI_emitter()
{
//Adiciona identificador (0101);
    //Coleta dado que será enviado;
    captaValor();
    //Converte dado para binario
    convertDadoToBinary();
    //Converde dado para codigo (o que é 0 vira 00 e o que é 1 vira 11).
    convertToCode();
    //Coloca codigo e identificador em uma sequencia
    colocaIdent();
    convertToPrint();
    //envia sequencia para arduino
    //O vetor dadoToPrint contem a sequencia que deverá ser enviada ao arduino
}
void LIFI_emitter::captaValor()
{

  //coleta o valor(0,1,2,3) e armazena na variavel valor.
  valor = 1;

}
void LIFI_emitter::convertDadoToBinary()
{
  int soma=valor;

  for(int i=1;i>=0;i--)
  {
    if(soma%2==0)
    {
      binary[i]=0;
      soma=soma/2;
    }
    else
    {
      binary[i]=1;
      soma=(soma-1)/2;
    }
  }
}
void LIFI_emitter::convertToCode()
{
  for(int i=0;i<2;i++)
  {
    if(binary[i]==0)
    {
      binaryToCode[2*i]=0;
      binaryToCode[2*i+1]=0;
    }
    else
    {
      binaryToCode[2*i]=1;
      binaryToCode[2*i+1]=1;
    }
  }
}
void LIFI_emitter::colocaIdent()
{
  dadoToPrint[0]=0;
  dadoToPrint[1]=1;
  dadoToPrint[2]=0;
  dadoToPrint[3]=1;
}
void LIFI_emitter::convertToPrint()
{
  for(int i=4;i<8;i++)
  {
    dadoToPrint[i]=binaryToCode[i-4];
  }
}

void LIFI_emitter::start_emitter()
{

	ROS_INFO(" Starting signal emitter...!");



}
void LIFI_emitter::codification_process()
{
  	ROS_INFO(" Performs signal codification ... " );

}


bool LIFI_emitter::send_info()
{

	ROS_INFO(" send_info ... ");



  	return true;
}





