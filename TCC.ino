#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include "Platform.h"
#include "SoftwareSerial.h"
#ifndef CDC_ENABLED
// Jumper do shield na posição SW
SoftwareSerial port(12,13);
#else
// Jumper do shield na posição HW (Para Arduino Leonardo)
#define port Serial1
#endif
#else // Arduino 0022 - use modified NewSoftSerial
#include "WProgram.h"
#include "NewSoftSerial.h"
NewSoftSerial port(12,13);
#endif

#include "EasyVR.h"

#include <Servo.h> //incluindo biblioteca para servo motor

Servo porta;  //nomeando o servo como variavel "porta".

EasyVR easyvr(port);

//*****************************************************************
#define banheiro 6             // Definindo os pinos a cada elemento
#define sala 7
#define externa 8
#define ventilador 9
//*****************************************************************

//Grupos e comandos
enum Groups               //*Enumerando os grupos
{                         
  GROUP_1  = 1,           
  GROUP_2  = 2,           
  GROUP_16 = 16,
};

enum Group1 
{                         //*Aqui são enumerados os comandos gravados
  G1_AUTOMACAO = 0,     // dentro de cada grupo, para facilitar referênciamento.*
  G1_AUTOMACAO2 = 1,
};

enum Group2 
{
  G2_PORTA = 0,
  G2_BANHEIRO = 1,
  G2_SALA = 2,
  G2_VENTILADOR = 3,
  G2_EXTERNAS = 4,
  G2_CONCLUIR_TRABALHO = 5,
  G2_PORTA2 = 6,
  G2_BANHEIRO2 = 7,
  G2_SALA2 = 8,
  G2_VENTILADOR2 = 9,
  G2_EXTERNAS2 = 10,
  G2_CONCLUIR_TRABALHO2 = 11,
};

enum Group16
{
  G16_ENGENHARIA = 0,
  G16_ENGENHARIA2 = 1,
};


EasyVRBridge bridge;

int8_t group, idx;

//variaveis criadas para acionamento e desacionamento.
int b; //banheiro
int s; //sala
int v; //ventilador
int e; //externas

void setup()     //função setup.
{
  porta.attach(3);                 //Definindo pino 3 ao servo "porta"
  pinMode (ventilador, OUTPUT);    // Setando os pinos referentes aos 
  pinMode (banheiro, OUTPUT);        // elementos como saída.
  pinMode (sala, OUTPUT);
  pinMode (externa, OUTPUT);

  porta.write(25);          //definindo valores iniciais.
  digitalWrite(sala, HIGH);
  digitalWrite(banheiro, HIGH);

#ifndef CDC_ENABLED
  // bridge mode?
  if (bridge.check())
  {
    cli();
    bridge.loop(0, 1, 12, 13);
  }
  // run normally
  Serial.begin(9600);
  Serial.println("Bridge not started!");
#else
  // bridge mode?
  if (bridge.check())
  {
    port.begin(9600);
    bridge.loop(port);
  }
  Serial.println("Bridge connection aborted!");
#endif
  port.begin(9600);

  while (!easyvr.detect())
  {
    Serial.println("EasyVR não detectado!");
    delay(1000);
  }

  easyvr.setPinOutput(EasyVR::IO1, LOW);
  Serial.println("EasyVR detectado!");
  easyvr.setTimeout(5);
  easyvr.setLanguage(0);   //Aqui pode se setar a linguagem.

  group = GROUP_1; // Este comando seta em qual grupo de comandos ira começar
}

void action(); 

void loop()   //função loop, esta ficara sempre lendo os dados e executando
{             //  ações caso seja acionada algum comando.
  easyvr.setPinOutput(EasyVR::IO1, HIGH); //Led do EasyVR ligado (mostra que está lendo o som)

  Serial.print("Diga um comando no grupo ");
  Serial.println(group);  //Imprime no monitor serial em qual grupo está.
  easyvr.recognizeCommand(group);

  do
  {
    // Pode se fazer algum processo enquanto espera um comando. Foi deixado vazio
  }
  while (!easyvr.hasFinished());

  easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
  if (idx >= 0)
  {
    // built-in trigger (ROBOT)
    // group = GROUP_X; <-- jump to another group X
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // print debug message
    uint8_t train = 0;
    char name[32];
    Serial.print("Command: ");
    Serial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train))
    {
      Serial.print(" = ");
      Serial.println(name);
    }
    else
      Serial.println();
    easyvr.playSound(0, EasyVR::VOL_FULL); //aciona o beep. 
    // perform some action
    action();    // Aqui se chama a função action, onde executará caso a voz reconhecida
  }
  else  // caso erros ou tempo de espera termine
  {
    if (easyvr.isTimeout())
      Serial.println("Tempo acabado, tente novamente...");
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      Serial.print("Error ");  //caso nao reconheça ou de algum erro ele imprime o numero do erro.
      Serial.println(err, HEX);
    }
  }
}

void action() //função action. Aqui se encadeia os "switch/case". Onde se decidirá os comandos 
{            // efetuados a cada reconhecimento de palavra. 
  switch (group)
  {
  case GROUP_1:
    switch (idx)
    {
    case G1_AUTOMACAO:
      easyvr.playSound(5, EasyVR::VOL_FULL);   // A casa será acionada e pedirá uma senha,
      Serial.println("Olá. Diga a senha");
      group = GROUP_16; //Pulará para o grupo 16 onde está a senha.
      break;
          
    case G1_AUTOMACAO2:
      easyvr.playSound(5, EasyVR::VOL_FULL);   // A casa será acionada e pedirá uma senha,
      Serial.println("Olá. Diga a senha-");
      group = GROUP_16; //Pulará para o grupo 16 onde está a senha.
      break;
    }
    break;

  case GROUP_2:
    switch (idx)
    {
    case G2_PORTA:    //caso reconheça o comando de acionamento da porta
      easyvr.playSound(1, EasyVR::VOL_FULL);   //executa o som gravado como resposta
      Serial.println("Abrindo Porta");      //imprime no monitor serial uma resposta
      porta.write(170);                      // comando para o servo ir para posição 170
      delay(5000);                          // tempo de espera ocioso.
      porta.write(25);                     // servo motor para posição 25
      break;

    case G2_PORTA2:    //caso reconheça o comando de acionamento da porta
      easyvr.playSound(1, EasyVR::VOL_FULL);   //executa o som gravado como resposta
      Serial.println("Abrindo Porta-");      //imprime no monitor serial uma resposta
      porta.write(170);                      // comando para o servo ir para posição 170
      delay(5000);                          // tempo de espera ocioso.
      porta.write(25);                     // servo motor para posição 25
      break;

    case G2_BANHEIRO:
      if (b == 0){     // se o banheiro estiver desligado
        easyvr.playSound(7, EasyVR::VOL_FULL);
        digitalWrite(banheiro, LOW);     // Desliga o relé, acionando a passagem de energia 110v da lampada.
        Serial.println("Luz do banheiro ligada");
        b = 1; //muda a variavel para 1.
      }
      else{
        easyvr.playSound(2, EasyVR::VOL_FULL);
        digitalWrite(banheiro, HIGH);     // Liga o relé, impedindo a passagem de energia 110v da lampada.
        Serial.println("Luz do banheiro desligada");          
        b = 0;
      }
      break;

    case G2_BANHEIRO2:
      if (b == 0){     // se o banheiro estiver desligado
        easyvr.playSound(7, EasyVR::VOL_FULL);
        digitalWrite(banheiro, LOW);     // Desliga o relé, acionando a passagem de energia 110v da lampada.
        Serial.println("Luz do banheiro ligada-");
        b = 1; //muda a variavel para 1.
      }
      else{
        easyvr.playSound(2, EasyVR::VOL_FULL);
        digitalWrite(banheiro, HIGH);     // Liga o relé, impedindo a passagem de energia 110v da lampada.
        Serial.println("Luz do banheiro desligada-");          
        b = 0;
      }
      break;

    case G2_SALA:
      if (s == 0){
        easyvr.playSound(8, EasyVR::VOL_FULL);
        digitalWrite(sala, LOW);        // Desliga o relé, acionando a passagem de energia 110v da lampada.
        Serial.println("Luz da Sala ligada");
        s = 1;
      }
      else{
        easyvr.playSound(3, EasyVR::VOL_FULL);
        digitalWrite(sala, HIGH);   // Liga o relé, impedindo a passagem de energia 110v da lampada.
        Serial.println("Luz da sala desligada");
        s = 0;
      }
      break;

    case G2_SALA2:
      if (s == 0){
        easyvr.playSound(8, EasyVR::VOL_FULL);
        digitalWrite(sala, LOW);        // Desliga o relé, acionando a passagem de energia 110v da lampada.
        Serial.println("Luz da Sala ligada-");
        s = 1;
      }
      else{
        easyvr.playSound(3, EasyVR::VOL_FULL);
        digitalWrite(sala, HIGH);   // Liga o relé, impedindo a passagem de energia 110v da lampada.
        Serial.println("Luz da sala desligada-");
        s = 0;
      }
      break;

    case G2_VENTILADOR:
      if (v == 0){
        easyvr.playSound(9, EasyVR::VOL_FULL);
        digitalWrite(ventilador, HIGH);      // Aciona a porta referente ao ventilador (cooler), fazendo com que ligue. 
        Serial.println("Ventilador ligado");
        v = 1;
      }
      else{
        easyvr.playSound(4, EasyVR::VOL_FULL);
        digitalWrite(ventilador, LOW);         // Desliga a passagem de energia na porta do cooler.
        Serial.println("Desligando Ventilador");
        v = 0;        
      }
      break;

    case G2_VENTILADOR2:
      if (v == 0){
        easyvr.playSound(9, EasyVR::VOL_FULL);
        digitalWrite(ventilador, HIGH);      // Aciona a porta referente ao ventilador (cooler), fazendo com que ligue. 
        Serial.println("Ventilador ligado-");
        v = 1;
      }
      else{
        easyvr.playSound(4, EasyVR::VOL_FULL);
        digitalWrite(ventilador, LOW);         // Desliga a passagem de energia na porta do cooler.
        Serial.println("Desligando Ventilador-");
        v = 0;        
      }
      break;

    case G2_EXTERNAS:
      if (e == 0){
        easyvr.playSound(11, EasyVR::VOL_FULL);
        digitalWrite(externa, HIGH);       // liga luzes externas (leds externos da maquete)
        Serial.println("Luzes externas ligadas");
        e = 1;
      }        
      else{
        easyvr.playSound(10, EasyVR::VOL_FULL);
        digitalWrite(externa, LOW);           //Desliga a porta dos leds externos
        Serial.println("Luzes externas desligadas");
        e = 0;
      }
      break;

    case G2_EXTERNAS2:
      if (e == 0){
        easyvr.playSound(11, EasyVR::VOL_FULL);
        digitalWrite(externa, HIGH);       // liga luzes externas (leds externos da maquete)
        Serial.println("Luzes externas ligadas-");
        e = 1;
      }        
      else{
        easyvr.playSound(10, EasyVR::VOL_FULL);
        digitalWrite(externa, LOW);           //Desliga a porta dos leds externos
        Serial.println("Luzes externas desligadas-");
        e = 0;
      }
      break;

    case G2_CONCLUIR_TRABALHO:         //Aqui é dito para sair dos comandos e voltar para o inicio
      easyvr.playSound(6, EasyVR::VOL_FULL);
      Serial.println("Modulo finalizado");
      for (int x=0; x<20; x++) {      //Comando para piscar os leds ao finalizar.
        digitalWrite (externa, HIGH);
        delay (100);
        digitalWrite (externa, LOW);
        delay (100);
      }
      e = 0;
      digitalWrite(sala, HIGH);
      s = 0;
      digitalWrite(banheiro, HIGH);
      b = 0;
      digitalWrite(ventilador, LOW);
      v = 0;
      group = GROUP_1;    // Volta para o grupo 1. 
      break;

    case G2_CONCLUIR_TRABALHO2:         //Aqui é dito para sair dos comandos e voltar para o inicio
      easyvr.playSound(6, EasyVR::VOL_FULL);
      Serial.println("Modulo finalizado-");
      for (int x=0; x<20; x++) {      //Comando para piscar os leds ao finalizar.
        digitalWrite (externa, HIGH);
        delay (100);
        digitalWrite (externa, LOW);
        delay (100);
      }
      e = 0;
      digitalWrite(sala, HIGH);
      s = 0;
      digitalWrite(banheiro, HIGH);
      b = 0;
      digitalWrite(ventilador, LOW);
      v = 0;
      group = GROUP_1;    // Volta para o grupo 1. 
      break;
    }
    break;

  case GROUP_16:
    switch (idx)
    {
    case G16_ENGENHARIA:
      easyvr.playSound(12, EasyVR::VOL_FULL);     //caso a senha seja dita.
      Serial.println("Senha confirmada");
      group = GROUP_2;      //muda para o grupo 2 de acionamento de equipamentos.
      break;
    case G16_ENGENHARIA2:
      easyvr.playSound(12, EasyVR::VOL_FULL);     //caso a senha seja dita.
      Serial.println("Senha confirmada");
      group = GROUP_2;      //muda para o grupo 2 de acionamento de equipamentos.
      break;
    }
    break;
  }
}








