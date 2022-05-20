# IoT-Mackenzie
Projeto de IoT - Mackenzie ADS - 5 Semestre
Grupo: Kil Hoo Kim, Thiago Gonçalves

Descrição:
Medição de temperatura em tempo real a cada 2 segundos. Caso a temperatura fique inferior a 40 graus Celsius, o relê é ativado e liga a lâmpada para aquecer, quando a temperatura atinge 65 graus Celsius o relê é ativado novamente desligando a lâmpada. Realizando o monitoramento da temperatura.
Os dados de temperatura são enviados via protocolo MQTT. Como o Arduino Uno R3 não possui rede embarcada, utilizamos na comunicação um broker local e as comunicações via porta serial.

Comunicação MQTT:
Foi criado um Broker em rede local utilizando o MQTTBox de endereço: mqtt://localhost:1883.
No MQTTBox, criamos um tópico de nome: topic/temp; QoS=0; Payload Type=Strings e realizamos o subscribe no tópico.
No Node Red, é necessário importar os pallete: node-red-contrib-aedes; node-red-node-arduino respectivamente.
Pallete node-red-contrib-aedes: Cria a conexão com o nosso broker local através da porta 1883;
Pallete node-red-node-arduino: Cria as interfaces de comunicação dos PINS do Arduino.

Criamos a interface de entrada de dados através do PIN analógico 0 do Arduino no Node Red, enviamos ao tópico do broker um payload com o data recebido do sensor de temperatura.
<img width="1440" alt="image" src="https://user-images.githubusercontent.com/62429714/169618537-c3c2dd19-d7ea-4461-9531-77abb73722db.png">
Debug do payload recebido.

Hardware: 
1. Arduino UNO R3
2. Protoboard
3. Resistor 220ohms
4. Resistor 5Kohms
5. Resistor 10Kohms
6. Transistor NPN
7. Display LCD 16x2
8. Sensor de Temperatura: DHT11
9. Rele DPDT 5V

Plataformas de desenvolvimento:
1. Arduino.cc - Software Open Source para desenvolvimento e deploy do código para o Arduino, disponível em https://arduino.cc
2. Node Red - Ferramenta de desenvolvimento baseada em fluxo para programação visual para conectar dispositivos de hardware, APIs e serviços online como parte da Internet das Coisas.
3. MQTTBox - Programa para criar e testar a conectividade através do protocolo MQTT.

![image](https://user-images.githubusercontent.com/62429714/169617577-1b74bc62-a49b-4642-a5a2-f0a452c917f9.png)
Visão esquemática do projeto.
