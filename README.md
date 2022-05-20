# IoT-Mackenzie
Projeto de IoT - Mackenzie ADS - 5 Semestre
Grupo: Kil Hoo Kim, Thiago Gonçalves

Descrição:
Medição de temperatura em tempo real a cada 2 segundos. Caso a temperatura fique inferior a 40 graus Celsius, o relê é ativado e liga a lâmpada para aquecer, quando a temperatura atinge 65 graus Celsius o relê é ativado novamente desligando a lâmpada. Realizando o monitoramento da temperatura.
Os dados de temperatura são enviados via protocolo MQTT. Como o Arduino Uno R3 não possui rede embarcada, utilizamos na comunicação um broker local e as comunicações via porta serial.

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
