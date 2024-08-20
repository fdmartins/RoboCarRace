# Construção de um Carro Autônomo - RoboCarRace
Veiculo Autonomo OpenSource

Este é o repositório oficial do RoboCar Race, o objetivo é disponibilizar dicas, circuitos e códigos para que você construa seu veículo autônomo em escala do zero.

* Conheça mais sobre a competição em:  
https://www.robocarrace.com.br/

* Veja os vídeos detalhados da construção do carro na playlist do YouTube: 
https://www.youtube.com/playlist?list=PLqjsxWOL0TM39jP3NmErrgpj0gdBl9_DG

### Divisão do projeto
Este projeto está separado por funcionalidades. 
- Parte 1: Codigo para comunicar com multiplos sensores de ultrassom genérico HC-SR04
- Parte 2: Codigo para controlar um servo motor, geralmente usado para o controle de direção em carros de radio controle.
- Parte 3: Codigo para controlar uma controladora ESC. O ESC é um dispositivo usado para controlar motores do tipo brushless (sem escovas) de 3 fios.
- Parte 4: Este código serve para coletar dados da pista. É feito um video e um arquivo csv. Cada linha o CSV é um frame e a posição da direção, velocidade e leituras de distancia do ultrassom.
- Parte 5: Este codigo é uma versão funcional que usa segmentação simples da pista, roda apenas em nivel de CPU, portanto, em qualquer plataforma como Raspberry Pi.

### Problemas de leitura/ruídos com o sensor de ultrassom?
veja este vídeo: https://www.youtube.com/watch?v=t73wGzd_4aE

---
### Custos da Plataforma básica:
- Chassis Usado Escala 1/8 (com servo e sem motor) : R$ 600,00
- Motor Brushless 3670 2150KV Genérico: R$ 109,12
- ESC 120A Genérico: R$ 87,15
- Sensor de Ultrassom HC-SR04: R$ 15,00 (cada unidade)
- Step Down Ajustável Dc/dc 5A C/ Display Xl4015: R$ 30,00
- Raspberry Pi PICO RP2040: R$ 59,00
- Conversor de Nível Lógico Bidirecional de 4 canais: R$ 8,00
- Rádio Controle + Receptor FlySky FSGT3B (opcional, mas útil para treinamento e testes): R$ 400,00
- Bateria Lipo 2500mah 3s 11.1v 40c:  R$ 190,00

Total da plataforma: R$ 1.528,27
Total sem rádio controle: R$ 1.128,27

## Custos com Placa de controle
- Nvidia® Jetson Nano 2gb Developer Kit Versão s/ Wifi: R$ 850,00

Custo Total Completo: R$ 2.378,27
