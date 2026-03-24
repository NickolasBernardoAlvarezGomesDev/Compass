# Compass

Documentacao consolidada do modulo de bussola embarcada baseado na implementacao de referencia localizada em `../Compass`.

## Sobre o projeto

Este projeto reune experimentos em MicroPython para leitura de heading a partir de um sensor MPU9250, com foco em:

- estimativa de orientacao por magnetometro;
- reducao de ruido com filtros de Kalman;
- transmissao serial do heading filtrado;
- exploracao de ajuste adaptativo com TinyML.

O material de referencia analisado inclui os scripts principais do diretorio `Compass`, o `README.md` original e o pacote `cowo.zip`, que contem a biblioteca `compass` e utilitarios de filtro.

## Objetivo

O objetivo do projeto e fornecer uma base embarcada para leitura, suavizacao e distribuicao do heading, permitindo comparar abordagens simples e experimentais para estabilizacao do angulo medido.

## Escopo tecnico

Pela implementacao atual, o sistema cobre quatro frentes:

- aquisicao de dados do MPU9250 via I2C;
- calibracao e correcao do magnetometro AK8963;
- filtragem do heading por Kalman escalar e vetorial;
- envio do angulo filtrado por UART para outro dispositivo.

## Arquitetura de referencia

### 1. Camada de sensores

O pacote `cowo.zip` concentra a base de drivers e wrappers do sensor:

- `compass/mpu9250.py`: integra acelerometro, giroscopio e magnetometro.
- `compass/mpu6500.py`: driver do acelerometro e giroscopio.
- `compass/ak8963.py`: driver do magnetometro com compensacao hard iron e soft iron.
- `compass/cowompaws.py`: wrapper de alto nivel com metodos como `get_head()`, `get_accel()`, `get_gyro()` e `get_temp()`.

### 2. Camada de filtragem

O mesmo pacote inclui componentes auxiliares para tratamento do sinal:

- `filters/kalman.py`: implementacao de referencia de um filtro de Kalman 1D.
- `filters/spike.py`: tentativa de filtro para rejeicao de picos.
- `compass/testing/kaltest.py`: script de teste para filtragem de heading.

### 3. Aplicacoes de referencia

Os scripts do diretorio `Compass` demonstram abordagens diferentes sobre o mesmo problema:

- `bussola_final.py`: filtro vetorial em seno e cosseno para evitar descontinuidades angulares.
- `maxdev.py`: Kalman escalar com rejeicao de leituras muito distantes do valor anterior.
- `trancep.py`: transmissao do heading filtrado via UART e exemplo de recepcao serial.
- `TinyML KF.py`: versao experimental com ajuste adaptativo do erro de medida via TinyGRU.

## Estrutura funcional dos arquivos

| Arquivo | Papel no projeto |
| --- | --- |
| `../Compass/README.md` | Descricao inicial e nome historico do modulo (`cowompaws`). |
| `../Compass/bussola_final.py` | Pipeline principal para heading filtrado com Kalman vetorial. |
| `../Compass/maxdev.py` | Variante com rejeicao de outliers antes da filtragem. |
| `../Compass/trancep.py` | Exemplo de integracao serial entre transmissor e receptor. |
| `../Compass/TinyML KF.py` | Prototipo de fusao entre Kalman e TinyML embarcado. |
| `../Compass/cowo.zip` | Pacote com drivers `compass`, filtros e script de teste. |

## Ambiente esperado

A implementacao e especifica para MicroPython e depende de modulos como `machine`, `utime` e `SoftI2C`. Pelos pinos definidos nos scripts de referencia, a configuracao assume uma placa compativel com ESP32 ou equivalente.

Configuracoes observadas nos exemplos:

- I2C em `SCL=22` e `SDA=21`;
- UART1 em `TX=17` e `RX=16`;
- baud rate de `115200` para transmissao serial;
- loop de amostragem com `sleep` de 50 ms.

## Como o projeto funciona

### Leitura do sensor

O wrapper `cowompaws.py` inicializa o `MPU9250`, faz a leitura do magnetometro e aplica uma suavizacao exponencial nos eixos magneticos antes de calcular o heading em radianos.

### Calculo do heading

O heading e obtido com `atan2` a partir dos eixos magneticos filtrados e depois normalizado para o intervalo `[0, 2pi)`.

### Filtragem

Ha duas estrategias principais nos exemplos:

- Kalman escalar sobre o heading bruto;
- Kalman vetorial sobre `sin(theta)` e `cos(theta)`, com reconstrucao posterior do angulo.

A abordagem vetorial, usada em `bussola_final.py` e `trancep.py`, e a mais robusta do conjunto porque reduz problemas de salto angular proximos de `0` e `360` graus.

### Comunicacao serial

No fluxo de `trancep.py`, o heading filtrado e convertido para graus e transmitido por UART como uma linha de texto, o que simplifica a integracao com outro microcontrolador, interface serial ou ferramenta de monitoramento.

## Configuracoes que exigem atencao

Antes de usar o projeto em campo, alguns parametros precisam ser revisados:

- Declination magnetica: `cowompaws.py` aplica um valor fixo de `35` graus em radianos.
- Calibracao do magnetometro: `ak8963.py` possui `offset` e `scale` hardcoded.
- Pinos de hardware: os scripts assumem um mapeamento especifico de I2C e UART.
- Parametros do Kalman: os valores de erro inicial e erro de medida variam entre os scripts.

Esses pontos sao criticos porque afetam diretamente a precisao do heading e a portabilidade da implementacao.

## Estado da implementacao

O projeto esta em estagio de prototipacao aplicada. A base principal de sensores e leitura esta organizada, mas os exemplos ainda representam exploracoes tecnicas de filtragem e comunicacao, nao um firmware unico e consolidado.

Dois pontos merecem destaque:

- `TinyML KF.py` e experimental e usa pesos placeholder na `TinyGRU`, portanto nao representa um modelo treinado pronto para producao.
- `filters/spike.py` aparenta estar incompleto e nao deve ser tratado como filtro validado sem revisao adicional.

## Fluxo recomendado de uso

1. Descompacte o conteudo de `../Compass/cowo.zip`.
2. Copie os diretorios `compass/` e `filters/` para o filesystem da placa com MicroPython.
3. Ajuste a declinacao magnetica, os offsets do magnetometro e os pinos conforme o hardware real.
4. Escolha um script de entrada conforme o objetivo:
   `bussola_final.py` para heading filtrado,
   `maxdev.py` para testes com rejeicao de desvio,
   `trancep.py` para UART,
   `TinyML KF.py` para pesquisa experimental.
5. Execute o script selecionado e acompanhe a saida serial para validar o comportamento.

## Direcao recomendada para evolucao

Para transformar a base atual em um modulo mais solido, os proximos passos mais naturais sao:

- centralizar a configuracao de pinos, declinacao e calibracao;
- separar scripts de demonstracao de codigo reutilizavel;
- revisar e validar o filtro de rejeicao de picos;
- documentar o processo de calibracao do magnetometro;
- substituir os pesos placeholder da TinyGRU por um pipeline de treinamento reproduzivel.

## Resumo

`Compass` e uma base embarcada promissora para leitura de heading com MPU9250 em MicroPython. O projeto ja demonstra aquisicao de dados, filtragem angular, transmissao serial e experimentacao com adaptacao de filtro, mas ainda depende de consolidacao de configuracao, calibracao e organizacao para se tornar um modulo pronto para reutilizacao.
