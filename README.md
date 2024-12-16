# Projeto de Estufa Automatizada

Este projeto consiste na criação de uma estufa inteligente para controle e monitoramento de temperatura, umidade e iluminação, utilizando o microcontrolador NUCLEO-F446RE e sensores específicos. Desenvolvido como parte da capacitação em Sistemas Embarcados do Virtus CC, o projeto integra conceitos de hardware, software e documentação.

---

## Requisitos do Projeto
 - Montagem Pré-definida: A montagem fornecida pelo professor inclui os seguintes componentes:

Microcontrolador:

  - NUCLEO-F446RE
  - Dispositivos de Saída:
  - Display I2C SSD1306
  - Servo motor
  - Relé
  - Buzzer
  - Porta USART (Terminal)
  - LED onboard (NUCLEO-F446RE)
  
Dispositivos de Entrada:

  - Botões
  - Acelerômetro: MPU6050
  - Sensor de Pressão e Temperatura: BMP280
  - Sensor de Distância Ultrassônico: HC-SR04

Aqui está a explicação detalhada de cada parte do código, incluindo o **que cada função faz** e os conceitos envolvidos.

---

## **Definição de Estruturas**

### **1. Estrutura `MPU6050_t`**
A estrutura `MPU6050_t` armazena os dados relacionados ao **MPU6050** (sensor de aceleração, giroscópio e temperatura):

- **Dados do Acelerômetro (RAW e Processados)**:
  - `int16_t Accel_X_RAW`, `Accel_Y_RAW`, `Accel_Z_RAW`: Valores brutos do acelerômetro nos eixos X, Y e Z.
  - `double Ax`, `Ay`, `Az`: Valores processados (convertidos para unidades físicas como "g").

- **Dados do Giroscópio (RAW e Processados)**:
  - `int16_t Gyro_X_RAW`, `Gyro_Y_RAW`, `Gyro_Z_RAW`: Valores brutos do giroscópio nos eixos X, Y e Z.
  - `double Gx`, `Gy`, `Gz`: Valores processados (convertidos para graus por segundo).

- **Temperatura**:
  - `float Temperature`: Valor da temperatura medida pelo sensor.

- **Ângulos Filtrados (Kalman)**:
  - `double KalmanAngleX`, `KalmanAngleY`: Ângulos estimados pelo filtro de Kalman para os eixos X e Y.

---

### **2. Estrutura `Kalman_t`**
A estrutura `Kalman_t` armazena os dados necessários para o **Filtro de Kalman**:

- **Parâmetros do Filtro**:
  - `double Q_angle`: Variância do ruído do processo (erro esperado do ângulo).
  - `double Q_bias`: Variância do ruído do bias (erro esperado do giroscópio).
  - `double R_measure`: Variância do ruído da medição.

- **Estado Atual do Sistema**:
  - `double angle`: Ângulo estimado pelo filtro.
  - `double bias`: Bias estimado (erro de desvio do giroscópio).

- **Matriz de Covariância**:
  - `double P[2][2]`: Matriz para rastrear o erro de estimativa.

---

## **Funções do Código**

### **1. `uint8_t MPU6050_Init()`**
- **Objetivo**: Inicializa o sensor MPU6050 configurando os registros necessários.
- **Retorno**: `uint8_t` indicando o status de sucesso ou falha.
- **Funcionamento**:
  - Configura o **MPU6050** em modo ativo.
  - Define a escala do acelerômetro e do giroscópio.
  - Inicializa o barramento I2C para comunicação com o sensor.

---

### **2. `void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct)`**
- **Objetivo**: Lê os valores brutos do acelerômetro e armazena na estrutura `MPU6050_t`.
- **Parâmetros**:
  - `I2C_HandleTypeDef *hi2c`: Estrutura de configuração da comunicação I2C.
  - `MPU6050_t *DataStruct`: Estrutura onde os dados serão armazenados.
- **Funcionamento**:
  - Lê os registros correspondentes ao **acelerômetro**.
  - Armazena os valores RAW em `Accel_X_RAW`, `Accel_Y_RAW` e `Accel_Z_RAW`.
  - Converte os valores para unidades físicas (gravidade "g") e armazena em `Ax`, `Ay` e `Az`.

---

### **3. `void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct)`**
- **Objetivo**: Lê os valores brutos do giroscópio e armazena na estrutura `MPU6050_t`.
- **Parâmetros**:
  - `I2C_HandleTypeDef *hi2c`: Estrutura de configuração da comunicação I2C.
  - `MPU6050_t *DataStruct`: Estrutura onde os dados serão armazenados.
- **Funcionamento**:
  - Lê os registros correspondentes ao **giroscópio**.
  - Armazena os valores RAW em `Gyro_X_RAW`, `Gyro_Y_RAW` e `Gyro_Z_RAW`.
  - Converte os valores para graus por segundo (°/s) e armazena em `Gx`, `Gy` e `Gz`.

---

### **4. `void MPU6050_Read_Temp(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct)`**
- **Objetivo**: Lê o valor da temperatura interna do sensor MPU6050.
- **Parâmetros**:
  - `I2C_HandleTypeDef *hi2c`: Estrutura de configuração da comunicação I2C.
  - `MPU6050_t *DataStruct`: Estrutura onde o dado será armazenado.
- **Funcionamento**:
  - Lê o registro da temperatura.
  - Converte o valor bruto em graus Celsius utilizando a fórmula do sensor:
    \[
    \text{Temperatura} = (\text{Valor\_Bruto} / 340.0) + 36.53
    \]
  - Armazena o resultado em `Temperature`.

---

### **5. `void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct)`**
- **Objetivo**: Lê todos os valores (acelerômetro, giroscópio e temperatura) de uma vez.
- **Parâmetros**:
  - `I2C_HandleTypeDef *hi2c`: Estrutura de configuração da comunicação I2C.
  - `MPU6050_t *DataStruct`: Estrutura onde os dados serão armazenados.
- **Funcionamento**:
  - Lê os registros do acelerômetro, giroscópio e temperatura.
  - Preenche a estrutura `MPU6050_t` com todos os valores lidos.

---

### **6. `double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)`**
- **Objetivo**: Aplica o **Filtro de Kalman** para estimar o ângulo com base nos dados do giroscópio e acelerômetro.
- **Parâmetros**:
  - `Kalman_t *Kalman`: Estrutura contendo os parâmetros do filtro.
  - `double newAngle`: Novo valor do ângulo medido (por exemplo, do acelerômetro).
  - `double newRate`: Taxa de mudança do ângulo (obtida do giroscópio).
  - `double dt`: Intervalo de tempo entre medições.
- **Retorno**: Ângulo filtrado.

- **Funcionamento**:
  - **Predição**:
    - Estima o ângulo com base na taxa do giroscópio e no tempo (`dt`).
    - Atualiza a matriz de covariância do erro.
  - **Atualização**:
    - Compara a estimativa com a medida do ângulo fornecida pelo acelerômetro.
    - Calcula o **Ganho de Kalman** para determinar o peso da medição.
    - Atualiza o ângulo estimado e o bias do giroscópio.
    - Ajusta a matriz de covariância.
  - **Resultado**: Retorna o valor filtrado do ângulo, mais preciso que as medições individuais.

---

## **Definições de Constantes e Endereços**

1. **`BMP280_I2C_ADDRESS_0` e `BMP280_I2C_ADDRESS_1`**  
   - Define os possíveis endereços I2C do sensor **BMP280/BME280**:
     - `0x76`: Quando o pino **SDO** está conectado ao **GND**.
     - `0x77`: Quando o pino **SDO** está conectado ao **VCC**.

2. **`BMP280_CHIP_ID` e `BME280_CHIP_ID`**  
   - **`0x58`**: Identificador do chip BMP280.  
   - **`0x60`**: Identificador do chip BME280 (que inclui medição de umidade).

---

## **Enumeradores**

### 1. **`BMP280_Mode`**
Define o modo de operação do sensor:
- `BMP280_MODE_SLEEP`: Modo de repouso.
- `BMP280_MODE_FORCED`: Medição iniciada manualmente pelo usuário.
- `BMP280_MODE_NORMAL`: Medições contínuas e automáticas.

### 2. **`BMP280_Filter`**
Define a configuração do filtro de ruído:
- `BMP280_FILTER_OFF`: Filtro desativado.
- `BMP280_FILTER_2` a `BMP280_FILTER_16`: Níveis progressivos de filtragem.

### 3. **`BMP280_Oversampling`**
Configura a precisão da medição (sobreamostragem):
- `BMP280_SKIPPED`: Nenhuma medição realizada.
- `BMP280_ULTRA_LOW_POWER`: Sobreamostragem **x1** (baixa potência).
- `BMP280_ULTRA_HIGH_RES`: Sobreamostragem **x16** (alta precisão).

### 4. **`BMP280_StandbyTime`**
Define o tempo de espera entre medições no modo **NORMAL**:
- Valores variam entre **0.5ms** e **4000ms** (4s).

---

## **Estruturas**

### 1. **`bmp280_params_t`**  
Estrutura para armazenar os parâmetros de configuração do sensor:
- **Modo de operação** (`mode`).
- **Filtro de ruído** (`filter`).
- **Sobreamostragem para pressão, temperatura e umidade** (`oversampling_pressure`, etc.).
- **Tempo de espera entre medições** (`standby`).

### 2. **`BMP280_HandleTypedef`**  
Estrutura principal do sensor que armazena:
- **Constantes de calibração** (dig_T1, dig_P1, etc.) para correção dos dados medidos.
- **Endereço I2C** (`addr`).
- **Ponteiro para o barramento I2C** (`I2C_HandleTypeDef* i2c`).
- **Parâmetros de configuração** (`params`).
- **ID do chip** (`id`).

---

## **Funções**

### 1. **`void bmp280_init_default_params(bmp280_params_t *params)`**
- **Objetivo**: Inicializa os parâmetros padrão do sensor.
- **Configurações Padrão**:
  - Modo: `NORMAL`
  - Filtro: `OFF`
  - Sobreamostragem: `x4`
  - Tempo de espera: `250ms`
- **Parâmetro**:
  - `bmp280_params_t *params`: Estrutura onde os parâmetros padrão serão armazenados.

---

### 2. **`bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params)`**
- **Objetivo**: Inicializa o sensor BMP280/BME280.
- **Ações**:
  1. **Soft Reset** do dispositivo.
  2. Lê e armazena as constantes de calibração do sensor.
  3. Configura o dispositivo conforme os parâmetros fornecidos.
- **Parâmetros**:
  - `BMP280_HandleTypedef *dev`: Estrutura do sensor.
  - `bmp280_params_t *params`: Parâmetros de configuração.
- **Retorno**: 
  - `true` se a inicialização for bem-sucedida, `false` caso contrário.

---

### 3. **`bool bmp280_force_measurement(BMP280_HandleTypedef *dev)`**
- **Objetivo**: Inicia manualmente uma medição no **modo FORCED**.
- **Requisitos**:
  - O sensor deve estar no modo **FORCED**.
- **Retorno**:
  - `true` se o comando for bem-sucedido, `false` caso contrário.

---

### 4. **`bool bmp280_is_measuring(BMP280_HandleTypedef *dev)`**
- **Objetivo**: Verifica se o sensor ainda está realizando medições.
- **Retorno**:
  - `true` se o sensor estiver ocupado.
  - `false` se a medição estiver concluída.

---

### 5. **`bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure, uint32_t *humidity)`**
- **Objetivo**: Lê os valores de temperatura, pressão e, opcionalmente, umidade (para o BME280).
- **Formato dos Dados**:
  - Temperatura: Fixado em **graus Celsius x100** (por exemplo, 2535 representa 25.35°C).
  - Pressão: Representada em **Pa** com 24 bits inteiros e 8 bits fracionários.
  - Umidade: Disponível apenas para o **BME280**, em formato de 22 bits inteiro e 10 bits fracionário.
- **Parâmetros**:
  - `int32_t *temperature`: Ponteiro para armazenar a temperatura.
  - `uint32_t *pressure`: Ponteiro para armazenar a pressão.
  - `uint32_t *humidity`: Ponteiro para armazenar a umidade.
- **Retorno**: 
  - `true` se a leitura for bem-sucedida.

---

### 6. **`bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature, float *pressure, float *humidity, float *altitude)`**
- **Objetivo**: Lê e converte os valores de temperatura, pressão e umidade em **ponto flutuante**.
- **Dados Retornados**:
  - Temperatura em **graus Celsius**.
  - Pressão em **Pascals**.
  - Umidade (apenas no **BME280**) em **percentual de umidade relativa**.
  - Altitude calculada em metros, se fornecido.
- **Parâmetros**:
  - `float *temperature`: Ponteiro para armazenar a temperatura.
  - `float *pressure`: Ponteiro para armazenar a pressão.
  - `float *humidity`: Ponteiro para armazenar a umidade.
  - `float *altitude`: Ponteiro para armazenar a altitude (opcional).
- **Retorno**:
  - `true` se a leitura for bem-sucedida.

---

## Roteiro para o Desenvolvimento

O projeto será desenvolvido seguindo as seguintes etapas:

1. **Planejamento e Definição do Tema**:
   - Escolha de um problema real: Automação de uma estufa inteligente.
   - Objetivo: Monitorar e controlar temperatura, umidade e iluminação.

2. **Projeto de Hardware**:
   - Montagem dos sensores e atuadores no microcontrolador.
   - Diagrama elétrico detalhado (ver **hardware/esquema_eletrico.png**).

3. **Implementação do Software**:
   - Desenvolvimento do código em C/C++ para a plataforma **NUCLEO-F446RE**.
   - Uso de bibliotecas para os dispositivos (I2C, UART, etc.).

4. **Testes e Validação**:
   - Validação da funcionalidade dos sensores e atuadores.
   - Teste do sistema integrado.

5. **Documentação**:
   - Documentar código, esquemas elétricos e o funcionamento da estufa.
  
## Diagrama Esquemático de Montagem

![diagrama_esquematico](https://github.com/user-attachments/assets/29d127f1-1e30-4e54-80af-1e94dd73e646)


## Como Executar o Projeto

Pré-requisitos:

  - Instale a STM32CubeIDE.
  - Instale as bibliotecas necessárias:

# Imagens do Projeto

  - Imagem do Projeto

