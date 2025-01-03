![V-CC_Color_Basica_TextoAzul](https://github.com/user-attachments/assets/4f50d856-e04d-4080-858c-ee59c36bcbc0)

# Projeto de Estufa Automatizada

Este projeto consiste na criação de uma estufa inteligente para controle e monitoramento de temperatura, umidade e iluminação, utilizando o microcontrolador NUCLEO-F446RE e sensores específicos. Desenvolvido como projeto final da capacitação em Sistemas Embarcados do <em> Virtus CC </em>, o projeto integra conceitos de hardware, software e documentação.

---

## **Requisitos do Projeto**

### **Hardware Utilizado**

#### **Microcontrolador**
- **NUCLEO-F446RE**

#### **Dispositivos de Saída**
- **Display OLED I2C**: SSD1306
- **Servo Motor**
- **Buzzer**
- **Porta USART** (Terminal Serial)
- **LED Onboard** (NUCLEO-F446RE)

#### **Dispositivos de Entrada**
- **Botões**
- **Acelerômetro/Giroscópio**: MPU6050
- **Sensor de Pressão e Temperatura**: BMP280
- **Sensor Ultrassônico de Distância**: HC-SR04

---

## **Funções Declaradas**

### **1. Funções de Inicialização**
Responsáveis pela configuração dos periféricos do microcontrolador:

| **Função**                        | **Descrição**                                                                |
|-----------------------------------|------------------------------------------------------------------------------|
| `void SystemClock_Config(void)`   | Configura o sistema de clock utilizando o oscilador HSI com PLL.             |
| `static void MX_GPIO_Init(void)`  | Inicializa os pinos GPIO.                                                    |
| `static void MX_USART2_UART_Init(void)` | Configura a comunicação serial pela UART2.                             |
| `static void MX_I2C1_Init(void)`  | Inicializa o barramento I2C para sensores (BMP280, MPU6050) e display OLED.  |
| `static void MX_TIM1_Init(void)`  | Configura o temporizador TIM1 para controle PWM do servo motor.              |
| `static void MX_TIM3_Init(void)`  | Configura o temporizador TIM3 para dispositivos dependentes da temperatura.  |
| `static void MX_TIM14_Init(void)` | Configura o TIM14 para medições do sensor ultrassônico HC-SR04.              |
| `void HCSR04_Read(void)`          | Realiza a medição de distância com o sensor ultrassônico HC-SR04.            |

---

### **2. Funções do Usuário**
Implementações personalizadas para leitura e manipulação de dispositivos:

| **Função**                        | **Descrição**                                                                 |
|-----------------------------------|------------------------------------------------------------------------------|
| `void atualizarDisplayMPU6050(void)` | Lê os dados do MPU6050, exibe no display OLED e transmite via UART.          |
| `void atualizarDisplayBMP280(void)` | Lê temperatura, pressão e altitude do BMP280. Ativa PWM se temperatura > 26,5°C. |
| `void openDrip(void)`          | Simula a abertura do sistema de gotejamento ao pressionar o BUTTON2          |
| `void capacity(void)`          | Realiza a medição de distância com o sensor ultrassônico HC-SR04.            |


---

### **3. Função Principal (`main`)**

O **loop principal** executa as seguintes tarefas de forma repetitiva:
1. **Inicialização**: Configura periféricos e inicializa sensores.
2. **Leitura e Exibição de Dados**:
   - Dados do **BMP280**: Pressão, temperatura e altitude.
   - Dados do **MPU6050**: Ângulos de inclinação (filtrados por Kalman).
3. **Controle do Servo Motor**: Movimento gradual com base em variáveis definidas.
4. **Medição de Distância**: Leitura do **HC-SR04** e transmissão via UART.

---

## **Estruturas Definidas**

### **1. `MPU6050_t`**
Armazena dados do sensor MPU6050:

- **Acelerômetro**: Valores brutos e processados (`Ax`, `Ay`, `Az`).
- **Giroscópio**: Valores brutos e processados (`Gx`, `Gy`, `Gz`).
- **Temperatura**: Valor da temperatura interna.
- **Ângulos Filtrados**: `KalmanAngleX` e `KalmanAngleY`.

### **2. `Kalman_t`**
Parâmetros do filtro de Kalman:
- **Estado Atual**: Ângulo estimado e bias.
- **Covariância**: Matriz `P[2][2]`.
- **Variâncias**: Ruído de medição e processo.

---

## **Funções do Código**

### **1. Inicialização e Leitura do MPU6050**
| **Função**                        | **Descrição**                                                                 |
|-----------------------------------|------------------------------------------------------------------------------|
| `uint8_t MPU6050_Init()`          | Inicializa o sensor MPU6050. Configura registradores e comunicação I2C.       |
| `void MPU6050_Read_Accel(...)`    | Lê valores do acelerômetro e converte para "g".                              |
| `void MPU6050_Read_Gyro(...)`     | Lê valores do giroscópio e converte para °/s.                                |
| `void MPU6050_Read_Temp(...)`     | Lê e calcula a temperatura interna do sensor.                                |
| `void MPU6050_Read_All(...)`      | Lê acelerômetro, giroscópio e temperatura de uma vez.                        |

### **2. Filtro de Kalman**
| **Função**                        | **Descrição**                                                                 |
|-----------------------------------|------------------------------------------------------------------------------|
| `double Kalman_getAngle(...)`     | Aplica o filtro de Kalman para estimar os ângulos.                           |

---

## **Sensores BMP280/BME280**

### **Parâmetros Configuráveis**
- **Modos de Operação**: `SLEEP`, `FORCED`, `NORMAL`
- **Filtros**: Níveis de filtragem para redução de ruído.
- **Sobreamostragem**: Configuração para melhorar a precisão.

### **Funções**
| **Função**                        | **Descrição**                                                                 |
|-----------------------------------|------------------------------------------------------------------------------|
| `bool bmp280_init(...)`           | Inicializa o sensor BMP280/BME280 e aplica parâmetros de configuração.        |
| `bool bmp280_read_float(...)`     | Lê e converte dados de temperatura, pressão e altitude.                      |

---

## **Diagrama Esquemático**
![diagrama_esquematico](https://github.com/user-attachments/assets/29d127f1-1e30-4e54-80af-1e94dd73e646)

---

## **Como Executar o Projeto**

### **Pré-requisitos**

1. Instale o **STM32CubeIDE**.
2. Configure as bibliotecas necessárias (HAL e drivers I2C/UART).

### **Passos para Compilar e Executar**

1. Clone o repositório:
   ```bash
   git clone https://github.com/GuilhermexL/project-final.git
   ```
2. Importe o projeto no STM32CubeIDE.
3. Compile o código e grave no microcontrolador.
4. Conecte um terminal serial para visualizar os dados (ex: PuTTY, Tera Term).

---

## **Imagens do Projeto**

![IMG_20241217_082957](https://github.com/user-attachments/assets/d1322794-862f-4f8c-b263-298109e1c066)

- Vídeo de Funcionamento: https://youtu.be/vEHmp8Znd-A

---

## Contribuições

Se você encontrar problemas durante a atualização ou quiser contribuir com melhorias para este guia, sinta-se à vontade para abrir uma "issue" ou enviar um "pull request".

---

## Autores

- Aryelson Gonçalves(https://github.com/aryelson1)
- Guilherme Santos(https://github.com/GuilhermexL)

---

## Licença

##### Sistema de Estufa Automatizada

Este trabalho é software livre; você pode redistribuí-lo e/ou modificá-lo nos termos da Licença Pública Geral GNU publicada pela Free Software Foundation, na versão 3 da Licença, ou (a seu critério) qualquer versão posterior.

Este trabalho é distribuído na esperança de que será útil, mas SEM QUALQUER GARANTIA; sem a garantia implícita de COMERCIALIZAÇÃO ou ADEQUAÇÃO A UMA FINALIDADE ESPECÍFICA. Consulte a Licença Pública Geral GNU para obter mais detalhes.

---
