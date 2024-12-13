# Projeto de Estufa Automatizada

Este repositório documenta o desenvolvimento de um sistema de estufa automatizada utilizando o microcontrolador NUCLEO-F446RE e dispositivos periféricos. O projeto integra conceitos de sistemas embarcados para monitorar e controlar o ambiente da estufa, garantindo condições ideais para o crescimento das plantas.

---

## Funcionalidades:

- **Monitoramento de Condições Ambientais:**
  - Temperatura (Sensor DS18B20)
  - Pressão e umidade (Sensor BMP280)
  - Distância para medição de nível de água (Sensor HC-SR04)
- **Controle Automatizado:**
  - Servo motor para ajuste de ventilação
  - Relé para acionamento de dispositivos externos (ex: aquecedores ou ventoinhas)
- **Interface de Usuário:**
  - Display I2C SSD 1306 para exibição de dados
  - Botões para interação e ajustes manuais
- **Feedback Visual:**
  - LED onboard para indicar estados do sistema

---

## Componentes Utilizados:

### Microcontrolador
- **NUCLEO-F446RE**

### Dispositivos de Saída
- Display I2C SSD 1306
- Servo motor
- Relé
- LED onboard

### Dispositivos de Entrada
- Botões
- Sensor de Pressão e Temperatura BMP280
- Sensor de Temperatura DS18B20
- Potenciômetro 10kΩ
- Acelerômetro MPU6050
- Sensor Ultrassônico HC-SR04

---

## Estrutura do Projeto:

### Diretórios
- `src/`: Códigos fonte do firmware
- `docs/`: Documentação técnica
- `hardware/`: Esquemas e fotos do protótipo

### Dependências
Certifique-se de instalar as bibliotecas necessárias antes de compilar o projeto.
- **STM32 HAL**
- Biblioteca para display SSD1306

### Arquivo principal
- `main.c`: Contém o fluxo principal do sistema.

---

## Roteiro de Desenvolvimento:

1. **Planejamento e Definição do Tema:**
   - Definição das funcionalidades e requisitos.
2. **Projeto de Hardware:**
   - Montagem do circuito conforme descrição acima.
3. **Implementação do Software:**
   - Desenvolvimento de códigos para leitura de sensores e controle de atuadores.
4. **Testes e Validação:**
   - Verificação da funcionalidade de cada componente e do sistema integrado.

---
