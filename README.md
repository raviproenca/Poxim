# Poxim Architecture - Processor Simulator

This repository contains an implementation of a simple, educational, and hypothetical processor based on the **Poxim** architecture. Poxim follows the **Complexity-Reduced Instruction Set Processor (CRISP)** philosophy, being an architecture designed for educational purposes, focusing on low-level operations, register manipulation, memory handling, and stack management.

## Architecture Structure

The Poxim architecture consists of 3 instruction formats:
- **U Format (U - Uniform):** Instructions with basic operations (e.g., `mov`, `add`, `sub`), with 6 bits for the operation and 5 bits for the operands.
- **F Format (F - Immediate):** Instructions that use immediate values (e.g., `addi`, `subi`, `muli`), with 16 bits for the immediate value.
- **S Format (S - Branch):** Control flow instructions like `beq`, `bne`, `bgt`, with 26 bits for the immediate value.

Additionally, the architecture includes the following features:
- **Interrupts and Exceptions (Software and Hardware)**
- **Interrupt Service Routines (ISR)**
- **Floating Point Unit (FPU)**
- **Terminal In/Out (Data Read and Write)**
- **Timer (Watchdog)**
- **Data Read and Write in Memory (8, 16, 32 bits)**
- **Memory Cache**
- **Pipeline (NOT IMPLEMENTED)**

## Implemented Features

### 1. **Register and Memory Simulation**
   The processor has 32 registers and 32KiB of memory. Data manipulation can be performed through bitwise operations, and memory is accessed directly by addresses.

### 2. **Interrupts and Exceptions**
   The processor can be interrupted by hardware (e.g., timer) or software exceptions (e.g., division by zero). An interrupt vector and interrupt service routines (ISRs) are implemented.

### 3. **FPU (Floating Point Unit)**
   The FPU is responsible for performing floating point operations (such as addition and subtraction), using specific instructions for these operations.

### 4. **Input and Output (Terminal In/Out)**
   The processor can interact with the terminal, simulating data read and write operations. This is done through dedicated memory buffers.

### 5. **Timer (Watchdog)**
   A timer is used to generate interrupts when the execution time exceeds a limit. This can be configured to generate an exception or reset the processor.

### 6. **Data Read and Write in Memory**
   The processor performs reads and writes in different data formats: 8 bits, 16 bits, and 32 bits, with support for flag operations and reading/writing immediate values.

### 7. **Memory Cache**
   The cache stores frequently accessed data to optimize access time, with a cache miss triggering a search in the main memory.

## How to Run
Note: GCC is required to run, either in a Linux environment (native or WSL) or using MinGW if you're on Windows.
-> `gcc -Wall -O3 raviproenca_poxim.c -o raviproenca_poxim`  
-> `./raviproenca_poxim poxim.input.txt poxim.output.txt`

(WARNING: The output of poxim is about 130 thousand lines long. If your computer is not powerful, it might freeze, but usually, it just takes a while and outputs correctly in the end.)

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Arquitetura Poxim - Simulador de Processador

Este repositório contém uma implementação de um processador simples, didático e hipotético baseado na arquitetura **Poxim**. A Poxim segue a filosofia do **Complexity-Reduced Instruction Set Processor (CRISP)**, sendo uma arquitetura projetada para fins educacionais, com foco em operações de baixo nível, manipulação de registradores, memória e gerenciamento de pilha.

## Estrutura da Arquitetura

A arquitetura Poxim é composta por 3 formatos de instrução:
- **Formato U (U - Uniform):** Instruções com operações básicas (ex: `mov`, `add`, `sub`), com 6 bits para operação e 5 bits para operandos.
- **Formato F (F - Immediate):** Instruções que utilizam valores imediatos (ex: `addi`, `subi`, `muli`), com 16 bits para o valor imediato.
- **Formato S (S - Branch):** Instruções de controle de fluxo, como `beq`, `bne`, `bgt`, com 26 bits para o valor imediato.

Além disso, a arquitetura possui as seguintes funcionalidades:
- **Interrupção e Exceção (Software e Hardware)**
- **Rotinas de Serviço de Interrupção (ISR)**
- **Unidade de Ponto Flutuante (FPU)**
- **Terminal In/Out (Leitura e Escrita de Dados)**
- **Temporizador (Watchdog)**
- **Leitura e Escrita de Dados em Memória (8, 16, 32 bits)**
- **Memória Cache**
- **Pipeline (NÃO IMPLEMENTADA)**

## Funcionalidades Implementadas

### 1. **Simulação de Registradores e Memória**
   O processador possui 32 registradores e uma memória de 32KiB. A manipulação de dados pode ser feita através de operações bit a bit, e a memória é acessada diretamente por endereços.

### 2. **Interrupções e Exceções**
   O processador pode ser interrompido por hardware (ex: temporizador) ou por exceções de software (ex: divisão por zero). Há um vetor de interrupção e rotinas de serviço de interrupção (ISR) implementadas.

### 3. **FPU (Unidade de Ponto Flutuante)**
   A FPU é responsável por realizar operações de ponto flutuante (como soma e subtração), utilizando as instruções específicas para este tipo de operação.

### 4. **Entrada e Saída (Terminal In/Out)**
   O processador pode interagir com o terminal, simulando operações de leitura e escrita de dados. Isso é feito através de buffers de memória dedicados.

### 5. **Temporizador (Watchdog)**
   Um temporizador é utilizado para gerar interrupções quando o tempo de execução excede um limite. Isso pode ser configurado para gerar uma exceção ou resetar o processador.

### 6. **Leitura e Escrita de Dados em Memória**
   O processador realiza leituras e gravações em diferentes formatos de dados: 8 bits, 16 bits e 32 bits, com suporte a operações de sinalização e leitura/escrita de valores imediatos.

### 7. **Memória Cache**
   A memória cache armazena dados frequentemente acessados para otimizar o tempo de acesso, com miss de cache gerando uma busca na memória principal.

## Como Executar
Obs: é necessário o GCC para rodar, seja em ambiente linux (nativo ou WSL) ou utilizando o MinGW caso esteja no windows. 
-> gcc -Wall -O3 raviproenca_poxim.c -o raviproenca_poxim
-> ./raviproenca_poxim poxim.input.txt poxim.output.txt

(AVISO: A saída do poxim tem cerca de 130 mil linhas, se o computador não for bom pode travar, mas geralmente ele apenas demora um pouco e sai corretamente no final.)
