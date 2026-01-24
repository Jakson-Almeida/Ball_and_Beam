# Ball and Beam - Controle Digital

Projeto de controle digital aplicado ao sistema Ball and Beam (Bola e Viga).

## 📋 Descrição

Este repositório contém implementações em MATLAB/Simulink para análise e controle do sistema Ball and Beam, um problema clássico de controle onde uma bola rola sobre uma viga que pode ser inclinada. O objetivo é controlar a posição da bola através da inclinação da viga.

## 🚀 Conteúdo

### T3 - Função de Transferência
- **bola_viga_controle.m**: Projeto de controlador usando técnica de alocação de polos
  - Modelagem do sistema em tempo contínuo
  - Linearização do sistema
  - Discretização do controlador (Ts = 0.02s)
  - Conversão para espaço de estados
- **trabalho3_controle_digital2023.slx**: Simulação do sistema não linear completo no Simulink

### T4 - Espaço de Estados *(em desenvolvimento)*
- Análise e controle utilizando representação em espaço de estados

## 🔧 Requisitos

- MATLAB (versão R2020a ou superior recomendada)
- Simulink
- Control System Toolbox
- Symbolic Math Toolbox

## 📊 Características do Sistema

- **Modelagem**: Sistemas contínuo e discreto
- **Linearização**: Análise do sistema linearizado em torno do ponto de operação
- **Simulação não linear**: Validação do controlador no sistema completo
- **Abordagens**: Função de transferência e espaço de estados

## 💻 Como Usar

1. Clone o repositório:
```bash
git clone https://github.com/seu-usuario/Ball_and_Beam.git
```

2. Abra o MATLAB e navegue até a pasta do projeto

3. Execute o script desejado:
   - Para T3: `bola_viga_controle.m`
   - Para simulação: Abra `trabalho3_controle_digital2023.slx` no Simulink

## 📝 Licença

Este projeto está sob a licença especificada no arquivo LICENSE.