````markdown
# Othello Robots Arena

Sistema integrado para execução do jogo Othello (Reversi) com robôs colaborativos, utilizando Aprendizado por Reforço e Visão Computacional.

## Estrutura do Projeto

- **`Agente_AR/`**: Contém a lógica e o treinamento do agente de Aprendizado por Reforço (`Othello_TD(0).ipynb`).
- **`Integrador/`**: O "cérebro local" do sistema. Contém a Bridge (Servidor FastAPI), a lógica de Visão Computacional (`opencv_module.py`) e os scripts de controle.
- **`Juiz_Colab/`**: Notebooks que rodam na nuvem (Google Colab) para arbitrar a partida e conectar o Agente ao Integrador local.

## Como Rodar

### 1. Pré-requisitos e Instalação

Clone o repositório e instale as dependências no computador local (que está fisicamente conectado aos robôs e à câmera):

```bash
git clone https://github.com/IcaroMoro/Othello_Robots_Arena.git
cd Othello_Robots_Arena
pip install -r requirements.txt
````

### 2. Iniciando o Integrador (Bridge Local)

O diretório `Integrador` contém o servidor que traduz os comandos da nuvem para o hardware. Para iniciar:

```bash
cd Integrador
python main.py
```

O servidor iniciará (geralmente na porta 8000).

### 3. Acessando a Interface (Dashboard)

Após iniciar o servidor, abra no navegador a interface do sistema em:

```
http://127.0.0.1:8000/dashboard
```

### 4. Verificando e Ajustando a Visão Computacional

Depois de rodar a `main.py`, verifique se a visão computacional está funcionando corretamente (detecção de tabuleiro/peças).
Se necessário, ajuste os ROIs (Regiões de Interesse) na própria imagem

### 5. Expondo com Ngrok

Para que o Juiz (no Google Colab) consiga se comunicar com sua máquina local, crie um túnel:

```bash
ngrok http 8000
```

Copie a URL gerada (ex: `https://abcd-123.ngrok-free.app`). Você precisará dela no próximo passo.

### 6. Rodando o Juiz

* Vá até a pasta `Juiz_Colab/` e abra o arquivo `Juiz_PMR3541_2024_Othello_Final.ipynb` no Google Colab.
* Cole a URL do Ngrok na variável de configuração `BRIDGE_URL` (ou similar) no início do notebook.
* Execute as células para carregar o jogo e iniciar a partida.

## Troca de Agentes de Aprendizado por Reforço

Caso queira mudar os agentes de AR que serão utilizados, vá na seção correspondente no notebook do Juiz e altere os links configurados em `teste_url`.

## Configuração do Robô (`config.py`)

No arquivo `Integrador/config.py` é possível:

* Recalibrar o robô (quando necessário, por exemplo após mudanças no setup).
* Alterar velocidades e parâmetros de movimento.

## Observações

* Arquivos de apoio: os arquivos dentro de `Agente_AR/` e módulos auxiliares em `Integrador/` (como `opencv_module.py` e `config.py`) são utilizados internamente pelo sistema ou servem para consulta/treinamento isolado. Não é necessário executá-los manualmente para rodar a partida principal.
* Hardware: este código foi projetado para manipular robôs Elite Robots EC63 e câmera Logitech C930e.

```
```
