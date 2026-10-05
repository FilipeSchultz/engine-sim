// =====================================================================
//  ÁRVORE DE CONTEÚDO (aba "Conteúdo")  -  ÚNICO ARQUIVO A EDITAR
//  ao adicionar, remover ou reorganizar páginas.
//
//  - Página:  { titulo: 'Texto na árvore', pagina: 'Arquivo.html' }
//  - Pasta:   { titulo: 'Nome da pasta', filhos: [ ...itens... ] }
//  - Uma pasta pode ter "pagina" também (abre ao clicar no nome).
//
//  Os arquivos .html ficam na pasta Help ou em subpastas
//  (ex.: 'ECU/Funcoes.html'), com nome SEM acento e SEM espaço.
//  Use sempre barra normal "/" no caminho.
//  A pesquisa indexa automaticamente todas as páginas desta lista.
// =====================================================================

window.HELP_TOC = [
  { titulo: 'Início', pagina: 'Inicio.html' },
  {
    titulo: 'Primeiros passos',
    filhos: [
      { titulo: 'Compilar e executar',    pagina: 'Compilar_Executar.html' },
      { titulo: 'Variáveis de ambiente',  pagina: 'Variaveis_Ambiente.html' }
    ]
  },
  {
    titulo: 'Operação',
    filhos: [
      { titulo: 'Controles do teclado',   pagina: 'Operacao/Controles.html' },
      { titulo: 'Telas e instrumentos',   pagina: 'Operacao/Telas.html' },
      { titulo: 'Dinamômetro',            pagina: 'Operacao/Dinamometro.html' },
      { titulo: 'Mensagens do sistema',   pagina: 'Operacao/Mensagens.html' }
    ]
  },
  {
    titulo: 'Scripts de motor (.mr)',
    pagina: 'Motores/Script_Motor.html',
    filhos: [
      { titulo: 'Parâmetros de combustão', pagina: 'Motores/Parametros_Combustao.html' },
      { titulo: 'Calibrar contra dados reais', pagina: 'Motores/Calibracao_Motor.html' },
      { titulo: 'Motor CFR',              pagina: 'Motores/Motor_CFR.html' },
      { titulo: 'Motores disponíveis',    pagina: 'Motores/Motores_Disponiveis.html' }
    ]
  },
  {
    titulo: 'Modelagem física',
    pagina: 'Modelagem/Modelagem.html',
    filhos: [
      { titulo: 'Modelo de gás',            pagina: 'Modelagem/Modelo_Gas.html' },
      { titulo: 'Transferência de calor',   pagina: 'Modelagem/Transferencia_Calor.html' },
      { titulo: 'Modelos de chama',         pagina: 'Modelagem/Modelos_Chama.html' },
      { titulo: 'Limitações conhecidas',    pagina: 'Modelagem/Limitacoes.html' }
    ]
  },
  {
    titulo: 'ECU virtual',
    pagina: 'ECU/ECU.html',
    filhos: [
      { titulo: 'Funções',                  pagina: 'ECU/Funcoes.html' },
      { titulo: 'Parâmetros de calibração', pagina: 'ECU/Parametros.html' },
      { titulo: 'Medições e datalog',       pagina: 'ECU/Medicoes.html' },
      { titulo: 'Arquivo DCM',              pagina: 'ECU/Arquivo_DCM.html' },
      { titulo: 'Aprendizado de mapas',     pagina: 'ECU/Aprendizado.html' },
      { titulo: 'Sensores e atuadores',     pagina: 'ECU/Sensores.html' },
      { titulo: 'ECU standalone',           pagina: 'ECU/ECU_Standalone.html' },
      { titulo: 'Estender a ECU',           pagina: 'ECU/Desenvolvimento.html' }
    ]
  },
  {
    titulo: 'Comunicação',
    filhos: [
      {
        titulo: 'Controle CAN (AVL EMCON)',
        pagina: 'Comunicacao/CAN_EMCON.html',
        filhos: [
          { titulo: 'Mensagens e sinais',  pagina: 'Comunicacao/CAN_Mensagens.html' },
          { titulo: 'Teste com Kvaser CanKing', pagina: 'Comunicacao/CAN_CanKing.html' }
        ]
      },
      { titulo: 'XCP com ATI VISION',      pagina: 'Comunicacao/XCP_VISION.html' }
    ]
  },
  {
    titulo: 'Indicação de combustão',
    pagina: 'Indicacao/Indicacao.html',
    filhos: [
      { titulo: 'IndiCom virtual',         pagina: 'Indicacao/IndiCom.html' },
      { titulo: 'Protocolo UDP',           pagina: 'Indicacao/Protocolo_UDP.html' },
      { titulo: 'Log de indicação',        pagina: 'Indicacao/Log_Indicacao.html' }
    ]
  },
  {
    titulo: 'Sobre esta ajuda',
    filhos: [
      { titulo: 'Como editar a ajuda', pagina: 'Manutencao_Ajuda.html' }
    ]
  }
];
