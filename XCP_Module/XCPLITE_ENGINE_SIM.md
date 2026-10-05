# Integração XCPlite ↔ ATI VISION na ECU virtual do engine-sim

Este documento passa o contexto para quem for integrar o XCPlite na ECU virtual do simulador
(C/C++, baseado no engine-sim / `engine_sim_app`). Comunicação com o usuário em **português (BR)**.

Tudo o que está aqui foi descoberto no projeto de teste `C:\Github\Simulation_XCP`. O código de
referência funcionando com o VISION é [xcplite_ecu/src/main.c](xcplite_ecu/src/main.c).

## Objetivo

A ECU virtual do simulador deve falar **XCP on Ethernet (UDP)** com o **ATI VISION**. Com isso, o
VISION mede as variáveis por DAQ e calibra os parâmetros da ECU (escalares, curvas e mapas com
eixos editáveis) com o simulador rodando. O uso é em laboratório de bancada de motores.

## Situação atual

| Item | Situação |
|---|---|
| XCPlite compilado no Windows (MSVC) | ✅ |
| VISION conecta, mede por DAQ (10 ms e 100 ms) e lê calibração | ✅ validado com a `xcplite_ecu` |
| Escrita de parâmetro escalar pelo VISION | ✅ chega na ECU e o modelo usa o valor novo |
| Valor escrito aparecendo na Data List do VISION | Corrigido com o "espelho" (ver abaixo); compilado, **falta confirmar no VISION** |
| Curva e mapa com eixos COM_AXIS e ponto de operação (INPUT_QUANTITY) | Implementado; **falta confirmar no VISION** (inclusive se o mapa aparece transposto) |
| "Act Rate" no VISION mostra 8,9 ms em todos os itens, até no de 100 ms | Em aberto. Conferir com `pedal_pos` num gráfico: a rampa tem período de exatamente 30 s |
| Persistência da calibração | Não implementada (ver seção própria) |
| Integração no engine-sim | **Ainda não começou**. É a próxima etapa |

## Ambiente de build

- Clone do XCPlite em `C:\Github\XCPlite`, commit `88e3cbe` (24/09/2026).
- O XCPlite **exige MSVC** no Windows. O MinGW (gcc do Strawberry Perl) falha: intrínsecos `_Interlocked*8/16`
  e `#error "Unsupported platform for section based event pre-registration"`.
- O MSVC está instalado: VS 2022 **Build Tools**, toolset 14.44, SDK 10.0.26100. O CMake 3.29 e o
  Ninja (do Strawberry) estão no PATH.
- O padrão de build está em [xcplite_ecu/build.bat](xcplite_ecu/build.bat):
  ```bat
  call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
  cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl
  cmake --build build
  ```
- O XCPlite entra no projeto por `add_subdirectory`, com o override de configuração **definido antes**.
  Ver [xcplite_ecu/CMakeLists.txt](xcplite_ecu/CMakeLists.txt):
  ```cmake
  set(XCPLITE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../XCPlite" CACHE PATH "Diretorio do XCPlite")
  set(XCPLITE_CFG_OVERRIDE "${CMAKE_CURRENT_SOURCE_DIR}/config/xcplib_app_cfg.h")
  add_subdirectory(${XCPLITE_DIR} xcplite)
  target_link_libraries(<alvo> PRIVATE xcplite::xcplite)
  ```
- O XCPlite usa C11 e C++17 (C++20 no Windows). Ainda não sei como o engine-sim é compilado
  (CMake? solução do Visual Studio?). Verifique antes de integrar.

## Configuração obrigatória para o VISION

Copie [xcplite_ecu/config/xcplib_app_cfg.h](xcplite_ecu/config/xcplib_app_cfg.h) para o projeto do simulador:

```c
#define OPTION_CAL_SEGMENTS_ABS      // modo ACSDD: address extension 0 = endereço absoluto
#undef  OPTION_ENABLE_PERSISTENCE    // .bin do XCPlite não funciona no modo ACSDD
```

**Motivo:** o VISION **ignora `ECU_ADDRESS_EXTENSION` do A2L** e manda sempre extensão 0 em
`WRITE_DAQ` e `SET_MTA`. No modo padrão do XCPlite (CASDD), a extensão 0 significa "relativo ao
segmento de calibração" e as variáveis globais ficam na extensão 1. Resultado: todo `WRITE_DAQ` volta
`ERR_ACCESS_DENIED (0x24)` e o VISION marca os itens como "Dropped". No modo ACSDD tudo fica na
extensão 0. O XCPlite reconhece endereços dentro da struct de calibração e redireciona leitura e
escrita (`UPLOAD`/`DOWNLOAD`) para a página ativa do segmento.

Mantive o timestamp no padrão (DWORD, 1 ns). Com `OPTION_CLOCK_TICKS_1US`, o A2L gerado continua
dizendo `0x1 UNIT_1NS`, mas `GET_DAQ_RESOLUTION_INFO` responde 1000 ticks, o que é inconsistente.

## Regras de código (aprendidas na prática)

1. **Use só variáveis globais ou estáticas** para medição, com endereçamento absoluto
   (`A2lSetAbsoluteAddrMode_i(evento)`). **Não use** medições de pilha (`A2lSetStackAddrMode`,
   `DaqEventVar`/`A2L_MEAS` com locais) nem endereçamento relativo ou dinâmico. Eles exigem address
   extension ≠ 0, e o VISION não envia.
2. **Crie os eventos com `XcpCreateEvent(nome, ciclo_ns, prioridade)`** e dispare com `XcpEvent(id)`.
   - Não use `DaqCreateEventExt`/`DaqCreateEvent`/`DaqTriggerEvent`: **no MSVC eles descartam o tempo
     de ciclo** e criam o evento como esporádico (ciclo 0).
   - Com eventos esporádicos, o VISION **não configura DAQ nenhum**. Foi o que aconteceu com o exemplo
     `hello_xcp`: ele conectava e todos os valores ficavam em 0.
3. **Parâmetros ficam numa struct registrada com `XcpCreateCalSeg`.** O código da ECU lê a struct a
   cada ciclo por `XcpLockCalSeg`/`XcpUnlockCalSeg`, ou pelo wrapper C++ `xcp::CalSeg<T>::lock()` do
   `xcplib.hpp`. Bloqueie **uma vez por ciclo da ECU**, não por passo de física.
4. **Espelho da página ativa (necessário para a Data List do VISION).** O VISION edita escalares na
   Data List, e a Data List mede o parâmetro **por DAQ** no endereço absoluto. Esse endereço é o da
   struct default (página de referência), que nunca muda: o valor escrito "volta" na tela, embora o
   modelo use o valor novo. O XCPlite não tem caminho de DAQ para a página ativa (ver
   `XcpAddOdtEntry` em `src/xcplite.c`). Solução adotada:
   - a struct default **não é `const`**;
   - depois de usar os parâmetros, ainda com o lock: `if (p != &params) memcpy(&params, p, sizeof(params));`
   - **efeito colateral:** a página de referência deixa de guardar os valores de fábrica.
5. **Eixos editáveis:** registre os eixos como `A2lCreateAxis` (vira `AXIS_PTS`) e as tabelas com
   `A2lCreateCurveWithSharedAxis`/`A2lCreateMapWithSharedAxis` (vira `COM_AXIS`). Nunca deixe eixos
   fixos no código: as funções sem eixo geram `FIX_AXIS` com índices 0..n−1.
6. **Ponto de operação nas tabelas do VISION:** antes de criar cada eixo e tabela, chame
   `A2lSetInputQuantity_x("medicao")` (e `_y` para mapas). Depois, volte para `NULL`. A medição citada
   precisa existir no A2L. Por exemplo, a entrada Y do mapa é o pedal **depois** da curva (`pedal_eff`),
   não `pedal_pos`.
7. **Mapas:** o record layout gerado é `ROW_DIR`. Declare a matriz em C como `[ydim][xdim]` e passe
   `xdim, ydim` para `A2lCreateMap*`. Se o VISION mostrar o mapa transposto, inverta.
8. **Gere o A2L na partida:** `A2lInit(..., A2L_MODE_WRITE_ALWAYS | A2L_MODE_AUTO_GROUPS)`,
   registre tudo e chame `A2lFinalize()` antes de entrar no loop. Assim o arquivo existe antes do
   VISION conectar.
   - O A2L faz `/include "XCP_104.aml"`. Copie `C:\Github\XCPlite\XCP_104.aml` para a pasta onde o A2L é gerado.
   - O A2L é gerado no diretório de trabalho do processo.
9. **No Windows:** chame `timeBeginPeriod(1)` (link `winmm.lib`). O `sleepUs` do XCPlite usa `Sleep`,
   que por padrão tem resolução de ~15,6 ms. Para depurar, use `setvbuf(stdout, NULL, _IONBF, 0)` e
   `XcpSetLogLevel(4)` (imprime cada comando XCP). Sem o `setvbuf`, o log redirecionado para arquivo
   perde o final no Ctrl+C.
10. **Fila de transmissão no Windows (`queue32`):** junta ~90 DTOs pequenos por datagrama UDP. O DAQ
    chega em rajadas, com até ~1 s de latência. Os timestamps continuam corretos. Ainda não foi ajustado.

## Esqueleto da integração (C++)

```cpp
#include <a2l.h>
#include <xcplib.h>
#include <xcplib.hpp>

struct EcuParams {             // tudo que for calibrável, inclusive os eixos
    float idle_speed;
    float ign_rpm_axis[16];
    float ign_load_axis[16];
    float ignition_map[16][16];   // [load (y)][rpm (x)]
};
EcuParams ecu_params = { /* padrões ou valores carregados do DCM, ANTES de criar o CalSeg */ };
float engine_rpm, engine_load, spark_advance;   // medições: globais

void ecuXcpInit() {
    uint8_t addr[4] = {0, 0, 0, 0};
    XcpInit("engine_sim_ecu", "ESIM_ECU_V1.0", XCP_MODE_LOCAL);       // 2º argumento = EPK
    XcpEthServerInit(addr, 5555, false /*UDP*/, 64 * 1024);
    A2lInit(addr, 5555, false, A2L_MODE_WRITE_ALWAYS | A2L_MODE_AUTO_GROUPS);
    ev_ecu = XcpCreateEvent("Ecu_10ms", 10000000, 0);
    seg = XcpCreateCalSeg("params", &ecu_params, sizeof(ecu_params));
    A2lSetSegmentAddrMode(seg, ecu_params);
    // A2lSetInputQuantity_x/y + A2lCreateAxis / A2lCreateMapWithSharedAxis / A2lCreateParameter ...
    A2lSetAbsoluteAddrMode_i(ev_ecu);
    // A2lCreatePhysMeasurement(engine_rpm, "Rotação", "rpm", 0, 8000); ...
    A2lFinalize();
}

void Ecu::step() {                                   // ritmo da ECU, não da física
    const EcuParams *p = (const EcuParams *)XcpLockCalSeg(seg);
    // ... lógica da ECU usando p-> ...
    if (p != &ecu_params) memcpy(&ecu_params, p, sizeof(ecu_params));   // espelho (regra 4)
    XcpUnlockCalSeg(seg);
    XcpEvent(ev_ecu);           // ou XcpEventAt(ev_ecu, timestamp) para usar o tempo simulado
}
```

Pontos específicos do engine-sim:
- A física roda em passos muito pequenos. A ECU deve rodar no ritmo dela (por exemplo 1 ms ou 10 ms,
  ou sincronizada com o ângulo do virabrequim). O lock e o evento XCP ficam no passo da ECU.
- **Tempo simulado × tempo real:** `XcpEventAt(evento, clock)` aceita um timestamp fornecido pela
  aplicação. Com ele, os gráficos do VISION ficam no tempo do motor, mesmo se a simulação não rodar
  em tempo real. Ainda não foi testado.
- A geometria e o hardware do motor (scripts `.mr` do engine-sim) não são calibráveis. Só o que
  estiver na struct de parâmetros é.

## Persistência e DCM (decisão do usuário)

- O simulador **já usa um arquivo DCM** para guardar a calibração, e o usuário quer **manter o DCM**.
- Papéis:
  - o DCM é armazenamento e troca de dados (carregar na partida, salvar quando pedido);
  - o XCPlite é o acesso ao vivo;
  - o código da ECU lê **sempre do CalSeg**, nunca do DCM durante a execução.
- Fluxo:
  - **partida:** DCM → struct default → `XcpCreateCalSeg`;
  - **execução:** VISION ↔ XCP;
  - **salvar:** snapshot sob `XcpLockCalSeg` → DCM. Gatilhos sugeridos: ao fechar, e/ou um parâmetro
    `save_request` que o usuário põe em 1 no VISION.
- **Atenção aos eixos:** para continuarem editáveis e serem salvos, os eixos precisam estar na
  struct (regra 5). O leitor e o gravador de DCM precisam tratar `ST/X`/`ST/Y` de
  `KENNLINIE`/`KENNFELD` e/ou `STUETZSTELLENVERTEILUNG` (eixo compartilhado). Verifique se o parser
  atual do simulador faz isso. Ele pode estar lendo só `WERT`.
- **Recomendação de projeto:** uma tabela única de parâmetros (nome, campo, tipo, dimensões, eixos,
  unidade). Ela alimenta tanto o registro no A2L quanto o leitor e o gravador de DCM. Os nomes no DCM
  devem ser iguais aos do A2L (o XCPlite gera `params.<campo>`).
- Recarregar o DCM com a ECU rodando: não confirmei se existe API pública para a aplicação escrever
  na página de trabalho. O caminho seguro é carregar só na partida.
- Alternativa sem código: o VISION guarda a calibração e grava PC → ECU ao conectar, quando o
  checksum difere.
- O número de pontos dos eixos é estrutural: muda a struct e o A2L, e exige reimportar no VISION.

## Configuração do VISION (já validada)

- Device: **XCP Controller Device**. IP Settings: UDP, **Override Device Settings**, `127.0.0.1:5555`.
- **A cada A2L novo** (os endereços mudam quando a struct ou as globais mudam): reimportar marcando
  **"Replace existing data items"** e **"Delete existing Device Settings and Memory Regions"**, ou criar
  strategy e device do zero.
  - Sem isso, o VISION mantém as regiões de memória antigas. No log aparece `BUILD_CHECKSUM` com o
    tamanho do segmento antigo, e os parâmetros mostram `#NAN#`.
- Apenas um processo pode usar a porta UDP 5555: feche a `xcplite_ecu` ou o slave Python antes.

## Como diagnosticar

- Rode a ECU com saída para arquivo (`> ecu_log.txt 2>&1`). Com log nível 4, cada comando XCP aparece
  com a resposta.
- `WRITE_DAQ ... <- ERROR: 24h` → address extension. Ver a configuração ACSDD e a regra 1.
- Nenhum `ALLOC_DAQ` no log, mas o VISION conectado → eventos esporádicos. Ver a regra 2.
- `BUILD_CHECKSUM size=` diferente do tamanho da struct → strategy antiga no VISION.
- Parâmetro "volta" ao valor padrão na Data List → falta o espelho. Ver a regra 4.

## Arquivos de referência neste repositório

| Arquivo | Conteúdo |
|---|---|
| [xcplite_ecu/src/main.c](xcplite_ecu/src/main.c) | ECU de teste completa com XCPlite (modelo simples de motor), validada com o VISION |
| [xcplite_ecu/config/xcplib_app_cfg.h](xcplite_ecu/config/xcplib_app_cfg.h) | Override de configuração para o VISION |
| [xcplite_ecu/CMakeLists.txt](xcplite_ecu/CMakeLists.txt), [xcplite_ecu/build.bat](xcplite_ecu/build.bat) | Build com MSVC + XCPlite via `add_subdirectory` |
| [CLAUDE.md](CLAUDE.md) | Histórico completo do projeto, incluindo o slave Python anterior |
| `C:\Github\XCPlite\docs\TECHNICAL.md` | Modos de endereçamento (CASDD/ACSDD) |
| `C:\Github\XCPlite\docs\CAL_RCU.md` | Funcionamento dos segmentos de calibração |

## Próximos passos sugeridos

1. Ler o código da ECU do engine-sim:
   - onde fica o passo da ECU;
   - como estão os parâmetros hoje;
   - como funciona o leitor e o gravador de DCM;
   - qual é o sistema de build.
2. Montar a struct de parâmetros (com eixos) e a tabela única de parâmetros.
3. Integrar o XCPlite seguindo as regras acima. Começar com poucos parâmetros e medições e validar
   no VISION.
4. Ligar o DCM: carregar na partida, salvar ao fechar e/ou por `save_request`.
5. Decidir o timestamp: tempo real ou simulado (`XcpEventAt`).
