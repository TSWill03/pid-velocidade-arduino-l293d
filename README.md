# Controle de Velocidade por PID (Arduino UNO + L293D)

**Autores:** Wícolly Pedro Alcântara; Fernando Frazão Moreira Nunes  
**Instituição:** Instituto Federal Goiano – Campus Morrinhos  
**Curso:** Ciência da Computação  
**Disciplina:** Automação, Simulação e Controle  
**Orientador:** Prof. Dr. Jesmmer da Silveira Alves

---

## Visão Geral

Este projeto implementa um **controle de velocidade por PID** para dois motores DC usando **Arduino UNO** e a ponte H **L293D**.  
A velocidade é medida por um **tacômetro** (sensor IR refletivo ou Hall) ligado à **interrupção INT1 (D3)**, convertida em **RPM**, filtrada e controlada por um **PID discreto** com:

- **Ponderação do setpoint (PID 2-DOF)** (`beta`)
- **Derivada sobre a medição** com **filtro de 1ª ordem** (`alpha`)
- **Anti-windup por back-calculation** (`Kt`)
- **Logger CSV** na Serial (115200)
- **Segurança por ultrassom** (PING))) com **freio ativo** se o obstáculo estiver próximo

> Importante: para liberar a interrupção D3 ao tacômetro, **IN2** do L293D foi **movido do D3 para o D9**.

---

## Estrutura do Repositório

```
.
├─ README.md
├─ LICENSE                 # MIT (incluso)
├─ .gitignore              # ignora logs/ e builds; mantém logs/.gitkeep e docs/.keep
├─ src/
│  └─ main.ino             # (Arduino IDE) ou renomeie para src/main.cpp (PlatformIO)
├─ scripts/
│  └─ log_serial.py        # script Python para gravar CSV da serial
├─ requirements.txt        # pyserial
├─ logs/
│  └─ .gitkeep             # marcador p/ manter a pasta versionada
└─ docs/
   └─ .keep                # marcador p/ manter a pasta (ex.: artigo.pdf futuramente)
```

Para criar os marcadores:

- Linux/macOS:
```bash
mkdir -p logs docs && touch logs/.gitkeep docs/.keep
```

- Windows (PowerShell):
```powershell
mkdir logs, docs -Force
ni logs/.gitkeep -ItemType File -Force | Out-Null
ni docs/.keep -ItemType File -Force | Out-Null
```

---

## Hardware (BOM)

- 1x Arduino UNO (ou compatível)
- 1x CI L293D
- 2x Motores DC
- 1x Sensor de rotação (IR refletivo ou Hall) com saída digital
- 1x Sensor ultrassônico **PING)))** (1 fio SIG)
- 1x Buzzer (5 V)
- 2x LEDs (verde/vermelho) + resistores 220 Ω
- Fonte para motores (6–12 V) + **GND comum com o Arduino**
- Protoboard e jumpers

---

## Pinout / Ligações

### Arduino → L293D
- **EN1/2** → D5 (PWM)  
- **IN1** → D2  
- **IN2** → **D9**  *(movido do D3 para D9)*  
- **OUT1/OUT2** → Motor A

- **EN3/4** → D6 (PWM)  
- **IN3** → D4  
- **IN4** → D7  
- **OUT3/OUT4** → Motor B

- **Vcc1 (lógica)** → 5 V do Arduino  
- **Vcc2 (potência)** → Fonte dos motores (6–12 V)  
- **GND** → GND comum (Arduino + Fonte motores)

> O L293D possui diodos internos de proteção.

### Sensores e atuadores
- **Tacômetro (Hall/IR)**: Sinal → **D3 (INT1)**, Vcc 5 V, GND  
- **Ultrassom PING)))**: **SIG → D8** (o código alterna entre saída/entrada)  
- **LED vermelho** → D12 (com resistor)  
- **LED verde** → D11 (com resistor)  
- **Buzzer** → D13 (com resistor, se necessário)

---

## Parâmetros Principais (no código)

```cpp
#define USE_SAFETY_ULTRA 1  // 1=ativa freio por ultrassom; 0=desativa
const int   FREIO_CM     = 35;
const int   DIST_OK_CM   = 100;

const int   SETPOINT_RPM = 1200;
const float Kp = 0.8f, Ki = 0.15f, Kd = 0.02f;
const float beta  = 1.0f;   // ponderação do setpoint (2-DOF)
const float alpha = 0.90f;  // filtro da derivada (0<alpha<1)
const float Kt    = 0.20f;  // anti-windup (back-calculation)

const uint16_t Ts_ms = 50;  // período do PID (ms)
const int PWM_MAX    = 255;

const int PULSOS_POR_VOLTA = 1; // ajuste conforme seu disco/ímã
```

---

## Como Compilar e Gravar

### Arduino IDE
1. Instale o Arduino IDE 2.x.  
2. **Board:** Arduino UNO.  
3. **Porta** correta.  
4. **Serial**: 115200 baud.  
5. Abra `src/main.ino` (ou cole o código em um sketch com o mesmo nome da pasta).  
6. **Verificar** (✓) e **Carregar** (→).

### PlatformIO (VS Code)
1. `pip install platformio` ou use a extensão do VS Code.  
2. `pio init --board uno`  
3. Coloque o código em `src/main.cpp` (inclui `#include <Arduino.h>`).  
4. Compilar: `pio run`  
5. Gravar: `pio run -t upload`  
6. Monitor serial: `pio device monitor -b 115200`

---

## Como Usar

1. Energize o Arduino e a fonte dos motores (**GND comum**).  
2. Abra o **Serial Monitor** ou **Serial Plotter** (115200).  
3. O loop:
   - mede **RPM** pelo tacômetro em D3;
   - roda o **PID** a cada `Ts_ms`;
   - aplica **PWM** nos pinos **5 e 6** (valor igual aos dois motores);
   - monitora **distância** (se habilitado) e executa **freio ativo** ao cruzar `FREIO_CM`;
   - imprime **CSV**: `t_ms,set_rpm,rpm,pwm,dist_cm`.

---

## Aquisição de Dados via Python (opcional)

Para registrar automaticamente o CSV vindo da serial:

### Requisitos
- Python 3.9+
- Instale dependências:
```bash
pip install -r requirements.txt
```

### Uso
Windows:
```bash
python scripts/log_serial.py --port COM3 --baud 115200 --outfile logs/ensaio.csv --echo
```

Linux (ex.: `/dev/ttyACM0` ou `/dev/ttyUSB0`):
```bash
python scripts/log_serial.py --port /dev/ttyACM0 --baud 115200 --outfile logs/ensaio.csv --echo
```

Opções úteis:
- `--echo`: imprime as linhas no console.  
- `--no-header`: não aguarda o cabeçalho do Arduino.  
- `--wait-header 8.0`: tempo, em segundos, esperando o cabeçalho.

---

## O que o Código Faz

1. **RPM**: `onPulse()` mede o período entre pulsos via `micros()`; `rpmMedida()` converte para RPM e aplica EWMA para suavizar.  
2. **PID 2-DOF**: `pidStep(r,y)` usa proporcional com **β·r − y**, derivada sobre a medição filtrada, e integrador com **anti-windup** por back-calculation.  
3. **Ação de controle**: `aplicarPWM(u)` satura em 0–255, seta direção e aplica PWM nos pinos 5 e 6.  
4. **Segurança**: se `USE_SAFETY_ULTRA` e `dist <= FREIO_CM`, aplica **freio curto**, LED vermelho e buzzer, zera integrador e loga com `pwm=0`.  
5. **Telemetria**: imprime `t_ms,set_rpm,rpm,pwm,dist_cm` a cada iteração.

---

## Sintonia do PID (sugestão prática)

1. **Desative** temporariamente o ultrassom (`USE_SAFETY_ULTRA 0`) para sintonizar.  
2. **Ajuste Kp** até resposta rápida **sem** oscilação sustentada.  
3. **Introduza Ki** para remover erro estacionário (sem provocar oscilação lenta).  
4. **Use Kd** para reduzir overshoot e acelerar acomodação (se ruído alto, aumente `alpha`).  
5. Se notar saturação prolongada (0/255), **aumente levemente `Kt`**.  
6. Reative o ultrassom e valide o freio de emergência.

Dicas:
- **`beta < 1`** pode suavizar mudanças de setpoint.  
- **`alpha → 1`** = derivada mais filtrada (menos ruído, porém mais lenta).  
- **`Ts_ms`** menor torna o controle mais responsivo, mas exige mais CPU.

---

## Tinkercad

- Simule no Tinkercad (substitua componentes não disponíveis).  
- Adicione aqui o link da sua simulação, caso público.

---

## Solução de Problemas

- **PWM não muda a velocidade**: confira **Vcc2**, **GND comum**, **EN1/2 (D5)** e **EN3/4 (D6)**.  
- **Sentido invertido**: ajuste `setDirecaoFrente()` ou inverta os fios do motor.  
- **RPM 0**: verifique alimentação do sensor, distância do refletor/ímã, `PULSOS_POR_VOLTA`, e se a interrupção dispara.  
- **Freio acionando sempre**: `FREIO_CM` alto, ruído no ultrassom; teste com `USE_SAFETY_ULTRA 0`.  
- **Serial ilegível**: 115200 baud.  
- **Oscilação/overshoot**: reduza `Kp` e/ou aumente `Kd`; aumente `alpha`; reavalie `Ki`.

---

## Licença

Este projeto está sob a **MIT License** (ver arquivo `LICENSE`).

---

## Código-Fonte

O código completo está em `src/main.ino` (Arduino IDE) ou `src/main.cpp` (PlatformIO).

> Mantenha o `.gitignore` para evitar que `logs/` e artefatos de build poluam os commits; os marcadores `logs/.gitkeep` e `docs/.keep` asseguram que as pastas existam no repositório.
