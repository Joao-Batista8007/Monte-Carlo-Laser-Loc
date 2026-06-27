# Monte Carlo Localization (MCL) com Laser — JdeRobot RoboticsAcademy

Implementação de um filtro de partículas (Monte Carlo Localization) para
localização de um robô móvel a partir de leituras de um sensor laser,
desenvolvido para o exercício `montecarlo_laser_loc` da plataforma
[JdeRobot RoboticsAcademy](https://jderobot.github.io/RoboticsAcademy/).

## Visão geral

O robô não sabe sua posição inicial no mapa. O algoritmo mantém uma
população de **partículas** — cada uma representando uma hipótese de pose
`(x, y, yaw)` — e usa as leituras do laser para, a cada ciclo, avaliar quais
hipóteses são mais plausíveis e redistribuir a população em torno delas.
Com o tempo, a nuvem de partículas converge para a pose real do robô.

O algoritmo segue o ciclo clássico de um filtro de partículas:

1. **Predição (motion update):** cada partícula se desloca segundo um
   modelo de movimento com ruído.
2. **Correção (sensor update):** cada partícula simula, via *ray casting*
   no mapa, a leitura de laser que teria na sua posição hipotética, e
   compara com a leitura real do robô. A diferença determina o peso da
   partícula.
3. **Resampling:** a população é redistribuída, favorecendo partículas de
   peso alto.
4. **Estimativa:** a pose do robô é estimada pela média ponderada das
   partículas.

## Estrutura do código

| Bloco | Função |
|---|---|
| Setup do mapa | Carrega `mapgrannyannie.png`, converte em grade de ocupação, calcula limites do mundo e resolução metros/pixel |
| `world_to_map`, `is_free`, `random_free_cell` | Conversão de coordenadas e checagem de células livres |
| `ray_cast` | Simula a distância que o laser mediria a partir de uma pose hipotética |
| `sensor_weight` | Calcula o peso de uma partícula comparando leitura real vs. simulada (modelo gaussiano) |
| `init_particles`, `normalize_weights`, `estimate_pose`, `systematic_resample`, `inject_random_particles` | Ciclo de vida do filtro de partículas |
| Loop principal | Executa predição → correção → resampling → estimativa → visualização a cada ciclo |

## Como rodar

1. Abra o exercício `montecarlo_laser_loc` no JdeRobot RoboticsAcademy.
2. Cole o conteúdo de `mcl_v4_doc.py` no editor de código do exercício.
3. Execute. A nuvem de partículas e a posição estimada aparecem no WebGUI.

## Parâmetros principais

| Parâmetro | Descrição | Valor atual |
|---|---|---|
| `NUM_PARTICLES` | Tamanho da população de partículas | 500 |
| `BEAMS` | Índices dos feixes do laser usados (0–179, passo de 15°) | 13 feixes |
| `SENSOR_SIGMA` | Desvio padrão do modelo gaussiano de erro do sensor | 0.20 |
| `MOTION_NOISE_XY`, `MOTION_NOISE_YAW` | Ruído aplicado ao movimento das partículas | 0.03 |
| `RANDOM_INJECTION_RATIO` | Fração de partículas substituídas por posições aleatórias a cada ciclo | 0.03 |
| `RESAMPLE_EVERY_N` | Resampling executado a cada N ciclos (não todo ciclo) | 4 |
| `LINEAR_VEL`, `ANGULAR_VEL` | Velocidade comandada ao robô | 0 (ver limitação abaixo) |

## Convenções confirmadas da plataforma

Durante o desenvolvimento, duas convenções da API foram confirmadas
experimentalmente (não documentadas explicitamente, ou facilmente
confundidas):

- **Indexação do mapa de ocupação:** `occupancy[mx, my]`, não
  `occupancy[my, mx]`. Confirmado comparando a célula da posição real do
  robô (`HAL.getPose3d()`, sempre em espaço livre) nas duas convenções.
- **Convenção do laser:** o feixe de índice `90` corresponde à frente do
  robô; `0` é a lateral direita; `180` é a lateral esquerda. O ângulo de
  um feixe `i`, relativo ao yaw do robô, é `radians(i - 90)`.

## Limitação conhecida: robô estacionário

Nesta versão, `LINEAR_VEL = ANGULAR_VEL = 0`: o robô permanece parado
durante a localização, e o motion update das partículas aplica apenas
ruído gaussiano (sem deslocamento direcional).

Isso não invalida o algoritmo como MCL — com velocidade zero, a predição
correta é justamente "a pose não muda, exceto por incerteza", e o ciclo de
correção (ray casting + resampling) continua operando normalmente,
convergindo a nuvem para a pose real apenas com a informação do sensor.

A limitação aparece quando o robô se movimenta de fato: como o motion
update por partícula não está acoplado a um deslocamento real
consistente, a nuvem não acompanha o robô de forma confiável, e a
convergência se torna instável — a estimativa pode aproximar-se
corretamente por um tempo e depois divergir, especialmente perto de
regiões do mapa com geometria localmente simétrica (cantos e quinas que
produzem leituras de laser muito parecidas entre si, um efeito conhecido
como *perceptual aliasing*). Aumentar o número de partículas e a densidade
de feixes do laser deve mitigar esse efeito, ao custo de mais processamento
por ciclo — não testado exaustivamente aqui por limitação de hardware.

## Referência teórica

Carpin, S. *Mobile Robotics: Theory and Practice*, UC Merced — Capítulo 8
(Estimation and Filtering, seção 8.5 — Particle Filters) e Capítulo 9
(Localization and Mapping).
