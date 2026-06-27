import HAL
import WebGUI
import math
import random
import time

# Parametros
NUM_PARTICLES = 500
BEAMS = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 179]
MOTION_NOISE_XY = 0.03
MOTION_NOISE_YAW = 0.03
# No momento, o robo nao se move
ANGULAR_VEL = 0 
LINEAR_VEL = 0
SENSOR_SIGMA = 0.20

# Fracao de particulas substituidas por posicoes aleatorias a cada ciclo
RANDOM_INJECTION_RATIO = 0.03

# Frequencia de redenrizacao
RENDER_EVERY_N = 1

# Frequencia da mensagem de debug
DEBUG_EVERY_N = 5

# Resampleia a cada N ciclos, nao todo ciclo.
RESAMPLE_EVERY_N = 4

MAP_URL = "/resources/exercises/montecarlo_laser_loc/images/mapgrannyannie.png"

# Mapa 
map_img = WebGUI.getMap(MAP_URL)

# preto = obstaculo, branco = espaço livre
occupancy = map_img[:, :, 0] < 0.5

HEIGHT, WIDTH = occupancy.shape

_corner_a = WebGUI.mapToPose(0, 0, 0)
_corner_b = WebGUI.mapToPose(WIDTH - 1, HEIGHT - 1, 0)

WORLD_MIN_X = min(_corner_a[0], _corner_b[0])
WORLD_MAX_X = max(_corner_a[0], _corner_b[0])
WORLD_MIN_Y = min(_corner_a[1], _corner_b[1])
WORLD_MAX_Y = max(_corner_a[1], _corner_b[1])

METERS_PER_PIXEL_X = (WORLD_MAX_X - WORLD_MIN_X) / float(WIDTH)
METERS_PER_PIXEL_Y = (WORLD_MAX_Y - WORLD_MIN_Y) / float(HEIGHT)

RAY_STEP = min(METERS_PER_PIXEL_X, METERS_PER_PIXEL_Y)

MAX_RANGE = min(
    math.hypot(WORLD_MAX_X - WORLD_MIN_X, WORLD_MAX_Y - WORLD_MIN_Y),
    10.0
)

print("[SETUP] Mundo: x[", WORLD_MIN_X, ",", WORLD_MAX_X, "] y[", WORLD_MIN_Y, ",", WORLD_MAX_Y, "]")
print("[SETUP] RAY_STEP:", RAY_STEP, "MAX_RANGE:", MAX_RANGE)



# Helpers

def world_to_map(x, y):
    """(x, y) do mundo -> (mx, my) em pixel. None se a conversao falhar."""
    try:
        mx, my, _ = WebGUI.poseToMap(x, y, 0)
        return int(mx), int(my)
    except Exception:
        return None


def is_free(x, y):
    """True se (x, y) do mundo cai numa celula livre dentro do mapa."""
    cell = world_to_map(x, y)
    if cell is None:
        return False
    mx, my = cell
    if mx < 0 or mx >= WIDTH or my < 0 or my >= HEIGHT:
        return False
    return not occupancy[mx, my]


def random_free_cell():
    """Sorteia uma posicao (x, y, yaw) aleatoria em celula livre do mapa."""
    while True:
        mx = random.randint(0, WIDTH - 1)
        my = random.randint(0, HEIGHT - 1)
        if not occupancy[mx, my]:
            break
    wx, wy, _ = WebGUI.mapToPose(mx, my, 0)
    yaw = random.uniform(-math.pi, math.pi)
    return wx, wy, yaw


def is_valid(v):
    """True se v nao for NaN nem infinito."""
    return not (math.isnan(v) or math.isinf(v))


def normalize_angle(a):
    """Normaliza um angulo para o intervalo [-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# Sensor model (ray casting)

def ray_cast(x, y, angle):
    """Distancia esperada do laser a partir de (x, y, angle), marchando
    em passos de RAY_STEP até bater em obstaculo ou atingir MAX_RANGE."""
    dist = 0.0
    while dist < MAX_RANGE:
        ex = x + dist * math.cos(angle)
        ey = y + dist * math.sin(angle)

        cell = world_to_map(ex, ey)
        if cell is None:
            return MAX_RANGE
        mx, my = cell
        if mx < 0 or mx >= WIDTH or my < 0 or my >= HEIGHT:
            return MAX_RANGE
        if occupancy[mx, my]:
            return dist

        dist += RAY_STEP
    return MAX_RANGE


def sensor_weight(x, y, yaw, laser):
    """
    Calcula o peso de uma partícula utilizando um modelo Gaussiano.

    Para cada feixe do laser:
      - calcula a distância esperada via ray casting;
      - compara com a distância medida;
      - calcula a probabilidade dessa diferença ocorrer;
      - multiplica essa probabilidade ao peso total.
    """

    if not is_free(x, y):
        return 0.0

    weight = 1.0

    for beam in BEAMS:
        measured = laser.values[beam]

        if not is_valid(measured):
            continue

        angle = yaw + math.radians(beam - 90)
        expected = ray_cast(x, y, angle)
        error = measured - expected
        likelihood = math.exp(-(error * error) / (2 * SENSOR_SIGMA * SENSOR_SIGMA))
        weight *= likelihood

    if not is_valid(weight) or weight <= 0.0:
        return 0.0

    return weight


# Filtro de particulas (MCL)

def init_particles():
    """Cria a nuvem inicial de particulas, todas em celulas livres e com peso uniforme."""
    result = []
    for _ in range(NUM_PARTICLES):
        wx, wy, yaw = random_free_cell()
        result.append([wx, wy, yaw, 1.0 / NUM_PARTICLES])
    return result


def normalize_weights(particles):
    """Se todos os pesos colapsarem a zero, reinicializa a nuvem inteira
    em posicoes livres, em vez de so igualar pesos (manteria posicoes mortas)."""
    total = sum(p[3] for p in particles)

    if total <= 0 or not is_valid(total):
        for p in particles:
            wx, wy, yaw = random_free_cell()
            p[0], p[1], p[2], p[3] = wx, wy, yaw, 1.0 / len(particles)
        return

    for p in particles:
        p[3] /= total


def estimate_pose(particles):
    """Estima a pose do robo como a media ponderada das particulas.

    O yaw usa media circular (via seno/cosseno), nao media aritmetica
    direta, pois angulos perto de +-pi nao podem ser somados linearmente.
    """
    x = y = sin_sum = cos_sum = 0.0
    for px, py, pyaw, pw in particles:
        x += px * pw
        y += py * pw
        sin_sum += math.sin(pyaw) * pw
        cos_sum += math.cos(pyaw) * pw
    yaw = math.atan2(sin_sum, cos_sum)
    return x, y, yaw


def systematic_resample(particles):
    """Resampling sistematico: sorteia novas particulas proporcionalmente
    ao peso (low variance resampling), e reseta todos os pesos para uniforme."""
    n = len(particles)
    cumulative = []
    s = 0.0
    for p in particles:
        s += p[3]
        cumulative.append(s)

    step = 1.0 / n
    r = random.uniform(0, step)
    i = 0
    new_particles = []

    for m in range(n):
        u = r + m * step
        while i < n - 1 and cumulative[i] < u:
            i += 1
        chosen = particles[i]
        new_particles.append([chosen[0], chosen[1], chosen[2], 1.0 / n])

    return new_particles


def inject_random_particles(particles, ratio):
    """Substitui uma fracao das particulas por posicoes aleatorias livres,
    para manter diversidade e permitir escapar de convergencia fora da regiao correta."""
    n = len(particles)
    num_to_replace = max(1, int(n * ratio))
    indices = random.sample(range(n), num_to_replace)

    for idx in indices:
        wx, wy, yaw = random_free_cell()
        particles[idx][0] = wx
        particles[idx][1] = wy
        particles[idx][2] = yaw
        particles[idx][3] = 1.0 / n

    return particles

# MAIN LOOP

particles = init_particles()
print("Particulas inicializadas:", len(particles))

HAL.setV(LINEAR_VEL)
HAL.setW(ANGULAR_VEL)

last_time = time.time()
iteration = 0

while True:
    iteration += 1

    # Motion update: com LINEAR_VEL=ANGULAR_VEL=0, isso so aplica ruido
    # gaussiano (a particula "treme" no lugar, sem deslocamento real).
    now = time.time()
    dt = now - last_time
    last_time = now
    if not is_valid(dt) or dt <= 0:
        dt = 0.05
    dyaw = ANGULAR_VEL * dt
    for p in particles:
        p[0] += LINEAR_VEL * dt * math.cos(p[2]) + random.gauss(0, MOTION_NOISE_XY)
        p[1] += LINEAR_VEL * dt * math.sin(p[2]) + random.gauss(0, MOTION_NOISE_XY)
        p[2] = normalize_angle(p[2] + dyaw + random.gauss(0, MOTION_NOISE_YAW))

    # Pesos ACUMULAM (multiplicam) entre resamples, em vez de serem
    # substituidos a cada ciclo.

    laser = HAL.getLaserData()

    for p in particles:
        p[3] *= sensor_weight(p[0], p[1], p[2], laser)

    normalize_weights(particles)

    # resaple a cada N ciclos

    if iteration % RESAMPLE_EVERY_N == 0:
        particles = systematic_resample(particles)

    # ---------------- ESTIMATE ----------------
    # Calculada antes da injecao aleatoria, para nao ser distorcida pelas
    # particulas recem-injetadas (ainda nao avaliadas pelo sensor model).

    x_est, y_est, yaw_est = estimate_pose(particles)

    # ---------------- RANDOM INJECTION ----------------

    particles = inject_random_particles(particles, RANDOM_INJECTION_RATIO)

    # ---------------- DEBUG ----------------

    if iteration % DEBUG_EVERY_N == 0:
        real_pose = HAL.getPose3d()
        error_dist = math.hypot(x_est - real_pose.x, y_est - real_pose.y)

        xs = [p[0] for p in particles]
        ys = [p[1] for p in particles]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        std_x = (sum((v - mean_x) ** 2 for v in xs) / len(xs)) ** 0.5
        std_y = (sum((v - mean_y) ** 2 for v in ys) / len(ys)) ** 0.5

        print(
            "[", iteration, "] est=(", round(x_est, 2), ",", round(y_est, 2),
            ") real=(", round(real_pose.x, 2), ",", round(real_pose.y, 2),
            ") erro=", round(error_dist, 2),
            "dispersao=(", round(std_x, 2), ",", round(std_y, 2), ")"
        )

    # ---------------- RENDER ----------------

    if iteration % RENDER_EVERY_N == 0:
        WebGUI.showParticles([[p[0], p[1], p[2], p[3]] for p in particles])
        WebGUI.showPosition(x_est, y_est, yaw_est)
