##### TRANSMISSÃO DE DADOS HEADING — KF + TinyGRU p/ ajuste de R (100% offline) #####

import utime, math
from machine import Pin, UART
from compass.cowompaws import MPU

# ===================== Utilitários =====================
def _wrap_pi(a):
    # wrap para (-pi, pi]
    while a <= -math.pi:
        a += 2*math.pi
    while a > math.pi:
        a -= 2*math.pi
    return a

def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

# ===================== Tiny GRU (1 camada, hidden=8) =====================
# Forma esperada:
#  x: (in_dim,)
#  h: (hidden_dim,)
#  z = sigmoid(Wz x + Uz h + bz)
#  r = sigmoid(Wr x + Ur h + br)
#  n = tanh   (Wn x + Un (r ⊙ h) + bn)
#  h' = (1 - z) ⊙ n + z ⊙ h

def _sigmoid(x): return 1.0/(1.0+math.exp(-x)) if x >= 0 else math.exp(x)/(1.0+math.exp(x))
def _tanh(x):
    # tanh estável numérica
    e1 = math.exp(x) if x < 10 else math.exp(10)
    e2 = math.exp(-x) if x > -10 else math.exp(10)
    return (e1 - e2) / (e1 + e2)

class TinyGRU:
    def __init__(self, in_dim=5, hidden=8):
        self.in_dim = in_dim
        self.hidden = hidden
        self.h = [0.0]*hidden

        # --------- PESOS PLACEHOLDER (trocar por pesos treinados) ----------
        # Inicializados para comportamento conservador (saída ~ neutra)
        # Cada matriz W? é [hidden][in_dim], U? é [hidden][hidden], b? é [hidden]
        def zeros(m,n): return [[0.0]*n for _ in range(m)]
        def tiny_eye(m): 
            M = zeros(m,m)
            for i in range(m): M[i][i] = 0.1
            return M

        self.Wz = zeros(self.hidden, self.in_dim)
        self.Wr = zeros(self.hidden, self.in_dim)
        self.Wn = zeros(self.hidden, self.in_dim)

        self.Uz = tiny_eye(self.hidden)
        self.Ur = tiny_eye(self.hidden)
        self.Un = tiny_eye(self.hidden)

        self.bz = [0.0]*self.hidden
        self.br = [0.0]*self.hidden
        self.bn = [0.0]*self.hidden

        # Camada de saída linear h -> y
        self.Wy = [0.05]*self.hidden
        self.by = 0.0

    def _mv(self, M, v):
        # M: [m][n], v: [n]
        out = []
        for row in M:
            s = 0.0
            for j, w in enumerate(row):
                s += w * v[j]
            out.append(s)
        return out

    def _vv(self, M, v):
        # M: [m][m], v: [m]
        out = []
        for i, row in enumerate(M):
            s = 0.0
            for j, w in enumerate(row):
                s += w * v[j]
            out.append(s)
        return out

    def forward(self, x):
        # x: lista de tamanho in_dim
        # gates
        Wz_x = self._mv(self.Wz, x)
        Wr_x = self._mv(self.Wr, x)
        Wn_x = self._mv(self.Wn, x)

        Uz_h = self._vv(self.Uz, self.h)
        Ur_h = self._vv(self.Ur, self.h)

        z = [_sigmoid(Wz_x[i] + Uz_h[i] + self.bz[i]) for i in range(self.hidden)]
        r = [_sigmoid(Wr_x[i] + Ur_h[i] + self.br[i]) for i in range(self.hidden)]

        # n = tanh(Wn x + Un (r ⊙ h) + bn)
        rh = [r[i]*self.h[i] for i in range(self.hidden)]
        Un_rh = self._vv(self.Un, rh)
        n = [_tanh(Wn_x[i] + Un_rh[i] + self.bn[i]) for i in range(self.hidden)]

        # h' = (1 - z) ⊙ n + z ⊙ h
        h_new = [(1.0 - z[i])*n[i] + z[i]*self.h[i] for i in range(self.hidden)]
        self.h = h_new

        # saída escalar
        y = self.by
        for i in range(self.hidden):
            y += self.Wy[i]*self.h[i]
        return y

# ===================== Kalman Adaptativo com R escalado pela GRU =====================
class AdaptiveKalman:
    def __init__(self, est0=0.0, est_err0=1.0, meas_err0=2.0):
        self.estimate = float(est0)
        self.est_error = float(est_err0)
        self.base_meas_error = float(meas_err0)
        self.meas_error = float(meas_err0)
        self.gain = 0.5

        # GRU minúscula para prever fator de R
        self.gru = TinyGRU(in_dim=5, hidden=8)
        self.r_smooth = 1.0   # suavização do multiplicador de R

        # histórico simples
        self.prev_raw_heading = None
        self.prev_filtered = None

    def _update_gain(self):
        self.gain = self.est_error / (self.est_error + self.meas_error)

    def _innovation(self, value):
        return _wrap_pi(value - self.estimate)

    def _features(self, heading_raw, innov, dhead_raw, gyro_mod=None, acc_mod=None, mag_mod=None):
        # 5 features leves (normalizadas)
        # 1) |innov| em rad (0..pi) -> [0..1] dividindo por pi
        f0 = abs(innov) / math.pi
        # 2) |dhead_raw|/pi (variação instantânea do heading)
        f1 = abs(dhead_raw) / math.pi
        # 3) gyro_mod (se disponível) normalizado ~200 dps -> rad/s ~3.49 -> divide por 3.5
        f2 = 0.0 if gyro_mod is None else _clamp(gyro_mod/3.5, 0.0, 1.0)
        # 4) acc_mod relativo a g (~9.81) -> |acc|/g (clamp 0..2)
        f3 = 0.0 if acc_mod is None else _clamp(acc_mod/9.81, 0.0, 2.0)/2.0
        # 5) desvio do módulo magnético em relação a 1 (esperado normalizado) -> |mag|-1 (clamp 0..1)
        # se não houver normalização do mag, fica 0.0 (conservador)
        f4 = 0.0 if mag_mod is None else _clamp(abs(mag_mod - 1.0), 0.0, 1.0)
        return [f0, f1, f2, f3, f4]

    def _scale_R_from_gru(self, feats):
        # y -> [0,1] via sigmoid “implícito” da GRU; mapeia para [0.5, 3.0]
        y = 0.0 + feats[0]*0.0  # evita lint; y vem da GRU
        y = self.gru.forward(feats)
        # squash final
        # mapeia y para 0..1 via sigmoide e para [0.5, 3.0]
        sigma = _sigmoid(y)
        scale = 0.5 + 2.5*sigma
        # suavização (EMA) para evitar “pulos” no R
        self.r_smooth = 0.9*self.r_smooth + 0.1*scale
        return self.r_smooth

    def apply(self, sensor_value, gyro_mod=None, acc_mod=None, mag_mod=None):
        # d(head)_raw
        if self.prev_raw_heading is None:
            dhead_raw = 0.0
        else:
            dhead_raw = _wrap_pi(sensor_value - self.prev_raw_heading)

        innov = self._innovation(sensor_value)
        feats = self._features(sensor_value, innov, dhead_raw, gyro_mod, acc_mod, mag_mod)
        r_scale = self._scale_R_from_gru(feats)

        # Atualiza R escalado pela GRU
        self.meas_error = _clamp(self.base_meas_error * r_scale, 0.05, 20.0)

        # Passo de correção do Kalman 1D
        self._update_gain()
        self.estimate += self.gain * innov
        # Erro de estimação com pequeno Q para manter adaptabilidade
        self.est_error = (1.0 - self.gain)*self.est_error + 0.001

        self.prev_raw_heading = sensor_value
        self.prev_filtered = self.estimate
        return self.estimate

# ===================== Inicialização de sensores/serial =====================
mpu = MPU(scl=22, sda=21)
uart = UART(1, baudrate=115200, tx=Pin(17))  # UART1 TX em GPIO17

kf_sin = AdaptiveKalman(est0=0.0, est_err0=1.0, meas_err0=2.0)
kf_cos = AdaptiveKalman(est0=1.0, est_err0=2.0, meas_err0=2.0)

print("Iniciando KF + TinyGRU para heading...\n")

# ===================== Loop principal =====================
while True:
    # --- Leitura bruta de heading (rad) ---
    head_raw = mpu.get_head()

    # Tentativa de obter sinais auxiliares (opcional)
    gyro_mod = acc_mod = mag_mod = None
    try:
        # Se a tua lib expõe métodos, substitui pelos corretos:
        # gx,gy,gz (rad/s), ax,ay,az (m/s^2), mx,my,mz (já calibrado e normalizado?  -> ajuste abaixo)
        gx, gy, gz = mpu.get_gyro()       # exemplo; senão, cairá no except
        ax, ay, az = mpu.get_accel()
        mx, my, mz = mpu.get_mag()

        gyro_mod = math.sqrt(gx*gx + gy*gy + gz*gz)
        acc_mod  = math.sqrt(ax*ax + ay*ay + az*az)
        # Para o magnético, ideal normalizar pelo módulo médio local.
        # Se a tua calibração já normaliza (|m|~1), deixa como está; do contrário, normaliza por um valor típico.
        mnorm = math.sqrt(mx*mx + my*my + mz*mz)
        # exemplo: assume módulo típico = m0 (ajuste com teu dado/calibração)
        m0 = 1.0 if mnorm == 0 else mnorm  # fallback ingênuo: trata o instante como referência
        mag_mod = mnorm / (m0 + 1e-6)
    except:
        pass  # se tua API não tiver, a GRU usará só inov/dhead

    # seno/cosseno
    s = math.sin(head_raw)
    c = math.cos(head_raw)

    # KF + GRU ajustando R
    s_f = kf_sin.apply(s, gyro_mod=gyro_mod, acc_mod=acc_mod, mag_mod=mag_mod)
    c_f = kf_cos.apply(c, gyro_mod=gyro_mod, acc_mod=acc_mod, mag_mod=mag_mod)

    # Ângulo filtrado (0..2π)
    head_f = math.atan2(s_f, c_f)
    if head_f < 0:
        head_f += 2*math.pi

    head_deg = math.degrees(head_f)

    # Envio
    uart.write("{:.2f}\n".format(head_deg))

    # Log local (inclui escala de R média dos dois ramos)
    r_avg = 0.5*(kf_sin.meas_error + kf_cos.meas_error)
    print("Cru: {:.2f}° | Filt: {:.2f}° | R≈{:.2f}".format(math.degrees(head_raw), head_deg, r_avg))

    utime.sleep_ms(50)
