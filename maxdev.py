import utime
from machine import I2C, Pin
#from compass.mpu9250 import MPU9250
from compass.cowompaws import MPU
import math
## filtros
#from filters.spike import spike

class KalmanFilter:
    def __init__(self, initial_estimate=0.0, initial_est_error=1.0, initial_measure_error=2.0):
        self.estimate = float(initial_estimate)
        self.gain = 0.0
        self.est_error = float(initial_est_error)
        self.measure_error = float(initial_measure_error)

    def calculate_kalman_gain(self):
        self.gain = self.est_error / (self.est_error + self.measure_error)

    def update_estimate(self, sensor_value):
        sensor_value = float(sensor_value)  # conversão garantida
        self.estimate = self.estimate + self.gain * (sensor_value - self.estimate)

    def calculate_estimate_error(self):
        self.est_error = (1 - self.gain) * self.est_error

    def apply(self, sensor_value):
        self.calculate_kalman_gain()
        self.update_estimate(sensor_value)
        self.calculate_estimate_error()
        return self.estimate


# --- Inicialização dos dispositivos ---
i2c = I2C(scl=Pin(22), sda=Pin(21))
mpu = MPU(scl=22, sda=21)
kf = KalmanFilter(initial_estimate=0.0, initial_est_error=0.75, initial_measure_error=5.0)

print("Iniciando leitura com filtro de Kalman e rejeição de valores desviantes...\n")

# Parâmetro para rejeição de outliers (em radianos)
MAX_ALLOWED_DEVIATION = math.radians(20)  # 20 graus

# Inicializa valor anterior filtrado
previous_filtered_heading = 0.0

while True:
    heading_raw = mpu.get_head()

    try:
        heading_value = float(heading_raw)
    except Exception:
        print("Valor inválido de heading:", heading_raw)
        continue

    # Filtro de valores desviantes
    if abs(heading_value - previous_filtered_heading) > MAX_ALLOWED_DEVIATION:
        print("Valor desviado rejeitado: {:.2f}".format(heading_value * (180 / math.pi)))
        utime.sleep_ms(50)
        continue

    # Aplica filtro de Kalman
    filtered_heading = kf.apply(heading_value)
    previous_filtered_heading = filtered_heading  # atualiza valor anterior

    # Converte para graus
    deg_head = filtered_heading * (180 / math.pi)
    deg_raw = heading_value * (180 / math.pi)

    print("Heading cru: {:.2f} | Filtrado: {:.2f}".format(deg_raw, deg_head))

    utime.sleep_ms(50)
