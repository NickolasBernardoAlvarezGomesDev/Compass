import utime
import math
from machine import Pin
from compass.cowompaws import MPU

class KalmanFilter:
    def __init__(self, initial_estimate=0.0, initial_est_error=1.0, initial_measure_error=2.0):
        self.estimate = float(initial_estimate)
        self.gain = 0.5
        self.est_error = float(initial_est_error)
        self.measure_error = float(initial_measure_error)

    def calculate_kalman_gain(self):
        self.gain = self.est_error / (self.est_error + self.measure_error)

    def update_estimate(self, sensor_value):
        self.estimate = self.estimate + self.gain * (sensor_value - self.estimate)

    def calculate_estimate_error(self):
        self.est_error = (1 - self.gain) * self.est_error

    def apply(self, sensor_value):
        self.calculate_kalman_gain()
        self.update_estimate(sensor_value)
        self.calculate_estimate_error()
        return self.estimate

# --- Inicialização ---
mpu = MPU(scl=22, sda=21)

kf_sin = KalmanFilter(initial_estimate=0.0)
kf_cos = KalmanFilter(initial_estimate=1.0)  # começa apontando para o leste

print("Iniciando leitura com filtro de Kalman vetorial...\n")

while True:
    heading_raw = mpu.get_head()  # valor em radianos
    sin_val = math.sin(heading_raw)
    cos_val = math.cos(heading_raw)

    sin_filtered = kf_sin.apply(sin_val)
    cos_filtered = kf_cos.apply(cos_val)

    # Recalcula ângulo filtrado
    filtered_heading = math.atan2(sin_filtered, cos_filtered)
    if filtered_heading < 0:
        filtered_heading += 2 * math.pi

    print(
        "Heading cru: {:.2f}° | Filtrado: {:.2f}°".format(
            math.degrees(heading_raw),
            math.degrees(filtered_heading)
        )
    )

    utime.sleep_ms(50)

