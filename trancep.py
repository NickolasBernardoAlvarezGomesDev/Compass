##### transmissão de dados head #####

import utime
import math
from machine import Pin, UART
from compass.cowompaws import MPU

# --- Classe do Filtro de Kalman ---
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

# --- Inicialização do MPU e UART ---
mpu = MPU(scl=22, sda=21)
uart = UART(1, baudrate=115200, tx=Pin(17))  # UART1 TX em GPIO17

# Filtros vetoriais para seno e cosseno
kf_sin = KalmanFilter(initial_estimate=0.0)
kf_cos = KalmanFilter(initial_estimate=1.0)

print("Iniciando leitura com envio serial...\n")

while True:
    # --- Leitura do heading ---
    heading_raw = mpu.get_head()  # valor em radianos
    sin_val = math.sin(heading_raw)
    cos_val = math.cos(heading_raw)

    # --- Filtragem ---
    sin_filtered = kf_sin.apply(sin_val)
    cos_filtered = kf_cos.apply(cos_val)

    # --- Recalcula ângulo filtrado ---
    filtered_heading = math.atan2(sin_filtered, cos_filtered)
    if filtered_heading < 0:
        filtered_heading += 2 * math.pi

    # Converte para graus
    heading_deg = math.degrees(filtered_heading)

    # --- Envia via UART ---
    msg = "{:.2f}\n".format(heading_deg)  # adiciona quebra de linha para facilitar leitura
    uart.write(msg)

    # --- Também imprime localmente ---
    print("Heading cru: {:.2f}° | Filtrado: {:.2f}°".format(
        math.degrees(heading_raw),
        heading_deg
    ))

    utime.sleep_ms(50)


########## receptor ##########

from machine import UART, Pin
import utime

# Inicializa UART na mesma taxa de baud
uart = UART(1, baudrate=115200, rx=Pin(16))  # RX em GPIO16

buffer = b""

while True:
    if uart.any():
        buffer += uart.read()
        if b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            try:
                heading = float(line.decode().strip())
                print("Recebido heading filtrado: {:.2f}°".format(heading))
            except ValueError:
                print("Erro ao converter:", line)
    utime.sleep_ms(20)
