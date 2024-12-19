import serial
import time
import math

# Configuración del puerto serial
ser = serial.Serial('COM3', 9600)  # Reemplazar 'COM3' por el puerto correspondiente

def get_corriente():
    sumatoria = 0
    N = 0
    tiempo_inicial = time.time()

    while time.time() - tiempo_inicial < 0.5:  # Duración de 0.5 segundos
        # Simulamos la lectura del sensor
        voltaje_sensor = random.uniform(0, 1) * 1.1  # Ajustar el rango según el sensor
        corriente = voltaje_sensor * 30.0
        sumatoria += corriente**2
        N += 1

    corriente_rms = math.sqrt(sumatoria / N)
    return corriente_rms

while True:
    Irms = get_corriente()
    P = Irms * 220.0

    print(f"Irms: {Irms:.3f} A, Potencia: {P:.3f} W")
    time.sleep(1)