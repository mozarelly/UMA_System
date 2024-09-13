#Importar librerías

import time
import board
import busio
import glob
import os
import datetime
import pytz

import RPi.GPIO as GPIO

#Módulos ADC
import Adafruit_MCP4725
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

#Inicializar variables
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')

#Codigo temperatura

def read_temp_raw(device_file):
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp_c(device_file):
    lines = read_temp_raw(device_file)
    while True:
        if lines[0].strip()[-3:] == 'YES':
            break
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c

def get_temperature(device_folder):
    device_file = device_folder + '/w1_slave'
    return read_temp_c(device_file)

def sensor_temperatura(device_folder):
    # Diccionario para guardar listas de temperatura
    # temperature_lists = {}
    current_temperatures=()



    for device in (device_folder):
        temperature = get_temperature(device)
        current_temperatures.append(temperature)
        #print(f"Temperature of sensor {i}: {temperature:.2f} C")
        """
        # Crear una nueva lista para cada sensor si no existe
        if i not in temperature_lists:
            temperature_lists[i] = []
        """
        # Guardar el valor del sensor
        #temperature_lists[i].append(round(temperature,2))
    return current_temperatures

# Temperatura deseada
setpoint_temp = 22 # in degrees Celsius

i2c = busio.I2C(board.SCL, board.SDA)

#Modulos ADC
ads1 = ADS.ADS1115(i2c, address=0x48)

ads1.gain = 1
V_corriente_motor = AnalogIn(ads1, ADS.P0)

V_p_input = AnalogIn(ads1, ADS.P2)
V_p_output = AnalogIn(ads1, ADS.P3)

ads2 = ADS.ADS1115(i2c, address=0x49)

ads2.gain = 1
V_posicion_valvula = AnalogIn(ads2, ADS.P0)
V_rpm = AnalogIn(ads2, ADS.P1)

#Feedback válvula
V_control_valvula = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)

control_valvula=0
V_control_valvula.set_voltage(control_valvula)

GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(25, GPIO.OUT)  # Set GPIO pin 25 as output
GPIO.output(25, GPIO.LOW)

def efecto_hall():
    valores = []
    rpm=0
    for _ in range(1000):
        valores.append(V_rpm.voltage)
        time.sleep(0.06)   
    rpm = sum(valores)/len(valores)
    return rpm 


def parada_motor():
    print("TRAKA")
    return GPIO.output(25, GPIO.HIGH)


def corriente():
    #Corriente motor
    V_corriente=round(V_p_input.voltage,2)
    
    corriente_motor=round(V_corriente*30.0,2)

    #print(f"Corriente motor \n Voltaje:{V_corriente_motor.voltage}V \n Corriente: {corriente_motor}A")
    return corriente_motor, V_corriente
    

def presion():
    # y=mx+b

    m_p=38.67
    b_p=0
    
    #Suministro
    V_p_suministro=round(V_p_input.voltage,2)
    p_suministro=round(m_p*V_p_suministro+b_p,2)
    #print(f"Presion suministro agua \n Voltaje:{V_p_input.voltage}V \n Presion: {p_input}psi")


    V_p_retorno=round(V_p_output.voltage,2)
    p_retorno=round(m_p*V_p_retorno+b_p,2)
    #print(f"Presion retorno agua \n Voltaje:{V_p_output.voltage}V \n Presion: {p_output}psi")

    return p_suministro, V_p_suministro, p_retorno, V_p_retorno


def f_valvula():
    #y=mx+b
    feedback_voltaje=round(V_posicion_valvula.voltage,2)
    m_val=-29.94
    b_val=126.24
    posicion_valvula=round(m_val*feedback_voltaje+b_val,0)
    
    return max(0, min(100, posicion_valvula)), feedback_voltaje

def porcentaje_a_bits(porcentaje):
    # Calcula en numero de bits
    raw_bits = (porcentaje / 100) * 4096
    # Redondea al numero entero mas cercano
    closest_bits = round(raw_bits)
    # El numero de bits debe estar en el rango valido (0 to 4096)
    return max(0, min(4096, closest_bits))

def control_valvula(valor_deseado):
    return V_control_valvula.set_voltage(valor_deseado)

j=0


class UMA_sensor:
    
    
    def __init__(self):
        # Initialize any necessary attributes here (if needed)
        pass

    #Adquisición de datos
    def get_uma():
        print('--------------Estado de maquina----------')
        
        now = datetime.datetime.now(pytz.timezone('America/Caracas'))

        print("Ciclo", j, now)
        
        #Corriente motor
        corriente_motor, V_corriente=corriente()
                        
        #Presion
        p_suministro, V_p_suministro, p_retorno, V_p_retorno=presion()
        
        #Posicion valvula
        posicion_valvula, feedback_voltaje = f_valvula()
        
        #Lista de sensores temperaturas
        current_temperatures=sensor_temperatura(device_folder)

        on_uma=True
        rpm=efecto_hall()

        control_valvula=54
        V_control_valvula=2.5

        print("---Corriente en el motor---: \n ", corriente_motor, "A", V_corriente, "V \n --Presión:--- \n Suministro:", p_suministro, "psi", V_p_suministro,"V \n Retorno:",p_retorno,"psi", V_p_retorno,"V \n ---Válvula--- \n Posición:", posicion_valvula, "%", feedback_voltaje, "V \n ---Temperatura:--- \n T1",current_temperatures(1), "°C \n T2",current_temperatures(2), "°C \n T3",current_temperatures(3),"°C \n T4",current_temperatures(4), "°C \n Promedio efecto Hall:", rpm, "\n \n")
        # print("T1",temperature_lists[1], "T2",temperature_lists[1], "T3",temperature_lists[3],"T4",temperature_lists[4] )
        
        return {"tmp1": current_temperatures(1), "tmp2": current_temperatures(2), "tmp3": current_temperatures(3), "tmp4": current_temperatures(4), "pre_suministro": p_suministro, "pre_retorno": p_retorno, "c_motor": corriente_motor, "prom_efecto_hall": rpm, "encendido": on_uma, "p_input_valvula": posicion_valvula, "input_valvula": feedback_voltaje, "p_output_valvula": control_valvula, "output_valvula": V_control_valvula} 
        """"
        #Ajuste valvula
        if valores_temp_salida[j]>setpoint_temp:
            #x=my+b
            print(f"Nada")
        else:
            #x=my+b
            control_valvula=-0.0499*posicion_valvula+4.99
            
            V_control_valvula.set_voltage(4069)
            print(f"Posicion deseada valvula \n Voltaje:{control_valvula}V \n Apertura: {posicion_valvula}%")
            
                """

        #j=1+j
        #time.sleep(10)

#except KeyboardInterrupt:
#    pass
#finally:
#    GPIO.cleanup()  # Clean up GP
