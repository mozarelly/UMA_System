#Importar librerías

import time
import board
import busio
import glob
import os
import datetime
import pytz
import math

#Modulo GPIO
import RPi.GPIO as GPIO

#Módulos ADC
import Adafruit_MCP4725
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

#Módulo de pantalla LCD
from RPLCD.i2c import CharLCD
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=20, rows=4, dotsize=8)

#Lector humedad DHT22
import adafruit_dht
sensor = adafruit_dht.DHT22(board.D17)

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
    #Listas de temperaturas
    list_temperatura=[]

    for device in (device_folder):
        temperatura = get_temperature(device)
        list_temperatura.append(round(temperatura,2))
    return list_temperatura

# Temperatura deseada
setpoint_temp = 16 # in degrees Celsius

#Modulos ADC
i2c = busio.I2C(board.SCL, board.SDA)

#ADS1115 1
ads1 = ADS.ADS1115(i2c, address=0x48)

ads1.gain = 1
V_corriente_motor = AnalogIn(ads1, ADS.P0, ADS.P1)
V_posicion_valvula = AnalogIn(ads1, ADS.P3)


#ADS1115 2
ads2 = ADS.ADS1115(i2c, address=0x49)

ads2.gain = 1
V_p_input = AnalogIn(ads2, ADS.P0)
V_p_output = AnalogIn(ads2, ADS.P1)

V_rpm = AnalogIn(ads2, ADS.P3)


#Feedback válvula
V_control_valvula = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
#Se establece la posición inicial de la válvula al 100% de apertura
control_valvula=0
V_control_valvula.set_voltage(control_valvula)

import time

#Valor inicial de PID
PID_error = 0
PID_valor = 0
Error_INT = 0

def control_PID(tiempo_loop, temp_suministro):
    global PID_error, Error_INT
    setpoint = 16
    Kc=-4.394
    Ki=-0.2645
    tao_I = Kc/Ki

    ultimo_PID=PID_error
    t_delay = tiempo_loop  #Periodo de muestreo en segundos

    PID_error = setpoint-temp_suministro                 #Calculo del error    
    Error_INT = Error_INT + PID_error*(5/t_delay)     #Calculo de la integral del error
    PID_valor = 100 + Kc*(PID_error + (1/tao_I)*Error_INT)  #Calculo de la salida del controlador PI

    if(PID_valor < 10):
        PID_valor = 10
    if(PID_valor > 100):
        PID_valor = 100
    
    if ((ultimo_PID==10 and PID_valor) or (ultimo_PID==100 and PID_valor==100)):
        return None
    

    c_valvula=round(PID_valor,2)
    c_valvula_bits= porcentaje_a_bits(c_valvula)
    control(c_valvula_bits)
    
    return c_valvula, PID_error, Error_INT, c_valvula_bits
    

def efecto_hall():
    valores = []
    rpm=True
    for _ in range(1000):
        valores.append(round(V_rpm.voltage,1))
        time.sleep(0.01)
    
    # Revisa si todas las entradas de la lista son el mismo valor
    if all(valor == valores[0] for valor in valores):
        rpm=False    
    return rpm, sum(valores)/1001

def corriente():
    #Corriente motor
    corriente = 0
    suma_cuadrado = 0
    start_time = time.time() 
    counter = 0
    mult=1.41
    while (time.time() - start_time) < 1.0:  #Mide por 1 segundo
        corriente = V_corriente_motor.voltage*30*mult
        suma_cuadrado += corriente**2 
        counter += 1
        
    rms_current = math.sqrt(suma_cuadrado / counter)

    V_corriente=round(V_corriente_motor.voltage,2)

    return round(rms_current,2), V_corriente

def presion():
    # y=mx+b
    
    #Suministro
    m_pi=38.67
    b_pi=0
    V_p_suministro=round(V_p_input.voltage,2)
    p_suministro=round(m_pi*V_p_suministro+b_pi,2)
    
    #Retorno
    m_po=38.67
    b_po=0
    V_p_retorno=round(V_p_output.voltage,2)
    p_retorno=round(m_po*V_p_retorno+b_po,2)
    
    return p_suministro, V_p_suministro, p_retorno, V_p_retorno


def f_valvula():
    feedback_voltaje=round(V_posicion_valvula.voltage,2)
    
    #y=mx+b
    m_val=-28.046
    b_val=124.84
    posicion_valvula=round(m_val*feedback_voltaje+b_val,0)
    
    return max(0, min(100, posicion_valvula)), feedback_voltaje

def porcentaje_a_bits(porcentaje):
    # Calcula en numero de bits
    raw_bits = (porcentaje / 100) * 4096
    # Redondea al numero entero mas cercano
    closest_bits = round(raw_bits)
    # El numero de bits debe estar en el rango valido (0 to 4096)
    return max(0, min(4096, closest_bits))

def control(valor_deseado):
    V_control_valvula.set_voltage(valor_deseado)

def lcd_print(valor_a, valor_b, valor_c, valor_d):
    lcd.clear()
    lcd.cursor_pos=(0, 0)
    lcd.write_string(valor_a)
    lcd.cursor_pos=(1, 0)
    lcd.write_string(valor_b)
    lcd.cursor_pos=(2, 0)
    lcd.write_string(valor_c)
    lcd.cursor_pos=(3, 0)
    lcd.write_string(valor_d)

def on_uma(corriente, rpm):
    if corriente>0.5 and rpm==True:
        return 'Encendido', True
    else:
        return 'Apagado', False

def uma_print(tiempo, estado, motor, valvula, p_suministro, p_retorno, t_agua_in, t_agua_out, t_aire_in, t_aire_out, humedad):
    lcd_print('Estado de maquina', 'UMA Piso 3-Este',tiempo,'')
    time.sleep(10)
    lcd_print('Motor UMA', estado, 'Corriente: '+(str(motor)+'A'),'')
    time.sleep(10)
    lcd_print('Valvula de control', 'Apertura: '+str(valvula)+'%', '','')
    time.sleep(10)
    lcd_print('Presion',('Suministro:'+ str(p_suministro)+ 'psi'),('Retorno:'+str(p_retorno)+'psi'),'')
    time.sleep(10)
    lcd_print('Temperatura agua',('Suministro:'+ str(t_agua_in)+'C'),('Retorno:'+str(t_agua_out)+'C'),'')
    time.sleep(10)
    lcd_print('Temperatura aire',('Suministro:'+ str(t_aire_in)+'C'),('Retorno:'+str(t_aire_out)+'C'),('Humedad:'+str(humedad)+'%'))
    time.sleep(10)
    lcd_print('CCCT','Gerencia Tecnica','','M. Panzarelli')

humedad=0

while True:
    try:
            
        inicio = time.time()
        tiempo = datetime.datetime.now(pytz.timezone('America/Caracas'))
        tiempo_formato = tiempo.strftime("%d/%m/%y %H:%M")

        #Corriente motor
        corriente_motor, V_corriente=corriente()       
        #Presion
        p_suministro, V_p_suministro, p_retorno, V_p_retorno=presion()
        #Posicion valvula
        posicion_valvula, feedback_voltaje = f_valvula()
        #Lista de sensores temperaturas
        list_temperatura=sensor_temperatura(device_folder)
        #Humedad
        try:
            #Indica el valor del sensor de humedad
            humedad = sensor.humidity
        except RuntimeError as error:
            print(error.args[0])
            time.sleep(2.0)
        except Exception as error:
            sensor.exit()
            raise error

        #Giro de motor
        rpm,promedio_h=efecto_hall()

        #Corriente
        estado, estado_binario=on_uma(corriente_motor, rpm)
        
        #Imprimir en la pantalla LCD
        uma_print(tiempo_formato, estado, corriente_motor, posicion_valvula, p_suministro, p_retorno, list_temperatura[0], list_temperatura[3], list_temperatura[2], list_temperatura[1], humedad)

        fin = time.time()
        #Tiempo transcurrido en el loop
        t_ciclo=(fin-inicio)
        control_valvula, PID_error, Error_INT, control_bits=control_PID(t_ciclo, list_temperatura[2])
        
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()  #Limpiar estatus GPIO
