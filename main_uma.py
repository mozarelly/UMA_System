from UMA_Db import UMA_Db
from control import UMA_sensor


uma_name='Piso 3'
UMA_Db = UMA_Db(hostStr="localhost", dbPort=5432,
                    dbStr="uma", uNameStr="postgres", dbPassStr="123456")

data=UMA_sensor()

UMA_Dict=data.get_uma()

if UMA_Dict == None:
    print("Error retrieving temp hum from sensor")
    exit(0)

UMA_Db.insertData(uma_name, UMA_Dict["tmp1"], UMA_Dict["tmp2"], UMA_Dict["tmp3"], UMA_Dict["tmp4"], UMA_Dict["pre_suministro"], UMA_Dict["pre_retorno"], UMA_Dict["c_motor"], UMA_Dict["prom_efecto_hall"], UMA_Dict["encendido"], UMA_Dict["p_input_valvula"], UMA_Dict["input_valvula"], UMA_Dict["p_output_valvula"], UMA_Dict["output_valvula"])
