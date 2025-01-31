# test_db.py
# test the data insertion into database with dummy data
from UMA_Db import UMA_Db

UMA_Db = UMA_Db(hostStr="192.168.59.136", dbPort=5432, dbStr="uma", uNameStr="postgres", dbPassStr="123456")
                    
                    
                    ww
UMA_Db.insertData('Piso 3', 11, 10, 20, 10, 121, 18, 18, 18, 19, False, 89, 17, 19, 71)
print("data insertion testing complete...")
