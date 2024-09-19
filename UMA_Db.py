import psycopg2

class UMA_Db:
    def __init__(self, hostStr, dbPort, dbStr, uNameStr, dbPassStr) -> None:
        self.hostStr = hostStr
        self.dbPort = dbPort
        self.dbStr = dbStr
        self.uNameStr = uNameStr
        self.dbPassStr = dbPassStr


    def insertData(self, uma, tmp1, tmp2, tmp3, tmp4, pre_suministro, pre_retorno, c_motor, prom_efecto_hall, encendido, p_input_valvula, input_valvula, p_output_valvula, output_valvula):
        try:
            conn = psycopg2.connect(host=self.hostStr, port=self.dbPort,
                                    dbname=self.dbStr, user=self.uNameStr, password=self.dbPassStr)

            # get a cursor object from the connection
            cur = conn.cursor()

            # prepare data insertion rows
            dataInsertionTuples = [(uma, tmp1, tmp2, tmp3, tmp4, pre_suministro, pre_retorno, c_motor, prom_efecto_hall, encendido, p_input_valvula, input_valvula, p_output_valvula, output_valvula)]

            # create sql command for rows insertion
            dataText = ','.join(cur.mogrify('(%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)', row).decode(
                "utf-8") for row in dataInsertionTuples)

            sqlTxt = 'INSERT INTO public.uma_data\
                    (uma, tmp1, tmp2, tmp3, tmp4, pre_suministro, pre_retorno, c_motor, prom_efecto_hall, encendido, p_input_valvula, input_valvula, p_output_valvula, output_valvula)\
                    VALUES {0} on conflict (uma, data_time) \
                do update set tmp1 = excluded.tmp1,	tmp2 = excluded.tmp2, tmp3 = excluded.tmp3, tmp4 = excluded.tmp4, pre_suministro = excluded.pre_suministro, pre_retorno = excluded.pre_retorno, c_motor = excluded.c_motor, prom_efecto_hall = excluded.prom_efecto_hall, encendido = excluded.encendido, p_input_valvula = excluded.p_input_valvula, input_valvula = excluded.input_valvula, p_output_valvula = excluded.p_output_valvula, output_valvula = excluded.output_valvula'.format(dataText)

            # execute the sql to perform insertion
            cur.execute(sqlTxt)

            rowCount = cur.rowcount
            print("number of inserted rows =", rowCount)

            # commit the changes
            conn.commit()
        except (Exception, psycopg2.Error) as error:
            print("Error de integración en PostgreSQL...\n", error)
        finally:
            if (conn):
                # close the cursor object to avoid memory leaks
                cur.close()
                # close the connection object also
                conn.close()
