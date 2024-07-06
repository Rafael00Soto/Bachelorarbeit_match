#! /usr/bin/env python3
from odf.opendocument import load
from odf.table import Table, TableRow, TableCell
from odf.text import P


def check_for_tables(file_path):
    doc = load(file_path)
    tables = doc.getElementsByType(Table)
    print(f"Tabla: {tables[0].getAttribute('name')}")
    for row in tables[0].getElementsByType(TableRow):
        if row==0:
            print("Se ignoraron los nombres de las columnas")#To ignore the first row with the names of the columns
        else:
            
            print(row)
        
   
    '''for row in tables[0].getElementsByType(TableRow):
        row_data = []
        for cell in row.getElementsByType(TableCell):
            text_elements = cell.getElementsByType(P)
            cell_text = " ".join([str(te) for te in text_elements])
            row_data.append(cell_text)'''




        
    

# Uso del script
check_for_tables("mi_hoja_de_calculo.ods")