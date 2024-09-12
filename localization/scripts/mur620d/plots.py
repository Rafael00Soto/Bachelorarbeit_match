#!/usr/bin/env python3
import pandas as pd
import numpy as np
from odf.opendocument import load, OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell, TableColumn
from odf.style import Style, TableColumnProperties
from odf.text import P
import matplotlib.pyplot as plt

import math

def read_ods(file_path, sheet_name):
    doc = load(file_path)
    data = []
    for sheet in doc.spreadsheet.getElementsByType(Table):
        if sheet.getAttribute("name") == sheet_name:
            for row in sheet.getElementsByType(TableRow):
                row_data = []
                for cell in row.getElementsByType(TableCell):
                    cell_value = ""
                    for p in cell.getElementsByType(P):
                        cell_value += str(p)
                    row_data.append(cell_value)
                data.append(row_data)
    
    if data:
        df = pd.DataFrame(data[1:], columns=data[0])
        if df.isnull().any().any():
            print(f"Value NaN in DataFrame {sheet_name}")

        print(df)
        return df
    else:
        df = pd.DataFrame()
        return df  # Return an empty DataFrame if sheet is not found
    


file_path = '/home/rafass/Documents/Bachelorarbeit/Posiciones/mur620d/positions.ods'

name='Accuracy'
try:
    data = read_ods(file_path, name)
except Exception as e:
    print(f'Error reading file: {e}')
    exit()
#data = data.iloc[:9]
#print(data)
#Text to numeric
cols_to_convert = ['Average Ori Error (°)', 'SD Orientation (°)' ] 
data[cols_to_convert] = data[cols_to_convert].apply(pd.to_numeric, errors='coerce')

fs = 15
fs2 = 16

# Asumimos que 'data' es tu DataFrame cargado previamente

# Definir los Ã­ndices de los pares
pares = [(1, 2), (3, 4), (5, 6)]

# Crear listas para almacenar los datos
group_labels = []
group_values_1 = []
group_values_2 = []
std_dev_1 = []
std_dev_2 = []

# Extraer los datos de los pares
for i, j in pares:
    group_labels.append(data.loc[i, 'Object Name'])  # Etiquetas del primer grupo
    group_values_1.append(data.loc[i, 'Average Ori Error (°)'])  # Valores del primer grupo
    group_values_2.append(data.loc[j, 'Average Ori Error (°)'])  # Valores del segundo grupo
    std_dev_1.append(data.loc[i, 'SD Orientation (°)'])  # DesviaciÃ³n estÃ¡ndar del primer grupo
    std_dev_2.append(data.loc[j, 'SD Orientation (°)'])  # DesviaciÃ³n estÃ¡ndar del segundo grupo

# Definir la posiciÃ³n de las barras en el eje X
x = np.arange(len(group_labels))  # PosiciÃ³n de los grupos
width = 0.35  # Ancho de las barras

# Crear el grÃ¡fico de barras agrupadas con barras de error
plt.figure(figsize=(10, 6))
plt.bar(x - width/2, group_values_1, width, yerr=std_dev_1, label='Unmapped Object', capsize=5, color='green')  
plt.bar(x + width/2, group_values_2, width, yerr=std_dev_2, label='Mapped Object', capsize=5, color='red')


plt.xlabel('', fontsize=fs)
plt.ylabel('', fontsize=fs)
plt.title('Average Orientation Error with Standard Deviation', fontsize=fs2)

# Colocar las etiquetas de los objetos en el centro de los grupos
plt.xticks(x, group_labels, rotation=45, ha='right', fontsize=14)
plt.yticks(fontsize=14)


plt.legend(fontsize=14)


plt.tight_layout()


plt.show()