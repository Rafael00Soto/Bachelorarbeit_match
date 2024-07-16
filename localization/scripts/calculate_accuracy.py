#!/usr/bin/env python3
import pandas as pd
import numpy as np
from odf.opendocument import load
from odf.table import Table, TableRow, TableCell
from odf.text import P
from math import sqrt


def orientation_error(gt_orientation, amcl_orientation):
    #Values already in gradians
    error = np.abs(gt_orientation - amcl_orientation)
    return error
   
def error_position(gx, gy, ax, ay):
    return sqrt((gx - ax)**2 + (gy - ay)**2)

def read_ods(file_path, sheet_name):
    doc = load(file_path)
    data = []
    for sheet in doc.spreadsheet.getElementsByType(Table):
        print(f'En Ods se llama {sheet.getAttribute("name")}')
        print(f'En python se llama {sheet_name}')
        
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
        df=pd.DataFrame(data[1:], columns=data[0])
        df = df.apply(pd.to_numeric, errors='coerce')  
        return df
            
    else:
        df= pd.DataFrame()
        return df # Return an empty DataFrame if sheet is not found
    
     


# Read file.ods
file_path = '/home/rafass/Documents/Bachelorarbeit/posiciones_sin_objeto.ods'
sheet_name='Tabla1'
number_decimals=5

try:
    data = read_ods(file_path, sheet_name)
except Exception as e:
    print(f'Error reading file: {e}')
    exit() 


############ Asegurate que siempre esten los 150 datos ####################


if not data.empty:
        num_rows = data.shape[0]
        if num_rows == 150:
            #Calculate position error
            data['Position Error'] = data.apply(lambda row: error_position(row['Ground Truth x'],
                                            row['Ground Truth y'], row['Amcl Pose x'], row['Amcl Pose y']), axis=1)
            mean_error_position = data['Position Error'].mean()
            mean_error_position = round(mean_error_position, number_decimals)
            print(f'Error Position Mean is: {mean_error_position}')

            #Calculate orientation error    (Verifiy what happen with negative orientation)
            data['Orientation Error'] = data.apply(lambda row: orientation_error(row['Ground Truth Orientation'], row['Amcl Pose Orientation']), axis=1)
            mean_error_orientation = data['Orientation Error'].mean()
            mean_error_orientation = round(mean_error_orientation, number_decimals)
            print(f'Error Orientatin Mean is: {mean_error_orientation}')

        else:
            print(f'There are only {num_rows}, information missing!!')
else:
    print('DataFrame empty')
            

