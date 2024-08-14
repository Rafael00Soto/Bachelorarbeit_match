#!/usr/bin/env python3
import pandas as pd
import numpy as np
from odf.opendocument import load, OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell
from odf.text import P
import math


def orientation_error(gt_orientation, amcl_orientation):
    # Values already in gradians
    error = np.abs(gt_orientation - amcl_orientation)
    return error


def error_position(ground_truth_x, ground_truth_y, amcl_x, amcl_y):
    return math.sqrt((ground_truth_x - amcl_x)**2 + (ground_truth_y - amcl_y)**2)


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
    data[1].insert(0, '1')
    if data:
        df = pd.DataFrame(data[1:], columns=data[0])
        df = df.apply(pd.to_numeric, errors='coerce')
        return df
    else:
        df = pd.DataFrame()
        return df  # Return an empty DataFrame if sheet is not found


# Read file.ods
file_path = '/home/rafass/Documents/Bachelorarbeit/posiciones_objeto.ods'
objects = ['0x0x0', '1x1x1','0.5x1x1', '2x1x1', '2x1x1.5']
number_decimals = 5
columns_table = ["Object Dimension", "Average Pos Error (m)", "Min Pos Error (m)", "Max Pos Error (m)", "Standard Dev Pos Error (m)",
                 "Average Ori Error (°)", "Min Pos Ori (°)", "Max Pos Ori (°)", "Standard Dev Ori Error (°)"]

# Crear la tabla 'Stadistic'
table = Table(name="Accuracy")
header_row = TableRow()
for column in columns_table:
    cell = TableCell()
    cell.addElement(P(text=str(column)))
    header_row.addElement(cell)
table.addElement(header_row)

for dimension in objects:

    try:
        data = read_ods(file_path, dimension)
    except Exception as e:
        print(f'Error reading file: {e}')
        exit()

    # Asegurate que siempre esten los 150 datos
    if not data.empty:
        num_rows = data.shape[0]
        if num_rows == 150:
            # Calculate position error
            data['Position Error'] = data.apply(lambda row: error_position(row['Ground Truth x'],
                                                                           row['Ground Truth y'], row['Amcl Pose x'], row['Amcl Pose y']), axis=1)
            mean_error_position = data['Position Error'].mean()
            mean_error_position = round(mean_error_position, number_decimals)
            min_error_pos = data['Position Error'].min()
            min_error_pos = round(min_error_pos, number_decimals)
            max_error_pos = data['Position Error'].max()
            max_error_pos = round(max_error_pos, number_decimals)
            std_deviation_pos = data['Position Error'].std()
            std_deviation_pos = round(std_deviation_pos, number_decimals)

            # Calculate orientation error
            data['Orientation Error'] = data.apply(lambda row: orientation_error(
                row['Ground Truth Orientation'], row['Amcl Pose Orientation']), axis=1)
            mean_error_orientation = data['Orientation Error'].mean()
            mean_error_orientation = math.degrees(mean_error_orientation)
            mean_error_orientation = round(mean_error_orientation, number_decimals)
            min_error_ori = data['Orientation Error'].min()
            min_error_ori = math.degrees(min_error_ori)
            min_error_ori = round(min_error_ori, number_decimals)
            max_error_ori = data['Orientation Error'].max()
            max_error_ori = math.degrees(max_error_ori)
            max_error_ori = round(max_error_ori, number_decimals)
            std_deviation_ori = data['Orientation Error'].std()
            std_deviation_ori=math.degrees(std_deviation_ori)
            std_deviation_ori = round(std_deviation_ori, number_decimals)

            row = TableRow()  # New Row

            cell = TableCell()
            cell.addElement(P(text=str(dimension)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(mean_error_position)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(min_error_pos)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(max_error_pos)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(std_deviation_pos)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(mean_error_orientation)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(min_error_ori)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(max_error_ori)))
            row.addElement(cell)
            cell = TableCell()
            cell.addElement(P(text=str(std_deviation_ori)))
            row.addElement(cell)
            table.addElement(row)

        else:
            print(f'There are only {num_rows}, in simulation with object {dimension} information missing!!')
    else:
        print('DataFrame empty')

doc = load(file_path)
for sheet in doc.spreadsheet.getElementsByType(Table):
    if sheet.getAttribute("name") == 'Accuracy':
        doc.spreadsheet.removeChild(sheet)

#Add new Table to spreadsheet
doc.spreadsheet.addElement(table)
doc.save(file_path)
