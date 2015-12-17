# -*- coding: utf-8 -*-

import xlrd
import numpy as np


book = xlrd.open_workbook('Char_Sheet.xlsx')
sheet1 = book.sheet_by_index(0)

rocket_data = []

for row in range(0,43):
    rocket_data.append(sheet1.cell(row,1).value)
    

np.savetxt("rocket_param.dat",rocket_data,fmt="%.4f",delimiter=",")


