import pandas as pd
import time

input_file_name = 'HAB_DATA.txt'

# 1. Load the text file
# If your delimiter is a tab, use sep='\t'. If it's a pipe, use sep='|'
df = pd.read_csv(input_file_name, sep='\t')

year = time.localtime(time.time()).tm_year
month = time.localtime(time.time()).tm_mon
day = time.localtime(time.time()).tm_mday

time_all = year * 10000 + month * 100 + day

# 2. Export to Excel
df.to_excel(f'HAB_{time_all}_FlightComputer.xlsx', index=False)