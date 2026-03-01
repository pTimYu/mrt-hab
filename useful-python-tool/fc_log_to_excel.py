import pandas as pd

input_file_name = 'HAB_DATA.txt'

# 1. Load the text file
# If your delimiter is a tab, use sep='\t'. If it's a pipe, use sep='|'
df = pd.read_csv(input_file_name, sep='\t')

# 2. Export to Excel
df.to_excel('output.xlsx', index=False)