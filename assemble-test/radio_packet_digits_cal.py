## Radio digit/bit number calculator
## Input the name/digit of items
## Output the transmitting byte number and the 
import numpy as np

name = ["Time",
        "Latitude", "Longitude",
        "GPS Speed", "GPS Heading",
        "Height",
        "Voltage",
        "RSSI", "Gain",
        "Acceleration X", "Acceleration Y", "Acceleration Z"]

digits = [4,
          9, 9,
          3, 3,
          5,
          3,
          3, 3,
          6, 6, 6]

print(f"Total digits (decimal): {sum(digits)}\n")

print("Digits Breakdown:")
i = 0
for index, num in enumerate(digits):
    print(f"{i}-{i + num - 1}: {name[index]}")
    i = i + num

bin_number = sum(digits) * np.log2(10)
byte_number = bin_number / 8
print(f"\nBinary number: {bin_number}")
print(f"Byte number: {byte_number}")

time_on_air_ascii = sum(digits) / 28 ## Estimated time on air time (reduced model)
time_on_air_bin = sum(digits) * byte_number / 28
print(f"\nTime-on-air by using ASCII: {time_on_air_ascii}s")
print(f"Time-on-air by using Binary Number: {time_on_air_bin}s")