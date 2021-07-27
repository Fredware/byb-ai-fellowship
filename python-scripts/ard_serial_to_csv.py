import serial
from struct import unpack

# setup
arduino_port = "COM10"
baud = 115200

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port: " + arduino_port)

# loop
for event in range(20):

    feature_table = unpack( '55f', ser.read(55*4))
        
    with open(f'thumb_data_{event}.csv', 'w') as file:
        file.writelines( feature_table)