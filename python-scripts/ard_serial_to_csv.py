import serial
from struct import unpack

# setup
arduino_port = "COM10"
baud = 115200

nano_serial = serial.Serial(arduino_port, baud)
print("Connected to Arduino port: " + arduino_port)

# loop
for event in range(20):

    feature_table = unpack( '55f', nano_serial.read(55*4))
    feature_table = [str(x) + ", " for x in feature_table]
    print("Got it!")   
    with open(f'thumb_data_{event}.csv', 'w') as file:
        file.writelines( feature_table)