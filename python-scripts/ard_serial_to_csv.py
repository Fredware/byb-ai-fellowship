import serial
from struct import unpack

# setup
arduino_port = "COM10"
baud = 115200

nano_serial = serial.Serial(arduino_port, baud)
print("Connected to Arduino port: " + arduino_port)
csv_buff = ['timestamp,','ch0,','ch1,','ch2,','ch3,','ch4,','ch5\n']

# loop
for event in range(20):
    feature_table = unpack( '55f', nano_serial.read(55*4))
    for count, feature in enumerate(feature_table):
        str_element = str(feature)
        if (count + 1) % 5 == 0:
            str_element = str_element + ','
        else:
            str_element = str_element + '\n'
        csv_buff.append(str_element)
    # feature_table = [str(x) + ", " for x in feature_table]
    print("Got it!")   
    with open(f'thumb_data_{event}.csv', 'w') as file:
        # file.writelines( feature_table)
        file.writelines(csv_buff)