import serial
from struct import unpack

# setup
FINGER_NAME = 'pinky'
N_EVENTS = 50
arduino_port = "COM10"
baud = 115200

nano_serial = serial.Serial(arduino_port, baud)
print("Connected to Arduino port: " + arduino_port)

# loop
for event in range(N_EVENTS):
    csv_buff = ['timestamp,','ch1,','ch2,','ch3,','ch4,','ch5\n', '0,']
    feature_table = unpack( '55f', nano_serial.read(55*4))
        
    for count, feature in enumerate(feature_table):
        str_element = str(feature)
        if( count + 1) % 5 == 0:
            str_element = str_element + '\n'
            csv_buff.append(str_element)
            if( count != 54):
                csv_buff.append(str((count + 1)/5) + ',')
        else:
            str_element = str_element + ','
            csv_buff.append(str_element)
    
    print(f'Got {event} out of {N_EVENTS}')   
    with open(f'{FINGER_NAME}_data_{50+event}.csv', 'w') as file:
        file.writelines(csv_buff)