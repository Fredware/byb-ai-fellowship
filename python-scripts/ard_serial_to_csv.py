import serial
from struct import unpack

# setup
SESSION_ID = 3
FINGER_NAME = 'pinky'

N_EVENTS = 55
arduino_port = "COM10"
baud = 115200

N_CHANS = 5
N_FEATURES = 11
FEATURE_BYTESIZE = 4

nano_serial = serial.Serial( arduino_port, baud)
print( "Connected to Arduino port: " + arduino_port)

# loop
for event in range( N_EVENTS):
    # Initialize buffer with column headers
    csv_buff = ['timestamp,','ch1,','ch2,','ch3,','ch4,','ch5\n', '0,']
    # Get the features for all channels
    feature_table = unpack( '55f', nano_serial.read( N_CHANS * N_FEATURES * FEATURE_BYTESIZE))
        
    for count, feature in enumerate( feature_table):
        str_element = str(feature)
        # Add a newline at the end of a row
        if( count + 1) % N_CHANS == 0:
            str_element = str_element + '\n'
            csv_buff.append(str_element)
            # Add the timestamp of the next row except for the last element
            if( count != 54):
                csv_buff.append(str((count + 1)/5) + ',')
        else:
            str_element = str_element + ','
            csv_buff.append(str_element)
    
    print(f'Got {event+1} out of {N_EVENTS}')   
    with open(f'{FINGER_NAME}_data_{SESSION_ID}_{event}.csv', 'w') as file:
        file.writelines(csv_buff)