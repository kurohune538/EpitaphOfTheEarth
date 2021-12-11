import argparse
import time
import random
from types import NoneType
from pythonosc import udp_client
import serial
import re
ser = serial.Serial('/dev/ttyACM0', 57600)
distance = ''

def create_osc_client(ip, port):
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=ip,
                       help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=port,
                       help="The port the OSC server is listening on")
    args=parser.parse_args()
    
    client = udp_client.SimpleUDPClient(args.ip, args.port)
    return client


if __name__ == "__main__":
    osc_client = create_osc_client("10.23.10.127", 5005)
    
    # for x in range(10):
    #     osc_client.send_message("/weight", random.random())

    while True:
        String_data = str(ser.readline())
        # print(String_data)
        loadcell_value = str(re.search('LoadCellValue:.*?,',String_data))
        if(loadcell_value is not NoneType and len(loadcell_value) > 0):
            weight = re.findall('\d+\.\d+', loadcell_value)
            if(len(weight) >0):
                weight = weight[0]
                print(weight)
        distance_sensor_value = re.search('irSensorValue:.*?,',String_data)
        if(distance_sensor_value is not None):
            print(distance_sensor_value)
            distance_sensor_value = distance_sensor_value[0]
            if(distance_sensor_value is not NoneType):
                distance = re.findall('[0-9]+', distance_sensor_value)[0]
                print(distance)
        # isPlayed_value = re.search('isPlayed:.*?n',String_data)
        # if(isPlayed_value is not NoneType):
        #     tmpIsPlayed = isPlayed_value[0]
        #     isPlayed_value = tmpIsPlayed[9:10]
        #     print(isPlayed_value)
        osc_client.send_message("/weight", 200)
        osc_client.send_message("/distance", int(distance))
    
    ser.close()