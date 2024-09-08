import can
import time
import j1939
import csv 
import msvcrt 
import os
import sys
from datetime import datetime

BASE_DIR = os.path.dirname(sys.argv[0])
start_time_offset = None
start_time = None 
data_dict = {"timestamp":None, "viscosity":None, "density": None, "dielectric": None, "resistance":None, "temperature":None, "diagnostics":None}
csvfile = None
csvwriter = None

message_mapping = {
            0xfd08:{"message_id":0xfd08,
            "variables":[
                {"variable_name":"viscosity",
                "byte_length":2,
                "start_byte":0,
                "end_byte":1,
                "factor":0.015625,
                "offset":0,
                "unit":"cP",
                "representation_type":"dec_num"},
                {"variable_name":"density",
                 "byte_length":2,
                 "start_byte":2,
                 "end_byte":3,
                 "factor":0.00003052,
                 "offset":0,
                 "unit":"g/cm³",
                 "representation_type":"dec_num"},
                {"variable_name":"dielectric",
                 "byte_length":2,
                 "start_byte":6,
                 "end_byte":7,
                 "factor":0.00012207,
                 "offset":0,
                 "unit":"",
                 "representation_type":"dec_num"}],
            },
            0xfa67:{"message_id":0xfa67,
            "variables":[
                {"variable_name":"resistance",
                "byte_length":3,
                "start_byte":0,
                "end_byte":3,
                "factor":1000,
                "offset":0,
                "unit":"Ohm",
                "representation_type":"dec_num"}]
            },
            0xfeee:{"message_id":0xfeee,
            "variables":[
                {"variable_name":"temperature",
                "byte_length":2,
                "start_byte":2,
                "end_byte":3,
                "factor":0.03125,
                "offset":-273,
                "unit":"°C",
                "representation_type":"dec_num"}]
            },
            0xff31:{"message_id":0xff31,
            "variables":[
                {"variable_name":"diagnostics",
                "byte_length":3,
                "start_byte":0,
                "end_byte":2,
                "factor":1,
                "offset":0,
                "unit":"",
                "representation_type":"bin_num"}]
            }}

def map_message(pgn, timestamp, data, message_mapping=message_mapping,data_dict=data_dict):
    global csvwriter
    global csvfile 

    cur_data = data_dict.copy() # copy of the empty data dictionary
    cur_time = datetime.now().timestamp()
    cur_data["timestamp"] = cur_time
    if pgn in message_mapping.keys():
        for variable in message_mapping[pgn]["variables"]:
            cur_value = int.from_bytes(data[variable["start_byte"]:variable["end_byte"]+1], byteorder="little")*variable["factor"]+variable["offset"]
            if variable["representation_type"] == "dec_num":
                cur_data[variable["variable_name"]] = round(cur_value,5)
            elif variable["representation_type"] == "bin_num":
                cur_data[variable["variable_name"]] = format(cur_value, "b").zfill(8*variable["byte_length"])
            else:
                print(f"WARNING: Not supported representation type '{variable["representation_type"]}'")

        # Print to screen
        for key, value in cur_data.items():
            if (value and (key != "timestamp")):
                print(f"{datetime.fromtimestamp(cur_data["timestamp"]).isoformat()} {key.upper()} {value}")
        
        # Write to CSV file
        csvwriter.writerow(cur_data)
        csvfile.flush()
    else:
        print(f"WARNING: Not configured message id '{hex(pgn)}'")

                 
def on_message(priority, pgn, sa, timestamp, data):
    map_message(pgn, data)

def main():
    global csvfile
    global csvwriter
    now = datetime.now()
    filename = os.path.join(BASE_DIR, f"OIL_MEASUREMENT_{now.year}_{now.month}_{now.day}_{now.hour}_{now.minute}.csv" )
    csvfile = open(filename,"a") 
    csvwriter = csv.DictWriter(csvfile, fieldnames = data_dict.keys())
    csvwriter.writeheader()

    ecu = j1939.ElectronicControlUnit()
    ecu.connect(bustype="ixxat", channel=0, bitrate=250000)
    print("INFO: Successfully connected to CAN interface")
    ecu.subscribe(on_message)

    try:
        while True:
            if msvcrt.kbhit():
                if msvcrt.getwche() == '\r':
                    csvfile.close()
                    print(f"INFO: File was stored in '{filename}'")
                    break
    except:
        csvfile.close()
    


if __name__ == '__main__':
     main()

