#!/usr/bin/env python
import csv
import time
import cereal.messaging as messaging
from cereal.messaging import *
from openpilot.common.realtime import Ratekeeper   # not using this
from openpilot.common.numpy_fast import interp, clip
from openpilot.common.params import Params
import argparse
import os

# to test this:
# use print statements to see the changes in self.axes_values over time

# how does this integrated with openpilo t system to update the car speed?


class datastream:
    def __init__(self, csv_filepath):
        """Set default values for when datastream object is created. Incr / decr only set to 1%"""
        self.axis_increment = 0.01    # 1% incr / decr when adjusting speed to match
        self.axis_values = {'gb':0., 'steer': 0.} 
        self.axes_order = ['gb', 'steer']
        self.csv_filepath = csv_filepath  
        self.control_sock = messaging.pub_sock('testControl')
        self.time_list = []
        self.speed_list = []
        self.csv_data = self.read_csv_data()


    def read_csv_data(self):
        """
        Read CSV file data into a list of dictionaries for indexing
        """
        data = []
        with open(self.csv_filepath, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                data.append(row)
                self.time_list.append(float(row['time']))
                self.speed_list.append(float(row['speed']))
        return data

    def get_target_speed(self, elapsed_time):
        """
        Return target speed based on the elapsed time.
        """
        # for row in self.csv_data:
        #     if float(row['time']) >= elapsed_time:
        #         return float(row['speed'])
        # If elapsed_time exceeds the data range, return the last speed (this case should not happen based on update_speed)
        # return float(self.csv_data[-1]['speed'])
        return interp(elapsed_time, self.time_list, self.speed_list)  # interp return type should be a float

    def control_speed(self, start_time):
        """
        Runs a while loop until the last time in the CSV file is reached. Uses Proportional controller 
        """
        # loop through the csv rows and update speed based on P controller
        # load csv data into a list of dictionaries
        # CSV could look like this
        # time,speed
        # 0,0
        # 1,5
        # 2,10
        # 3,15
        # 4,20
        # List of Dictionaries could look like this
        # [{'time': '0', 'speed': '0'}
        # {'time': '1', 'speed': '5'}
        # {'time': '2', 'speed': '10'}
        # {'time': '3', 'speed': '15'}
        # {'time': '4', 'speed': '20'}]
        # calculate elapsed time 
        end_time = self.time_list[-1]
        current_time = time.time()
        elapsed_time = current_time - start_time
        while elapsed_time < end_time:   # should this be <=?
            #elapsed_time = current_time - start_time   

            # Get the target speed based on the elapsed time from CSV data
            target_speed = self.get_target_speed(elapsed_time)

            # Proportional controller or other logic to adjust the speed (need to implement)
            error = target_speed - self.axis_values['gb']   # need to get the actual GB value from openpilot for car
            gas_brake = clip(self.axis_values['gb'] + error, -1, 1)  # with 0.1 being the tuning parameter
            # clip means the value must be [-1, 1]

            self.axis_values['gb'] = gas_brake  # set the gb to the calculated p

            # send control signals to Openpilot
            # messaging communicates info between different components of the system
            dat = messaging.new_message('testJoystick')
            dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
            dat.testJoystick.buttons = [False]
            self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket


            # Check if elapsed time has exceeded the last time in the CSV, if so exit loop
            # if elapsed_time >= float(self.csv_data[-1]['time']):
            #     break
            print("target speed is: ", target_speed, "actual gb: ", self.axis_values['gb'])
            time.sleep(0.01)
            current_time = time.time()
            elapsed_time = current_time - start_time
        #process_time = new_time - current_time

        # Sleep for a short duration (not sure what to adjust this to)
        #time.sleep(1 - process_time)   # make it more precise



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                                 'openpilot must be offroad before starting joysticked.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--csv_file', help='CSV file containing time (s) and speed (mph)')
    args = parser.parse_args()

    if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
        print("The car must be off before running datastream.")
        exit()

    # create the data stream object
    ds = datastream(csv_filepath = args.csv_file)
    start_time = time.time() 

    while True:
        ds.control_speed(start_time)
