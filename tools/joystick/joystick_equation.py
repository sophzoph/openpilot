#!/usr/bin/env python
import csv
import time
import cereal.messaging as messaging
from cereal.messaging import *
from openpilot.common.realtime import Ratekeeper
from openpilot.common.numpy_fast import interp, clip
from openpilot.common.params import Params
import argparse
import os

# check line 274 in the openpilot file for vEgo ex
from selfdrive.locationd.calibrationd import *


class datastream:
    def __init__(self, csv_filepath):
        """Set default values for when datastream object is created. Incr / decr only set to 1%"""
        self.axis_increment = 0.01    # 1% incr / decr to adjusting speed
        self.axis_values = {'gb':0.1, 'steer': 0.}   # initialize gb to a small value for later on proportional calculation
        self.axes_order = ['gb', 'steer']
        self.csv_filepath = csv_filepath  
        self.control_sock = messaging.pub_sock('testJoystick')
        self.time_list = []
        self.speed_list = []
        self.csv_data = self.read_csv_data()

        # subscribe to get car real-time info
        self.sm = messaging.SubMaster(['cameraOdometry', 'carState', 'carParams'], poll=['cameraOdometry'])
        # create calibrator object here
        self.calibrator = Calibrator(param_put=True)

        # start time before communicating first car speed
        self.start_time = time.time() 
        # communicate the first speed to get the car moving
        dat = messaging.new_message('testJoystick')
        dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
        dat.testJoystick.buttons = [False]
        self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket


    def read_csv_data(self):
        """
        Read CSV file data into a list of dictionaries for indexing
        """
        # VERY IMPORTANT: NEED TO MAKE SURE THAT SPEEDS IN FILE ARE MPH
        data = []
        with open(self.csv_filepath, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                data.append(row)
                self.time_list.append(float(row['time']))
                # self.speed_list.append(float(row['speed']) / 2.236)  # convert from mph to m/s
                self.speed_list.append(float(row['speed'])) 
        return data


    def get_target_speed(self, elapsed_time):
        """
        Return target speed based on the elapsed time.
        """
        return interp(elapsed_time, self.time_list, self.speed_list)  # interp return type should be a float


    def control_speed(self, start_time):
        """
        Runs a while loop until the last time in the CSV file is reached. Uses Proportional controller 
        """
        # loop through the csv rows and update speed based on P controller
        # load csv data into a list of dictionaries

        # calculate elapsed time 
        end_time = self.time_list[-1]
        current_time = time.time()
        self.calibrator.v_ego = self.sm['carState'].vEgo
        Vr1 = self.calibrator.v_ego * 2.23694
      
        elapsed_time = current_time - self.start_time

        # define a sleep time (pause between iterations)
        interval = 0.1  # loop will run every 10 ms
        t0 = 0
        Vr0 = 0
        g0 = 0.1
        t1 = elapsed_time
   
        while elapsed_time < end_time:   # should this be <=?
            
            """Refer to below for original proportional control"""
            # # Get the target speed based on the elapsed time from CSV data
            # target_speed = self.get_target_speed(elapsed_time)
            # # Proportional controller or other logic to adjust the speed (need to implement)
            # error = target_speed - self.axis_values['gb']   # need to get the actual GB value from openpilot for car
            # gas_brake = clip(self.axis_values['gb'] + error, -1, 1)  # with 0.1 being the tuning parameter
            # # clip to ensure the value in [-1, 1]
            # self.axis_values['gb'] = gas_brake  # set the gb to calculated p
        
            # calculate equation vars
            t2 = t1 + interval  # next time (next iteration), interval is an approximation (assume calculatiosn and communication time is tiny)
            # calculate target V
            Vt2 = self.get_target_speed(t2)

            # uncoment the bottom two lines when i can actually get car speed from openpilot
            # calibrator.v_ego = sm['carState'].vEgo
            # Vr1 = calibrator.v_ego * 2.23694   # to convert from m/s to miles
            # Vr1 = Vr0 + 0.07
            
            prev_vEgo_V = Vr1  # store curr openpilot V for the next iteration

            # solve equation for g1
            if Vt2 == Vr1:
                g1 = 0
            elif Vr1 == Vr0:
                g1 = 0.1  # deafult value in this case
            else:
                g1 = g0 * (t1 - t0) * (Vt2 - Vr1) / (Vr1 - Vr0) / interval
              
            if g1 > 1:
                g1 = 1
            elif g1 < -1:
                g1 = -1

            
            self.axis_values['gb'] = g1

            # use equation (prev_gas(curr_time - prev_time)) / (curr_target_V - prev_vEgo_V) 
            # where curr refers to time at 1st second and prev refers to time at 0th second
            # prev_gas = self.axis_values['gb']
            # elapsed_time = current_time - start_time
            # prev_time = elapsed_time - 0.010   # where elasped time refers to curr time
            # curr_target_V = self.get_target_speed(elapsed_time)
            # prev_vEgo_V was calculated for the previous iteration
            

            # set gb to the newly calculated proportion
            # self.axis_values['gb'] = (prev_gas * (elapsed_time - prev_time)) / (curr_target_V - prev_vEgo_V)
            print("AT TIME:", t1)
            print("g0:", self.axis_values['gb'])
            print("t0:", t0)
            print("t1:", t1)
            print("t2:", t2)
            print("Vt2:", Vt2)
            print("Vr0:", Vr0)
            print("Vr1:", Vr1)

            # this calculation is used for the next iteration
            # vEgo speed provided in m/s, convert it to miles
            # calibrator.v_ego = sm['carState'].vEgo
            # prev_vEgo_V = calibrator.v_ego * 2.23694
            # prev_vEgo_V += 0.0005

            #gas_brake = clip(self.axis_values['gb'], -1, 1)  # with 0.1 being the tuning parameter
            # # clip to ensure the value in [-1, 1]
            #gas_brake = self.axis_values['gb']
            #self.axis_values['gb'] = gas_brake  # set the gb to calculated p

            # send control signals to Openpilot
            # messaging communicates info between different components of the system
            dat = messaging.new_message('testJoystick')
            dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
            dat.testJoystick.buttons = [False]
            self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket



            # calculate target speed here just to see if close
            Vt1 = self.get_target_speed(t1)
            print("target speed: ", Vt1, "---- current gb: ", self.axis_values['gb'])
            print("actual speed: ", Vr1)
            print()

            time.sleep(interval)  # adjustmnet occurs every 10ms, adjust if needed

            # calculate values for next loop
            current_time = time.time()
            self.calibrator.v_ego = self.sm['carState'].vEgo
            Vr1 = self.calibrator.v_ego * 2.23694
          
            elapsed_time = current_time - start_time
            t0 = t1
            Vr0 = Vr1
            g0 = g1
            if g0 == 0:
                g0 = 0.1
            t1 = elapsed_time


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
    # start_time = time.time() 

    # communicate the first speed for car in the constructor
    
    # subscribe to get real-time info from openpilot
    # sm = messaging.SubMaster(['cameraOdometry', 'carState', 'carParams'], poll=['cameraOdometry'])
    # create calibrator object to get vEgo
    # calibrator = Calibrator(param_put=True)
    
    while True:
        ds.control_speed(ds.start_time)




    
