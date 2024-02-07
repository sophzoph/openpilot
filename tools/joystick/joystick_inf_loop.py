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

#from selfdrive.locationd.calibrationd import *
#from tools.sim.lib.simulated_sensors import SimulatedSensors


# pretend real speed : take a random number [-1, 1] and divide by 20 and add 1, multiply this by the current target speed
# tomorrow look at each times calculated gas/brake 

DEFAULT_SPEED = 10

class datastream:
    def __init__(self, csv_filepath):
        """Set default values for when datastream object is created. Incr / decr only set to 1%"""
        self.axis_increment = 0.01    # 1% incr / decr to adjusting speed
        self.axis_values = {'gb':0.1, 'steer': 0.}   # initialize gb to a small value for later on proportional calculation
        self.axes_order = ['gb', 'steer']
        self.csv_filepath = csv_filepath  
        self.control_sock = messaging.pub_sock('testJoystick')  # for sending gb value
        self.time_list = []
        self.speed_list = []
        self.csv_data = self.read_csv_data()

        # subscribe to get car real-time speed
        self.sm = messaging.SubMaster(['gpsLocationExternal'])
       
        # start time before communicating first car speed
        self.start_time = time.time() 

        # communicate the first speed to get the car moving
        dat = new_message('testJoystick')
        dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
        dat.testJoystick.buttons = [False]
        self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket


    def read_csv_data(self):
        """
        Read CSV file data into a list of dictionaries for indexing
        """
        # need to make sure of the unit of speed in the csv
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


    def get_actual_speed(self):
        """
        Return actual car speed in MPH
        """
        self.sm.update()
        gle = self.sm['gpsLocationExternal']
        print("get_actual_speed returned", gle.speed * 2.236)  # im assuming m/s so convert it to mph
        return gle.speed * 2.236


    def control_speed(self, start_time):
        """
        Runs a while loop until the last time in the CSV file is reached. Uses Proportional controller 
        """
        end_time = self.time_list[-1]
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # define sleep time (pause between iterations)
        interval = 0.02  # loop will run every 2 ms
        t0 = 0
        Vr0 = 0
        g0 = 0.1
        t1 = elapsed_time
   
        while True:
            if elapsed_time >= end_time:
                start_time = current_time
                elapsed_time = 0

            # calculate equation vars
            t2 = t1 + interval  # next time (next iteration), interval is an approximation (assume calculatiosn and communication time is tiny)
            # calculate target V
            Vt2 = self.get_target_speed(t2)
            
            # get current actual speed
            actual_speed = self.get_actual_speed()
            if actual_speed is not None:
                Vr1 = actual_speed
            else:
                Vr1 = DEFAULT_SPEED

            # solve equation for g1
            if Vt2 == Vr1:   # check if next speed = current actual speed
                g1 = 0
            elif Vr1 == Vr0:
                g1 = 0.1  # default gb
            else:
                g1 = g0 * (t1 - t0) * (Vt2 - Vr1) / (Vr1 - Vr0) / interval
                print("Calculated g1:", g1)
              
            if g1 > 1:
                g1 = 1
            elif g1 < -1:
                g1 = -1

            # set gb to newly calculated gas brake
            self.axis_values['gb'] = g1
            print("ACTUAL g1:", self.axis_values['gb'])
            print("AT TIME:", t1)
            print("g0:", self.axis_values['gb'])
            print("t0:", t0)
            print("t1:", t1)
            print("t2:", t2)
            print("Vt2:", Vt2)
            print("Vr0:", Vr0)
            print("Vr1:", Vr1)

            # send control signals to Openpilot
            # messaging communicates info between different components of the system
            dat = new_message('testJoystick')
            dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
            dat.testJoystick.buttons = [False]
            self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket



            # calculate target speed here just to see if close
            Vt1 = self.get_target_speed(t1)
            print("target speed: ", Vt1, "---- current gb: ", self.axis_values['gb'])
            print("actual speed: ", Vr1)
            print("Difference target-actual:", Vt1 - Vr1)
            print()

            time.sleep(interval)  # adjustmnet occurs every 10ms, adjust if needed

            # calculate values for next loop
            current_time = time.time()
            
          
            elapsed_time = current_time - start_time
            t0 = t1
            Vr0 = Vr1
            Vr1 += 0.04
            g0 = g1
            if g0 == 0:
                g0 = 0.1
            t1 = elapsed_time


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                                 'openpilot must be offroad before starting joysticked.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--csv_file', help='CSV file containing time (s) and speed (mph)')
    args = parser.parse_args()# subscribe to get real-time info from openpilot
    

    if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
        print("The car must be off before running datastream.")
        exit()

    # create the data stream object
    ds = datastream(csv_filepath = args.csv_file)
    
    while True:
        ds.control_speed(ds.start_time)