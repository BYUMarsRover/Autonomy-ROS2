#!/usr/bin/env python3
import os
import rclpy
from rover_msgs.msg import ScienceSensorValues, ScienceSaveSensor, ScienceSaveNotes, ScienceFADIntensity
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class ScienceDataSaver(Node):

    def __init__(self):
        super().__init__('data_saver')

        self.science_sensor_values = self.create_subscription(ScienceSensorValues, '/science_sensor_values', self.sensor_values_callback, 10)
        self.science_fad_value = self.create_subscription(ScienceFADIntensity, '/science_fad_value', self.fad_value_callback, 10)
        self.science_save_sensor = self.create_subscription(ScienceSaveSensor, '/science_save_sensor', self.save_sensor_callback, 10)
        self.science_save_notes = self.create_subscription(ScienceSaveNotes, '/science_save_notes', self.save_notes, 10)

        self.sensor_values = [[], [], []]
        self.saving_sensor_values = [False, False, False]
        self.sensor_count = 0
        self.fad_count = 0
        # self.DIR = os.path.join(os.path.dirname(__file__), "resources")
        self.DIR = os.path.expanduser("~/science_data") #This is due to the install files usually not being writable. 
        os.makedirs(self.DIR, exist_ok=True)
        self.file_num = 1
        self.curr_site_num = 0
        self.sensor_map = ['moisture', 'temperature', 'fad']

        print('Science Data Save Started.')
    
    # def get_file_num(self):
    #     self.file_num += 1
    #     return self.file_num - 1

    def sensor_values_callback(self, msg: ScienceSensorValues):
        # Prevents the sensor data from getting saved constantly, only saved every 60 calls
        # self.sensor_count = (self.sensor_count + 1) % 60
        self.sensor_count = 0 #For testing purposes, go back to the mod 60 for actual stuff
        # print("save")
        print(self.saving_sensor_values, self.sensor_count, self.sensor_values)
        if self.saving_sensor_values[0] and self.sensor_count == 0:
            self.sensor_values[0].append(msg.moisture)
        
        if self.saving_sensor_values[1] and self.sensor_count == 0:
            self.sensor_values[1].append(msg.temperature)

    def fad_value_callback(self, msg: ScienceFADIntensity):
        self.fad_count = (self.fad_count + 1) % 60

        if self.saving_sensor_values[2] and self.fad_count == 0:
            self.sensor_values[2].append(msg.intensity_avg)

    def save_sensor_callback(self, msg: ScienceSaveSensor):
        print('Recieved.')
        self.saving_sensor_values[msg.position] = msg.save

        if msg.site != self.curr_site_num:
            self.change_site_directory(msg.site)
        
        if msg.save: # here put in the actual value as the first value
            self.sensor_values[msg.position] = [float(msg.observed_value)]
        else: #Change this f string so that it gets this going in the correct directory

            file_name = f'{self.sensor_map[msg.position]}-plot-{self.file_num}.txt'
            # file_name = f'{presentation_path}/{self.sensor_map[msg.position]}-plot-{self.get_file_num()}.txt'
            print(file_name)
            file_path = os.path.join(self.DIR, file_name)
            self.save_sensor_values(msg.position, file_path)

    def change_site_directory(self, site_num):
        self.curr_site_num = site_num
        directory_name = f'site-{self.curr_site_num}'
        self.DIR = os.path.join(self.DIR, directory_name) #THIS COULD CAUSE PROBLEMS!
        try:
            os.mkdir(self.DIR)
        except FileExistsError:
            print("This directory already exists")
    
    def save_sensor_values(self, p: int, file_path):
        print(f'Saving Sensor {p} Values.')
        print(self.sensor_values)
        with open(file_path, "a") as f:
            f.write(" ".join(map(str, self.sensor_values[p])) + "\n")
        print("Sensor Values Saved.")

    def save_notes(self, msg):
        print("Received notes", msg.notes)
        notes = str(msg.notes)
        fname = f'{msg.site}-notes-{self.file_num}.txt'
        with open(os.path.join(self.DIR, fname), "w") as f:
            f.write(notes)
        print('Notes Saved.')

def main(args=None):
    rclpy.init(args=args)
    data_saver = ScienceDataSaver()
    rclpy.spin(data_saver)
    data_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()