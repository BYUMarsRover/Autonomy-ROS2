#!/usr/bin/env python3
import os
import rclpy
from rover_msgs.msg import ScienceSensorValues, ScienceSaveSensor, ScienceSaveNotes, ScienceSaveFAD, ScienceSaveSpectro
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class ScienceDataSaver(Node):

    def __init__(self):
        super().__init__('data_saver')

        self.science_save_fad = self.create_subscription(ScienceSaveFAD, '/science/save_fad', self.save_fad, 10)
        self.science_save_sensor = self.create_subscription(ScienceSaveSensor, '/science/save_sensor', self.save_sensor_callback, 10)
        self.science_save_notes = self.create_subscription(ScienceSaveNotes, '/science/save_notes', self.save_notes, 10)
        self.science_save_spectro = self.create_subscription(ScienceSaveSpectro, '/science/save_spectro', self.save_spectro, 10)

        self.sensor_map = ['moisture', 'temperature', 'fad', 'uv', 'als']
        self.sensor_values = [[] for _ in self.sensor_map]
        self.saving_sensor_values = [False for _ in self.sensor_map]
        self.sensor_count = 0
        self.fad_count = 0
        self.basedir = os.path.expanduser("~/science_data") #This is due to the install files usually not being writable. 
        os.makedirs(self.basedir, exist_ok=True)
        self.file_num = 1
        self.curr_site_num = 0

        self.get_logger().info('Science Data Save Started.')

    # I dont know much about the FAD data, this should be revisited by whomever is calibrating the FAD
    def save_fad(self, msg: ScienceSaveFAD):
        fname = f'site-{msg.site}-fad-list.csv'
        path = os.path.join(self.get_site_directory(msg.site), fname)
        if not os.path.exists(path):
            with open(path, "w") as f:
                f.write("Intensity,Observed Value\n")
        with open(path, "a") as f:
            f.write(str(msg.intensity) + " " + str(msg.observed))
        # self.get_logger().info(f"FAD saved at: {path}")

    def save_sensor_callback(self, msg: ScienceSaveSensor):
        fname = f'site-{msg.site}-{msg.measurement}-plot.csv'
        path = os.path.join(self.get_site_directory(msg.site), fname)
        if not os.path.exists(path):
            with open(path, "w") as f:
                f.write("Timestamp,Observed Value\n")
        with open(path, "a") as f:
            f.write(" ".join([str(x) for x in [msg.timestamp, msg.observed_value]]) + "\n")
        # self.get_logger().info(f"Sensor saved at: {path}")

    def save_notes(self, msg):
        print("Received notes", msg.notes)
        fname = f'site-{msg.site}-notes.txt'
        path = os.path.join(self.get_site_directory(msg.site), fname)
        with open(path, "w") as f:
            f.write(str(msg.notes))
        # Log a confirmation message with the path where the notes were saved
        self.get_logger().info(f"Notes saved at: {path}")

    def save_spectro(self, msg: ScienceSaveSpectro):
        wavelengths = [410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940]
        path = self.get_indexed_file_name(f'site-{msg.site}-spectro', 'csv', self.get_site_directory(msg.site))
        with open(path, "w") as f:
            f.write("Wavelength,Channel Value\n")
            for wavelength, channel in zip(wavelengths, msg.channels):
                f.write(f"{wavelength},{channel}\n")
        self.get_logger().info(f"Spectrograph saved at: {path}")

    def get_indexed_file_name(self, prefix, extension, basedir):
        """
        Generates a unique file name by appending an index to the prefix.
        """
        index = 1
        while os.path.exists(os.path.join(basedir, f"{prefix}-{index}.{extension}")):
            index += 1
        return os.path.join(basedir, f"{prefix}-{index}.{extension}")
    
    def get_site_directory(self, site_num):
        directory_name = f'site-{site_num}'
        path = os.path.join(self.basedir, directory_name) #THIS COULD CAUSE PROBLEMS!
        try:
            os.mkdir(path)
        except FileExistsError:
            print("This directory already exists")
        return path


def main(args=None):
    rclpy.init(args=args)
    data_saver = ScienceDataSaver()
    rclpy.spin(data_saver)
    data_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()