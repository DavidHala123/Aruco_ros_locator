import math
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import numpy as np
import rclpy
from rclpy import publisher
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import time
import subprocess
import matplotlib.pyplot as plt
import xlsxwriter

import yaml


class accuracyMeas(Node):

    def __init__(self):
        super().__init__('accuracy_measurements')

        self.get_logger().info("THIS NODE SENDS CAMERA INFO TO ROSBAG FILES AND CALCULATES MEAN")

        self.declare_parameter('folders_corresponds_to', "z")
        self.declare_parameter("real_x", "")
        self.declare_parameter("real_y", "")
        self.declare_parameter("real_z", "")

        self.declare_parameter("real_rx", "")
        self.declare_parameter("real_ry", "")
        self.declare_parameter("real_rz", "")
        self.declare_parameter("rosbag_path", "/home")
        self.declare_parameter('cam_info_path')

        self.real_x = self.get_parameter("real_x").get_parameter_value().string_value
        self.real_y = self.get_parameter("real_y").get_parameter_value().string_value
        self.real_z = self.get_parameter("real_z").get_parameter_value().string_value
        self.real_rx = self.get_parameter("real_rx").get_parameter_value().string_value
        self.real_ry = self.get_parameter("real_ry").get_parameter_value().string_value
        self.real_rz = self.get_parameter("real_rz").get_parameter_value().string_value
        self.rosbag_path = self.get_parameter('rosbag_path').value
        self.folders_corresponds_to = self.get_parameter("folders_corresponds_to").value
        self.camI_path = self.get_parameter('cam_info_path').value

        with open(self.camI_path, 'r') as file:
            self.calib_data = yaml.safe_load(file)

        if self.folders_corresponds_to == "x":
            self.axisIndex = 0
            self.real_x = []
            for file in os.listdir(self.rosbag_path):
                try:
                    self.real_x.append(float(file))
                except:
                    continue

            self.real_x = sorted(self.real_x)

            self.reference_axis = self.real_x

        if self.folders_corresponds_to == "y":
            self.axisIndex = 1
            self.real_y = []
            for file in os.listdir(self.rosbag_path):
                try:
                    self.real_y.append(float(file))
                except:
                    continue

            self.real_y = sorted(self.real_y)

            self.reference_axis = self.real_y

        if self.folders_corresponds_to == "z":
            self.axisIndex = 2
            self.real_z = []
            for file in os.listdir(self.rosbag_path):
                try:
                    self.real_z.append(float(file))
                except:
                    continue
            
            self.real_z = sorted(self.real_z)

            self.reference_axis = self.real_z

        self.get_logger().info(str(self.reference_axis))


        axis = [self.real_x, self.real_y, self.real_z, self.real_rx, self.real_ry, self.real_rz]

        for i in range(len(axis)):
            if not isinstance(axis[i], np.ndarray):
                axis[i] = np.zeros(len(self.reference_axis))

        self.real_x, self.real_y, self.real_z, self.real_rx, self.real_ry, self.real_rz = axis

        self.meanTR = [0.0, 0.0, 0.0]
        self.meanROT = [0.0, 0.0, 0.0]
        self.valsTR = [[0.0],[0.0],[0.0]]
        self.valsROT = [[0.0],[0.0],[0.0]]
        self.SQRE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.accuracy_arr = []
        self.failrate_arr = []
        self.bag_fileNr = 0
        self.allowSave = False
        self.samples = 0
        self.lastCall = 0
        self.frames = 0
        self.lastCallImage = 0
        self.xlsxCol = 0
        self.xlsxRow = 1

        self.workbook = xlsxwriter.Workbook(self.rosbag_path + '/values.xlsx')
        self.worksheetVals = self.workbook.add_worksheet()
        self.worksheetStats = self.workbook.add_worksheet()

        self.worksheetVals.write(0, 0, self.reference_axis[self.bag_fileNr])
        self.worksheetVals.write(1, 0 , 'X')
        self.worksheetVals.write(1, 1, 'Y')
        self.worksheetVals.write(1, 2, 'Z')
        self.worksheetVals.write(1, 3, 'Rx')
        self.worksheetVals.write(1, 4, 'Ry')
        self.worksheetVals.write(1, 5, 'Rz')


        self.worksheetStats.write(1, 2, 'X')
        self.worksheetStats.write(2, 2, 'mean')
        self.worksheetStats.write(2, 3, 'std')
        self.worksheetStats.write(2, 4, 'RMSE')
        self.worksheetStats.write(1, 5, 'Y')
        self.worksheetStats.write(2, 5, 'mean')
        self.worksheetStats.write(2, 6, 'std')
        self.worksheetStats.write(2, 7, 'RMSE')
        self.worksheetStats.write(1, 8, 'Z')
        self.worksheetStats.write(2, 8, 'mean')
        self.worksheetStats.write(2, 9, 'std')
        self.worksheetStats.write(2, 10, 'RMSE')
        self.worksheetStats.write(1, 11, 'Rx')
        self.worksheetStats.write(2, 11, 'mean')
        self.worksheetStats.write(2, 12, 'std')
        self.worksheetStats.write(2, 13, 'RMSE')
        self.worksheetStats.write(1, 14, 'Ry')
        self.worksheetStats.write(2, 14, 'mean')
        self.worksheetStats.write(2, 15, 'std')
        self.worksheetStats.write(2, 16, 'RMSE')
        self.worksheetStats.write(1, 17, 'Rz')
        self.worksheetStats.write(2, 17, 'mean')
        self.worksheetStats.write(2, 18, 'std')
        self.worksheetStats.write(2, 19, 'RMSE')

        self.plot_url = [
            self.rosbag_path + "/accuracy.png",
            self.rosbag_path + "/fail_rate.png"
            ]
        
        poseListener = self.create_subscription(PoseWithCovarianceStamped, '/aruco_ros_locator/pose', self.getTransform, 1)
        frameListener = self.create_subscription(Image, '/marker_publisher/result', self.frameAccepted, 1)
        self.CamIpub = self.create_publisher(CameraInfo, '/camera_info', 10)      

        self.timer2 = self.create_timer(2.0, self.saveToFile)

        self.sendCamInfo()
        self.playBag()

    def frameAccepted(self, Image):
        self.lastCallImage = time.time()
        self.frames += 1



    def sendCamInfo(self):
        camera_info_msg = CameraInfo()
        camera_info_msg.width = self.calib_data["image_width"]
        camera_info_msg.height = self.calib_data["image_height"]
        camera_info_msg.k = self.calib_data["camera_matrix"]["data"]
        camera_info_msg.d = self.calib_data["distortion_coefficients"]["data"]
        camera_info_msg.r = self.calib_data["rectification_matrix"]["data"]
        camera_info_msg.p = self.calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = self.calib_data["distortion_model"]
        self.CamIpub.publish(camera_info_msg)
        time.sleep(2)



    def calculateSTD(self, listTR, listROT, samples, meanROT, meanTR):
        listTR[0] = listTR[0][1:]
        listROT[0] = listROT[0][1:]
        listTR[1] = listTR[1][1:]
        listROT[1] = listROT[1][1:]
        listTR[2] = listTR[2][1:]
        listROT[2] = listROT[2][1:]
        outpTR = [0,0,0]
        outpROT = [0,0,0]
        for idx, list in enumerate(listTR):
            for val in list:
                outpTR[idx] += math.pow(val - meanTR[idx], 2)

            outpTR[idx] = math.sqrt(outpTR[idx]/(samples - 1))

        for idx, list in enumerate(listROT):
            for val in list:
                outpROT[idx] += math.pow(val - meanROT[idx], 2)

            outpROT[idx] = math.sqrt(outpROT[idx]/(samples - 1))

        return [outpTR, outpROT]


    
    def playBag(self):
        self.valsTR = [[0],[0],[0]]
        self.valsROT = [[0],[0],[0]]
        dir = self.rosbag_path + "/" + str(self.reference_axis[self.bag_fileNr]) + "/"
        fileName = ""
        for file in os.listdir(dir):
            if 'rosbag' in file:
                fileName = file

        
        self.get_logger().info("Playing bag file: '" + file + "' from directory: '" + dir + "'")
        player_proc = subprocess.Popen(['ros2', 'bag' , 'play', file], cwd=dir)



    def getTransform(self, pose: PoseWithCovarianceStamped):
        self.lastCall = time.time()
        self.allowSave = True

        try:
            self.meanTR[0] += pose.pose.pose.position.x
            self.meanTR[1] += pose.pose.pose.position.y
            self.meanTR[2] += pose.pose.pose.position.z

            self.SQRE[0] += math.pow(pose.pose.pose.position.x - self.real_x[self.bag_fileNr], 2)
            self.SQRE[1] += math.pow(pose.pose.pose.position.y - self.real_y[self.bag_fileNr], 2)
            self.SQRE[2] += math.pow(pose.pose.pose.position.z - self.real_z[self.bag_fileNr], 2)
            self.SQRE[3] += math.pow(pose.pose.pose.orientation.x - 0, 2)
            self.SQRE[4] += math.pow(pose.pose.pose.orientation.y - 0, 2)
            self.SQRE[5] += math.pow(pose.pose.pose.orientation.z - 0, 2)
            
            self.meanROT[0] += pose.pose.pose.orientation.x
            self.meanROT[1] += pose.pose.pose.orientation.y
            self.meanROT[2] += pose.pose.pose.orientation.z

            self.valsTR[0].append(pose.pose.pose.position.x)
            self.valsTR[1].append(pose.pose.pose.position.y)
            self.valsTR[2].append(pose.pose.pose.position.z)

            self.valsROT[0].append(pose.pose.pose.orientation.x)
            self.valsROT[1].append(pose.pose.pose.orientation.y)
            self.valsROT[2].append(pose.pose.pose.orientation.z)


            self.xlsxRow += 1

            self.worksheetVals.write(self.xlsxRow, self.xlsxCol, pose.pose.pose.position.x)
            self.worksheetVals.write(self.xlsxRow, self.xlsxCol + 1, pose.pose.pose.position.y)
            self.worksheetVals.write(self.xlsxRow, self.xlsxCol + 2, pose.pose.pose.position.z)

            self.worksheetVals.write(self.xlsxRow, self.xlsxCol + 3, pose.pose.pose.orientation.x)
            self.worksheetVals.write(self.xlsxRow, self.xlsxCol + 4, pose.pose.pose.orientation.y)
            self.worksheetVals.write(self.xlsxRow, self.xlsxCol + 5, pose.pose.pose.orientation.z)

            self.get_logger().info(f"Transform acquired")
            self.samples += 1

         
        except Exception as ex:
                self.get_logger().info(f'Could not get a transform: {ex}')



    def plotGraph(self):
        
        xAxisNumbers = []
        for number in self.reference_axis:
            if not number == 0.0:
                xAxisNumbers.append(round(number, 2))

        plt.figure()
        plt.plot(xAxisNumbers, self.accuracy_arr)
        plt.xlabel("Distance [m]")
        plt.ylabel("Error abs [m]")
        plt.savefig(self.plot_url[0])

        plt.figure()
        plt.plot(xAxisNumbers, self.failrate_arr)
        plt.xlabel("Distance [m]")
        plt.ylabel("Fail rate [%]")
        plt.savefig(self.plot_url[1])


    def saveToFile(self):

        canEnd1 = self.allowSave and time.time() - self.lastCall >= 2
        canEnd2 = not canEnd1 and self.samples == 0 and not self.frames == 0 and time.time() - self.lastCallImage >= 3
        if not canEnd1 and not canEnd2:
            return
        elif canEnd1:
            if(self.samples > 0):
                self.meanTR[0] /= self.samples
                self.meanTR[1] /= self.samples
                self.meanTR[2] /= self.samples
                self.meanROT[0] /= self.samples
                self.meanROT[1] /= self.samples
                self.meanROT[2] /= self.samples


                self.SQRE[0] = math.sqrt(self.SQRE[0]/self.samples)
                self.SQRE[1] = math.sqrt(self.SQRE[1]/self.samples)
                self.SQRE[2] = math.sqrt(self.SQRE[2]/self.samples)
                self.SQRE[3] = math.sqrt(self.SQRE[3]/self.samples)
                self.SQRE[4] = math.sqrt(self.SQRE[4]/self.samples)
                self.SQRE[5] = math.sqrt(self.SQRE[5]/self.samples)
                self.get_logger().info(str(self.samples) + "<---------------")
                self.get_logger().info(str(self.frames) + "<---------------")
                failRate = 1 - (self.samples / self.frames)
                accuracy = abs(self.reference_axis[self.bag_fileNr] - self.meanTR[self.axisIndex])
                [stdTR, stdROT] = self.calculateSTD(self.valsTR, self.valsROT,self.samples, self.meanROT, self.meanTR)
                  
            self.failrate_arr.append(round(failRate, 2))
            self.accuracy_arr.append(round(accuracy, 4))
            
            self.worksheetStats.write(3 + self.bag_fileNr, 1, self.reference_axis[self.bag_fileNr])
            self.worksheetStats.write(3 + self.bag_fileNr, 2, self.meanTR[0])
            self.worksheetStats.write(3 + self.bag_fileNr, 5, self.meanTR[1])
            self.worksheetStats.write(3 + self.bag_fileNr, 8, self.meanTR[2])
            self.worksheetStats.write(3 + self.bag_fileNr, 11, self.meanROT[0])
            self.worksheetStats.write(3 + self.bag_fileNr, 14, self.meanROT[1])
            self.worksheetStats.write(3 + self.bag_fileNr, 17, self.meanROT[2])
            self.worksheetStats.write(3 + self.bag_fileNr, 3, stdTR[0])
            self.worksheetStats.write(3 + self.bag_fileNr, 6, stdTR[1])
            self.worksheetStats.write(3 + self.bag_fileNr, 9, stdTR[2])
            self.worksheetStats.write(3 + self.bag_fileNr, 12, stdROT[0])
            self.worksheetStats.write(3 + self.bag_fileNr, 15, stdROT[1])
            self.worksheetStats.write(3 + self.bag_fileNr, 18, stdROT[2])
            self.worksheetStats.write(3 + self.bag_fileNr, 4, self.SQRE[0])
            self.worksheetStats.write(3 + self.bag_fileNr, 7, self.SQRE[1])
            self.worksheetStats.write(3 + self.bag_fileNr, 10, self.SQRE[2])
            self.worksheetStats.write(3 + self.bag_fileNr, 13, self.SQRE[3])
            self.worksheetStats.write(3 + self.bag_fileNr, 16, self.SQRE[4])
            self.worksheetStats.write(3 + self.bag_fileNr, 19, self.SQRE[5])

        elif canEnd2:

            self.worksheetStats.write(3 + self.bag_fileNr, 1, self.reference_axis[self.bag_fileNr])
            self.worksheetStats.write(3 + self.bag_fileNr, 2, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 5, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 8, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 11, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 14, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 17, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 3, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 6, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 9, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 12, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 15, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 18, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 4, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 7, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 10, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 13, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 16, '/')
            self.worksheetStats.write(3 + self.bag_fileNr, 19, '/')

            self.reference_axis[self.bag_fileNr] = 0.0

        self.get_logger().info(f"DONE WRITING")
        self.bag_fileNr += 1
        self.meanTR = [0, 0, 0]
        self.meanROT = [0, 0, 0]
        self.valsTR = [0, 0, 0]
        self.valsROT = [0, 0, 0]
        self.SQRE = [0,0,0,0,0,0]
        self.allowSave = False
        self.frames = 0
        self.samples = 0
        

        if(self.bag_fileNr < len(self.reference_axis)):
            self.xlsxCol += 7
            self.worksheetVals.write(0, self.xlsxCol, self.reference_axis[self.bag_fileNr])
            self.xlsxRow = 2
            self.worksheetVals.write(1, self.xlsxCol, 'X')
            self.worksheetVals.write(1, self.xlsxCol + 1, 'Y')
            self.worksheetVals.write(1, self.xlsxCol + 2, 'Z')
            self.worksheetVals.write(1, self.xlsxCol + 3, 'Rx')
            self.worksheetVals.write(1, self.xlsxCol + 4, 'Ry')
            self.worksheetVals.write(1, self.xlsxCol + 5, 'Rz')

            self.playBag()
        else:
            self.workbook.close()
            self.get_logger().info("Workbook closed")
            self.plotGraph()


def main():
    rclpy.init()
    node = accuracyMeas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()