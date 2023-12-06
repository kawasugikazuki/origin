## 	@package Test packageのご説明（一覧で表示されるときはここが表示されます。）
#   @brief briefにはpackageのくわしい説明をかけます（詳細ではここが表示されます
#   @author packageの作者：私です。やめてください死んでしまいます。^o^
#   @date   packageの日付はここ。2018.04.24
#   @version  packageのバージョンはここ1.1
# @file calc.py

import cv2
import numpy as np
import os
import socket
import getpass

class Calc():

    ## Distance between markers[cm]
    __H = 80

    __MARK_DIR = 'mark/'
    __MARK_EXT = '.png'

    ## This is the constructor of this class
    # @brief Creates a mark directory and reads lens calibration parameters(@link __get_calib_coefficient @endlink) using its own IP address (@link __get_myIP @endlink)
    # @param None
    # @return None
    def __init__(self) :
        self.__get_myIP()
        self.__get_calib_coefficient()
        if not os.path.exists(self.__MARK_DIR) :
            os.makedirs(self.__MARK_DIR)

    ## Get own IP address
    # @brief Obtain own IP address
    # @param None
    # @return None
    # @anchor __get_myIP
    def __get_myIP(self) :
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8",80))
        self.__myIP = s.getsockname()[0]
        self.__username = getpass.getuser()
        s.close()

    ## Get own calibration parameter
    # @brief Reads its own calibration parameters using its own IP address (@link __get_myIP @endlink)
    # @brief Calibration parameters are listed in calibration.txt
    # @param None
    # @return None
    # @anchor __get_calib_coefficient
    def __get_calib_coefficient(self) :
        calib_txt = np.loadtxt('/home/red/Red2.0-Control-Software/explore_algorithm/calibration.txt', delimiter=',', skiprows=1, dtype='str')
        index = np.where(calib_txt == self.__myIP)[0]
        row = calib_txt[index].reshape(10)
        row = np.delete(row, 0).astype(np.float32)
        print('read params : ' + str(row))
        self.__Cx = row[0]
        self.__Cy = row[1]
        self.__a = np.array([row[2], row[3], row[4], row[5]])
        self.__c = row[6]
        self.__d = row[7]
        self.__e = row[8]
        self.__C = np.array([self.__Cx, self.__Cy])
        self.__A = np.array([[self.__c, self.__d], [self.__e, 1]])

    ## Obtain the marker's coordinate values in the opencv image coordinate system
    # @brief The coordinates of the marker are obtained from the mask image. However, if there are multiple candidate points, the coordinate value of the center of gravity is used as the marker
    # @param result one mask image(grayscale image)
    # @return <class 'numpy.ndarray'> shape=(2,) [x y](opencv image coordinates)
    # @anchor __get_marker_point2D
    def __get_marker_point2D(self, result) :
        result = result.astype(bool)
        index = np.array(np.where(result == True))
        marker_point = np.sum(index, axis=1) / index.shape[1]
        marker_point = np.roll(marker_point, 1)
        return marker_point

    ## Transformation image coordinate -> sensor coordinate
    # @brief This function converts image coordinates to sensor coordinates
    # @param point <class 'numpy.ndarray'> shape=(2,) [x y](opencv image coordinates)
    # @return <class 'numpy.ndarray'> shape=(2, 1) [x y](sensor coordinates)
    # @anchor __image_coordinate2sensor_coordinate
    def __image_coordinate2sensor_coordinate(self, point) :
        point = (point - self.__C).reshape(2, 1)
        point = np.matmul(np.linalg.inv(self.__A), point)
        return np.ravel(point)

    ## Transformation sensor coordinate -> camera coordinate
    # @brief This function converts sensor coordinates to camera coordinates
    # @param point <class 'numpy.ndarray'> shape=(2,) [x y](sensor coordinates)
    # @return <class 'numpy.ndarray'> shape=(3,) [x y z](camera coordinates)
    # @anchor __sensor_coordinate2camera_coordinate
    def __sensor_coordinate2camera_coordinate(self, point) :
        r = np.sqrt(np.sum(np.square(point)))
        z = -(self.__a[0] + self.__a[1] * r**2 + self.__a[2] * r**3 + self.__a[3] * r**4)
        point3D = np.append(point, z)
        return point3D

    ## Transformation camera coordinate -> robot coordinate
    # @brief This function converts camera coordinates to robot coordinates
    # @param point <class 'numpy.ndarray'> shape=(3,) [x y z](camera coordinates)
    # @return <class 'numpy.ndarray'> shape=(3,) [x y z](robot coordinates)
    # @anchor __camera_coordinate2robot_coordinate
    def __camera_coordinate2robot_coordinate(self, point3D) :
        theta = np.deg2rad(-90)
        rot = np.array([[np.cos(theta), -np.sin(theta), 0],\
                        [np.sin(theta), np.cos(theta), 0],\
                        [0, 0, 1]])
        point3D = np.matmul(rot, point3D)
        return point3D

    ## Generate rotation matrix around x-axis
    # @brief Generate 3D rotation matrix with the roll angle as an argument
    # @param roll [deg]
    # @return <class 'numpy.ndarray'> shape=(3, 3) [[1 0 0], [0 cos -sin], [0 sin cos]]
    # @anchor __get_Rx
    def __get_Rx(self, roll) :
        Rx = np.array([[1, 0, 0],\
                       [0, np.cos(roll), -np.sin(roll)],\
                       [0, np.sin(roll), np.cos(roll)]])
        return Rx
    ## Generate rotation matrix around y-axis
    # @brief Generate 3D rotation matrix with the pitch angle as an argument
    # @param pitch [deg]
    # @return <class 'numpy.ndarray'> shape=(3, 3) [[cos 0 sin], [0 1 0], [-sin 0 cos]]
    # @anchor __get_Ry
    def __get_Ry(self, pitch) :
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],\
                       [0, 1, 0],\
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        return Ry

    ## Correcting the robot's posture
    # @brief Correct the robot's posture using @link __get_Rx @endlink and @link __get_Ry @endlink
    # @param point3D <class 'numpy.ndarray'> shape=(3,) [x y z](robot coordinates)
    # @return <class 'numpy.ndarray'> shape=(3,) [x y z](robot coordinates)
    # @anchor __posture_angle_correction
    def __posture_angle_correction(self, point3D) :
        Rx = self.__get_Rx(self.GYRO_ANGLES[0])
        Ry = self.__get_Ry(self.GYRO_ANGLES[1])
        #Rx = self.__get_Rx(0)
        #Ry = self.__get_Ry(0)
        point3D = np.matmul(np.matmul(Ry, Rx), point3D)
        return point3D

    ## Transformation image coordinate -> robot coordinate
    # @brief This function converts image coordinates to robot coordinates @link __image_coordinate2sensor_coordinate @endlink, @link __sensor_coordinate2camera_coordinate @endlink, @link __camera_coordinate2robot_coordinate @endlink, @link __posture_angle_correction @endlink
    # @param marker_point <class 'numpy.ndarray'> shape=(2,) [x y](opencv image coordinates)
    # @return <class 'numpy.ndarray'> shape=(3,) [x y z](robot coordinates)
    # @anchor __image_coodinate2robot_coordinate
    def __image_coodinate2robot_coordinate(self, marker_point) :
        point = self.__image_coordinate2sensor_coordinate(marker_point)
        point3D = self.__sensor_coordinate2camera_coordinate(point)
        point3D = self.__camera_coordinate2robot_coordinate(point3D)
        point3D = self.__posture_angle_correction(point3D)
        return point3D

    ## Obtain the marker's coordinate values in the robot coordinate system
    # @brief Obtain the coordinate values of markers in the robot coordinate system from grayscale images
    # @param result one mask image(grayscale image)
    # @return <class 'numpy.ndarray'> shape=(3,) [x y z](robot coordinates)
    # @anchor __get_point3D_robot_coordinate
    def __get_point3D_robot_coordinate(self, result) :
        point2D = self.__get_marker_point2D(result)
        point3D = self.__image_coodinate2robot_coordinate(point2D)
        return point3D

    ## Estimate the distance and azimuth from the real marker
    # @brief Estimates distance and azimuth from a marker using the coordinate values in the robot coordinate system of two markers mounted on one post
    # @param point3D <class 'numpy.ndarray'> shape=(2, 3) [[x1, y1, z1], [x2, y2, z2]](robot coordinates)
    # @return <class 'tuple'> len=2 (r[cm], phi[rad])(robot coordinates)
    # @anchor __get_distance_phi
    def __get_distance_phi(self, point3D) :
        r = np.sqrt(point3D[:, 0]**2 + point3D[:, 1]**2)
        incidence_angle = np.arctan2(r, point3D[:, 2])
        theta = np.abs(np.diff(incidence_angle))[:]
        tmp = (self.__H * np.sin(incidence_angle[0])) / np.sin(theta)
        distance = float(tmp) * np.sin(incidence_angle[1])
        phi = np.mean(np.arctan2(point3D[:, 1], point3D[:, 0]))
        if phi < 0 : phi = (2 * np.pi) + phi
        return distance, phi

    ## Transformation Polar coordinate -> Cartesian coordinate
    # @brief This function converts polar coordinates to cartesian coordinates
    # @param point <class 'numpy.ndarray'> shape=(2,2) [[r1[cm], r2[cm]], [phi1[rad], phi2[rad]]](robot coordinates)
    # @return <class 'numpy.ndarray'> shape=(2,2) [[x1, x2], [y1, y2]](robot coordinates)
    # @anchor __polar2cartesian
    def __polar2cartesian(self, point) :
        x = point[0, :] * np.cos(point[1, :])
        y = point[0, :] * np.sin(point[1, :])
        point = np.array([x, y])
        return point

    ## Transformation Cartesian coordinate -> Polar coordinate
    # @brief This function converts cartesian coordinates to polar coordinates
    # @param point <class 'numpy.ndarray'> shape=(2,) [x, y](robot coordinates or virtual marker coordinates)
    # @return <class 'tuple'> len=2 (r[cm], phi[rad])(robot coordinates)
    # @anchor __cartesian2polar
    def __cartesian2polar(self, point) :
        r = np.sqrt(np.sum(np.square(point)))
        phi = np.arctan2(point[1], point[0])
        if phi < 0 : phi = (2 * np.pi) + phi
        return r, phi

    ## Coordinate transformation between robot coordinate system and virtual marker coordinate system
    # @brief Obtain polar coordinate values from virtual markers
    # @param point <class 'numpy.ndarray'> shape=(2,2) [[x1, x2], [y1, y2]](robot coordinates)
    # @param virtual_marker <class 'list'> len = 2 [x, y](virtual marker coordinates)
    # @return <class 'tuple'> len=2 (r[cm], phi[rad])(robot coordinates)
    # @anchor __getTfArray_virtual_robot
    def __getTfArray_virtual_robot(self, point, virtual_marker = [0, 0]) :
        ov = np.sum(point, axis=1) / 2
        xv = point[:, 0] - ov
        xv /= np.linalg.norm(xv)
        theta = np.pi/2
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        yv = np.matmul(rot, xv)
        Ub = np.array([[1, 0], [0, 1]])
        Uv = np.array([[xv[0], yv[0]], [xv[1], yv[1]]])
        R = np.linalg.inv(np.dot(Ub, Uv.T))
        tf = np.array([[R[0, 0], R[0, 1], ov[0]],
                       [R[1, 0], R[1, 1], ov[1]],
                       [0, 0, 1]])
        marker_point = np.dot(tf, np.array([virtual_marker[0], virtual_marker[1], 1]))
        robot_point = np.dot(np.linalg.inv(tf), np.array([0, 0, 1]))
        marker_point_ans = self.__cartesian2polar(marker_point[:2])
        robot_point_ans = self.__cartesian2polar(robot_point[:2])
        #print('--- ロボット座標系 ---')
        #print('マーカまでの距離 : ' + str(marker_point_ans[0]) + '[cm]')
        #print('マーカまでの方位角 : ' +str(np.rad2deg(marker_point_ans[1])) + '[deg]')
        #print('--- 仮想マーカ座標系 ---')
        #print('x[cm] : ' + str(robot_point[0]) + '\ty[cm] : ' + str(robot_point[1]))
        #print('ロボットまでの距離 : ' + str(robot_point_ans[0]) + '[cm]')
        #print('ロボットまでの方位角 : ' + str(np.rad2deg(robot_point_ans[1])) + '[deg]')
        return marker_point_ans

    ## Estimate the distance and azimuth from the virtual marker
    # @brief Estimates distance and azimuth from a virtual marker using the coordinate values in the robot coordinate system of two markers mounted on one post
    # @param distance <class 'numpy.ndarray'> shape=(2,) [r1[cm], r2[cm]](robot coordinates)
    # @param phi <class 'numpy.ndarray'> shape=(2,) [phi1[rad], phi2[rad]](robot coordinates)
    # @return <class 'tuple'> len=2 (r[cm], phi[rad])(robot coordinates)
    # @anchor __get_distance_phi_virtual
    def __get_distance_phi_virtual(self, distance, phi, vmarker) :
        point = np.array([distance, phi])
        point = self.__polar2cartesian(point)
        r, phi = self.__getTfArray_virtual_robot(point, vmarker)
        return r, phi

    ## This function estimates the distance and azimuth from a marker detection image
    # @brief The function estimates the distance and azimuth from the marker using the calibration parameters, using the marker detection image and frequency as arguments
    # @param result <class 'numpy.ndarray'> shape=(n, 768, 1024) (n : num of marker detection image)
    # @param marker_freq <class 'tuple'> len=n (n : num of frequency)
    # @return <class 'tuple'> len=2 (r[m], phi[rad])(robot coordinates)
    # @anchor main
    def main(self, result, marker_freq, vmarkerX, vmarkerY, betweenmarker,GYRO_ANGLES):
        if not self.__H == betweenmarker*100 : self.__H = betweenmarker*100
        distance = np.empty(shape=(0, 1))
        phi = np.empty(shape=(0, 1))
        self.GYRO_ANGLES = np.deg2rad(GYRO_ANGLES)
        for _ in range(len(marker_freq)) :
            point3D = np.empty(shape=(0, 3))
            point3D = np.array([np.append(point3D, self.__get_point3D_robot_coordinate(_)) for _ in result[:2]])
            tmp = self.__get_distance_phi(point3D)
            distance = np.append(distance, tmp[0])
            phi = np.append(phi, tmp[1])
            result = np.delete(arr=result, obj=slice(2), axis=0)
        if len(marker_freq) == 2 :
            vmarker = [vmarkerX*100, vmarkerY*100]
            tmp = self.__get_distance_phi_virtual(distance, phi, vmarker)
            distance = np.empty(shape=(0, 1))
            phi = np.empty(shape=(0, 1))
            distance = np.append(distance, tmp[0])
            phi = np.append(phi, tmp[1])

        distance = (distance.reshape(1, 1))*0.01
        phi = phi.reshape(1, 1)
        return distance, phi

if __name__ == '__main__' :
    x = Calc()
    print(x.main())