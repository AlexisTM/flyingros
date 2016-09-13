#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Kalman.py

Kalman.py was originaly just a Kalman filter and is now multiple implementatin of filters

This file is part of FlyingROS.

FlyingROS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

FlyingROS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with FlyingROS.  If not, see <http://www.gnu.org/licenses/>.

Software created by Alexis Paques and Nabil Nehri for the UCL
in a Drone-Based Additive Manufacturing of Architectural Structures
project financed by the MIT Seed Fund

Copyright (c) Alexis Paques 2016
"""


from __future__ import division
from copy import deepcopy

# Static 1D Filter
# I want the mean FAST
class KalmanStatic1D:
  def __init__(self, _ErrorMeasurement, _ErrorEstimate, _InitialMeasurement, _InitialEstimate = None):
    self.ErrorMeasurement = _ErrorMeasurement
    self.ErrorEstimate = _ErrorEstimate
    self.InitialMeasurement = _InitialMeasurement
    if(_InitialEstimate == None):
      self.PreviousEstimate = _InitialMeasurement
    else :
      self.PreviousEstimate = _InitialEstimate
    # Initiate everythings
    self.next(_InitialMeasurement)

  def next(self, Measurement):
    KalmanGain = self.ErrorEstimate / (self.ErrorEstimate + self.ErrorMeasurement)
    self.PreviousEstimate = self.PreviousEstimate + KalmanGain * (Measurement - self.PreviousEstimate)
    self.ErrorEstimate = (1-KalmanGain)*(self.ErrorEstimate)
    return self.PreviousEstimate

  def get(self):
    return self.PreviousEstimate


# Dynamic optimized 3D Filter with 6 lasers to get X, Y, Z, yaw
# NOT USED because the Kalman gain is stuck @ 0.37 since the error
# on the prediction and on the value are constant
class Custom3DKalman:
    def __init__(self, _ErrorMeasurements, _Error_Yaw, _InitialPrediction, process_covariance_noise_distance=0.01 , process_covariance_noise_angle=0.0174533):
        # Xk is the output value (and the initial value X0 in this case)
        # Should be : (X, Y, Z, Phi(yaw))
        # Should be : (meters, meters, meters, radians)
        self.Xk = _InitialPrediction

        # Measurments covariance matrix, imprecision on the lasers
        # Represents the DIAGONAL of the matrix
        self.R = [_ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _Error_Yaw*_Error_Yaw]

        # This is the Process covariance matrix
        # => 2cm for distances & 5 degrees for angle
        # Represents the DIAGONAL of the matrix
        self.Pk = [_ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _Error_Yaw*_Error_Yaw]

        # The Process Covariance NOISE is some additionnal error which is unpredictable or not modelised
        # => Walls are unprecised, could be X cm of error
        # => 1 cm for distances &  1 degree (0,0523599 radians) for angle
        # Represents the DIAGONAL of the matrix
        self.Q = [process_covariance_noise_distance*process_covariance_noise_distance, \
                   process_covariance_noise_distance*process_covariance_noise_distance, \
                   process_covariance_noise_distance*process_covariance_noise_distance, \
                   process_covariance_noise_angle*process_covariance_noise_angle]

    # Angular speed is in radians/s
    # Speed in m/s
    # Acceleration in m/(s*s)
    # Measures in meters
    def next(self, Measurements, Linear_speeds, Linear_accelerations, angular_speed, dt):
        # Predicted value
        dt2 = dt*dt
        # Position
        Xkp = [0.0, 0.0, 0.0, 0.0]
        Xkp[0] = self.Xk[0] + Linear_speeds[0]*dt + Linear_accelerations[0]*dt2
        Xkp[1] = self.Xk[1] + Linear_speeds[1]*dt + Linear_accelerations[1]*dt2
        Xkp[2] = self.Xk[2] + Linear_speeds[2]*dt + Linear_accelerations[2]*dt2
        Xkp[3] = self.Xk[3] + angular_speed*dt

        # Process Covariance Matrix is the precedent Process Covariance + noise
        Pkp = [0.0, 0.0, 0.0, 0.0]
        Pkp[0] = self.Pk[0] + self.Q[0]
        Pkp[1] = self.Pk[1] + self.Q[1]
        Pkp[2] = self.Pk[2] + self.Q[2]
        Pkp[3] = self.Pk[3] + self.Q[3]

        # Kalman Filter
        K = [0.0, 0.0, 0.0, 0.0]
        K[0] = Pkp[0] / (Pkp[0] + self.R[0])
        K[1] = Pkp[1] / (Pkp[1] + self.R[1])
        K[2] = Pkp[2] / (Pkp[2] + self.R[2])
        K[3] = Pkp[3] / (Pkp[3] + self.R[3])

        # Input of measures, improving values cause we got 2 lasers for each directions
        # 6 equations with 4 unknonw
        Yk = [0.0, 0.0, 0.0, 0.0]
        Yk[0] = (Measurements[0] + Measurements[1])/2 # X
        Yk[1] = (Measurements[2] + Measurements[3])/2 # Y
        Yk[2] = (Measurements[4] + Measurements[5])/2 # Z
        Yk[3] = (Measurements[6] + Measurements[7])/2 # yaw

        # New output value taking account the Kalman gain
        self.Xk[0] = Xkp[0] + K[0]*(Yk[0] - Xkp[0])
        self.Xk[1] = Xkp[1] + K[1]*(Yk[1] - Xkp[1])
        self.Xk[2] = Xkp[2] + K[2]*(Yk[2] - Xkp[2])
        self.Xk[3] = Xkp[3] + K[3]*(Yk[3] - Xkp[3])

        # Update the covariance matrix
        self.Pk[0] = (1-K[0])*Pkp[0]
        self.Pk[1] = (1-K[1])*Pkp[1]
        self.Pk[2] = (1-K[2])*Pkp[2]
        self.Pk[3] = (1-K[3])*Pkp[3]
        return self.Xk, K

def square(array):
  for a in range(len(array)):
    array[a] = array[a]*array[a]
  return array

class simple_filter:
    def __init__(self, Prediction_gain = 0.4, New_data_gain = 0.2, data_length = 10):
        # data length is the number of sample to take, AT LEAST 3
        self.max_length = data_length
        # Lower the gain is, the more we take in account the prediction
        # If no speed nor acceleration is given, use Kalman = 1
        # If you perfectly trust your Speed/Acceleration and there is no unknonw in the process
        # use 0.1
        # If you don't trust that much your Speed/Acceleration
        # use 0.6
        # If you do not know, just leave 0.4
        self.setGains(Prediction_gain, New_data_gain)
        self.data = []
        self.last_output = 0

    def next(self, NewData, dt = 1, Speed = 0, Acceleration = 0):
        if type(NewData) is list :
            self.data += NewData
            NewData = (NewData[0]+NewData[1])/2
        else :
            self.data.append(NewData)
        prediction = self.last_output + Speed*dt + Acceleration*dt*dt
        # Take the mean of the N last values
        data_mean = self.mean_unoutlier()
        # Remove the oldest data, multiple times if newData is a list
        while len(self.data) >=  self.max_length :
            self.data.pop(0)
        # calculate the output
        self.last_output = self.Prediction_gain*prediction + self.New_data_gain*NewData + self.Mean_gain*data_mean
        return self.last_output

    # Gains are 0.4 for prediction, 0.2 for new data, 0.6 for mean on STATIONARY flight
    # Gains are 0.65 for prediction, 0.35 for new data, 0 for mean on DYNAMIC flight
    def setGains(self, Prediction_gain = 0.4, New_data_gain = 0.2):
        self.Prediction_gain = Prediction_gain
        self.New_data_gain = New_data_gain
        self.Mean_gain = 1 - Prediction_gain - New_data_gain

    # remove one outlier and take the mean
    def mean_unoutlier(self):
        sorted_data = deepcopy(self.data)
        sorted_data.sort()
        # sort to avoid to use outliers by popping extremes, if we got enough data
        if len(self.data) >=  self.max_length :
            sorted_data.pop()
            sorted_data.pop(0)
        result = 0

        total = sum(v for v in sorted_data)
        return total/len(sorted_data)

# Applies the coeficients of the filter (B in MATLAB, h in the class) to the data
# NOTE : Applies too much delay
class simple_lowpass:
    def __init__(self):
        self.data = []
        self.last_output = 0
        self.h = h = [
            -0.000000000000000003,
            0.009489865835136195,
            0.047599359273529158,
            0.121261259186275366,
            0.202402690898081855,
            0.238493649613954889,
            0.202402690898081855,
            0.121261259186275394,
            0.047599359273529186,
            0.009489865835136214,
            -0.000000000000000003,
        ]
        self.max_length = len(self.h)
    def next(self, NewData):
        self.data.append(NewData)
        if len(self.data) >  self.max_length  :
            self.data.pop(0)
            return sum(self.data[v]*self.h[v] for v in range(self.max_length))
        return NewData


class simple_decay_filter :
    def __init__(self, decay = 0.93):
        self.b = 1 - decay
        self.y = 0
    def next(self, x):
        self.y += self.b * (x - self.y)
        return self.y

class filter_container :
    def __init__(self):
        self.X = simple_filter()
        self.Y = simple_filter()
        self.Z = simple_filter()
        self.Yaw = simple_filter()
        self.raw_x_1 = simple_decay_filter()
        self.raw_x_2 = simple_decay_filter()
        self.raw_y_1 = simple_decay_filter()
        self.raw_y_2 = simple_decay_filter()
        self.raw_z_1 = simple_decay_filter()
        self.raw_z_2 = simple_decay_filter()
        self.Vx = simple_decay_filter()
        self.Vy = simple_decay_filter()
        self.Vz = simple_decay_filter()
        self.Vyaw = simple_decay_filter()

    def filter_raw(self, raw):
        result = list()
        result.append(self.raw_x_1.next(raw[0]))
        result.append(self.raw_x_2.next(raw[1]))
        result.append(self.raw_y_1.next(raw[2]))
        result.append(self.raw_y_2.next(raw[3]))
        result.append(self.raw_z_1.next(raw[4]))
        result.append(self.raw_z_2.next(raw[5]))
        return result

    def filter_position(self, positions, dt, speeds, accelerations):
        result = list()
        result.append(self.X.next(positions[0], dt, speeds[0], accelerations[0]))
        result.append(self.Y.next(positions[1], dt, speeds[1], accelerations[1]))
        result.append(self.Z.next(positions[2], dt, speeds[2], accelerations[2]))
        result.append(self.Yaw.next(positions[3], dt, speeds[3], accelerations[3]))
        return result
