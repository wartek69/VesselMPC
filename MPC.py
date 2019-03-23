import sys
import numpy as np
import copy
import math
from sklearn.preprocessing import MinMaxScaler


from keras.engine.saving import load_model

rotdot = True;
class MPC:
    #tuning params
    rot1_max = 180
    rot1_min = -180
    rot1_dot = 1
    rot_tmax = 0.65
    rot_tmin = 0.35
    rot_tdot = 0.1
    rot2_max = 180
    rot2_min = -180
    rot2_dot = 1
    prediction_horizon = 60
    heading_weight = 5;

    def __init__(self):
        if rotdot:
            input_shape_mlp = 2
            self.scaler = MinMaxScaler(feature_range=(-1, 1))
            self.MLP_model = load_model('model/mlp_model_rotdot_10k_1000ep_2hidden.h5')
            # determine scaling factor
            dataset = np.loadtxt("generated_data/random_rot_dot/training_less_random_10k.data", delimiter=";", comments='#')
            x_train = dataset[:, 0:input_shape_mlp]
            self.scaler.fit_transform(x_train)
        else:
            input_shape_mlp = 3
            self.scaler = MinMaxScaler(feature_range=(-1, 1))
            self.MLP_model = load_model('model/mlp_model_eenvoudiger_200k_100ep.h5')
            #determine scaling factor
            dataset = np.loadtxt("generated_data/random/training_less_random_200k.data", delimiter=";", comments='#')
            x_train = dataset[:, 0:input_shape_mlp]
            self.scaler.fit_transform(x_train)

    def optimize(self, px, py, vessel_model):
        model = copy.copy(vessel_model)
        min_cost = sys.maxsize
        best_rot = 0;
        for i in range(self.rot1_min, self.rot1_max + 1, self.rot1_dot):
            for k in range(self.rot2_min, self.rot2_max + 1, self.rot2_dot):
                for l in np.arange(self.rot_tmin, self.rot_tmax, self.rot_tdot):
                    # vessel model should be reset
                    temp_model = copy.copy(model)
                    for t in range(self.prediction_horizon):
                        if t < self.prediction_horizon * l:
                            temp_model.simulate(i)
                        else:
                            temp_model.simulate(k)
                    xte, closest_index = self.calc_xte_improved(px, py, temp_model)
                    cost = self.angular_diff(temp_model.heading,
                                             self.get_heading_curve(px, py, closest_index)) ** 2 * self.heading_weight
                    cost += xte
                    if cost < min_cost:
                        min_cost = cost
                        best_rot = i
                        best_rot2 = k
                        best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return best_rot

    # optimization done by either increasing or decreasing the rot
    # the cost is recalculated every iteration
    def optimize_simple_accurate(self, px, py, vessel_model):
        model = copy.copy(vessel_model)
        min_cost = sys.maxsize
        best_rot = 0;
        for i in range(0, 3, 1):
            for k in range(0, 3, 1):
                for l in np.arange(self.rot_tmin, self.rot_tmax, self.rot_tdot):
                    # vessel model should be reset
                    temp_model = copy.copy(model)
                    cost = 0;
                    for t in range(self.prediction_horizon):
                        if t < self.prediction_horizon * l:
                            temp_model.simulate(self.__get_rot(i, temp_model.rot))
                        else:
                            temp_model.simulate(self.__get_rot(k, temp_model.rot))
                        xte, closest_index = self.calc_xte_improved(px, py, temp_model)
                        cost += (temp_model.heading - self.get_heading_curve(px, py,
                                                                             closest_index)) ** 2 * self.heading_weight;
                        cost += xte
                        if cost > min_cost:
                            break
                    if cost < min_cost:
                        min_cost = cost
                        best_rot = i
                        best_rot2 = k
                        best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return self.__get_rot(best_rot, vessel_model.rot)

    def optimize_one_rot(self, px, py, vessel_model):
        model = copy.copy(vessel_model)
        min_cost = sys.maxsize
        best_rot = 0;
        for i in range(self.rot1_min, self.rot1_max + 1, self.rot1_dot):
            # vessel model should be reset
            temp_model = copy.copy(model)
            for t in range(self.prediction_horizon):
                temp_model.simulate(i)
            #cost should is determined by xte and heading difference
            cost = (temp_model.heading - 45) ** 2 * self.heading_weight;
            cost += self.__calc_xte(px, py, temp_model)
            if cost < min_cost:
                min_cost = cost
                best_rot = i
        print(best_rot)
        return best_rot

    #optimization done by either increasing or decreasing the rot
    def optimize_simple(self, px, py, vessel_model):
        model = copy.copy(vessel_model)
        min_cost = sys.maxsize
        best_rot = 0;
        for i in range(0, 3, 1):
            for k in range(0, 3, 1):
                    for l in np.arange(self.rot_tmin, self.rot_tmax, self.rot_tdot):
                        # vessel model should be reset
                        temp_model = copy.copy(model)
                        for t in range(self.prediction_horizon):
                            if t < self.prediction_horizon * l:
                                temp_model.simulate(self.__get_rot(i, temp_model.rot))
                            else:
                                temp_model.simulate(self.__get_rot(k, temp_model.rot))
                        xte, closest_index = self.calc_xte_improved(px, py, temp_model.x, temp_model.y)
                        #xte_test = self.__calc_xte(px, py, vessel_model)
                        # print("xte: {}\n closest index: {}".format(xte, closest_index))
                        #print("test: {}".format(xte_test))
                        # print("heading error: {}".format(self.angular_diff(temp_model.heading, self.get_heading_curve(px, py, closest_index))))
                        cost = (self.angular_diff(temp_model.heading, self.get_heading_curve(px, py, closest_index)) ** 2) * self.heading_weight
                        # print("cost1: {}".format(cost))
                        cost += xte
                        # print("cost2: {}".format(cost))
                        if cost < min_cost:
                            min_cost = cost
                            best_rot = i
                            best_rot2 = k
                            best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return self.__get_rot(best_rot, vessel_model.rot)

    # optimization done by either increasing or decreasing the rot using a NN model
    def optimize_simple_MLP(self, px, py, vessel_model):
        model = copy.copy(vessel_model)
        min_cost = sys.maxsize
        best_rot = 0;
        for i in range(0, 3, 1):
            for k in range(0, 3, 1):
                for l in np.arange(self.rot_tmin, self.rot_tmax, self.rot_tdot):
                    # vessel model should be reset
                    coordx = model.x
                    coordy = model.y
                    predicted_rot = model.rot
                    predicted_heading = model.heading
                    for t in range(self.prediction_horizon):
                        if t < self.prediction_horizon * l:
                            prediction_data = np.reshape(np.array(
                                [predicted_rot, predicted_heading,
                                 self.__get_rot(i, predicted_rot)]), [1, 3])
                            prediction_data_scaled = self.scaler.transform(prediction_data)
                            prediction = self.MLP_model.predict(prediction_data_scaled)
                        else:
                            prediction_data = np.reshape(np.array(
                                [predicted_rot, predicted_heading,
                                 self.__get_rot(k, predicted_rot)]), [1, 3])
                            prediction_data_scaled = self.scaler.transform(prediction_data)
                            prediction = self.MLP_model.predict(prediction_data_scaled)
                        predicted_x = prediction[:, 0]
                        predicted_y = prediction[:, 1]
                        predicted_heading = prediction[:, 2]
                        predicted_rot = prediction[:, 3]
                        coordx += predicted_x
                        coordy += predicted_y
                    xte, closest_index = self.calc_xte_improved(px, py, coordx, coordy)
                    cost = self.angular_diff(predicted_heading, self.get_heading_curve(px, py,
                                                                                       closest_index)) ** 2 * self.heading_weight
                    cost += xte
                    if cost < min_cost:
                        min_cost = cost
                        best_rot = i
                        best_rot2 = k
                        best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return self.__get_rot(best_rot, vessel_model.rot)

    # optimization done by either increasing or decreasing the rot using
    #  a NN model for the rotdot param
    def optimize_simple_MLP_rotdot(self, px, py, vessel_model):
        model = copy.copy(vessel_model)
        min_cost = sys.maxsize
        best_rot = 0;
        for i in range(0, 3, 1):
            for k in range(0, 3, 1):
                for l in np.arange(self.rot_tmin, self.rot_tmax, self.rot_tdot):
                    # vessel model should be reset
                    temp_model = copy.copy(model)
                    for t in range(self.prediction_horizon):
                        if t < self.prediction_horizon * l:
                            prediction_data = np.reshape(np.array(
                                [temp_model.rot, self.__get_rot(i, temp_model.rot)]), [1, 2])
                            prediction_data_scaled = self.scaler.transform(prediction_data)
                            prediction = self.MLP_model.predict(prediction_data_scaled)
                        else:
                            prediction_data = np.reshape(np.array(
                                [temp_model.rot,
                                 self.__get_rot(k, temp_model.rot)]), [1, 2])
                            prediction_data_scaled = self.scaler.transform(prediction_data)
                            prediction = self.MLP_model.predict(prediction_data_scaled)
                        predicted_rotdot = prediction[:, 0]
                        temp_model.simulate_var_rotdot(predicted_rotdot)
                    xte, closest_index = self.calc_xte_improved(px, py, temp_model.x, temp_model.y)
                    cost = self.angular_diff(temp_model.heading, self.get_heading_curve(px, py,
                                                                                        closest_index)) ** 2 * self.heading_weight
                    cost += xte
                    if cost < min_cost:
                        min_cost = cost
                        best_rot = i
                        best_rot2 = k
                        best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return self.__get_rot(best_rot, vessel_model.rot)



    # naive version of calculating xte
    def __calc_xte(self, px, py, vessel_model):
        closest_index = -1;
        # calculate xte
        xte_min = sys.maxsize
        for k in range(len(px)):
            xte = (px[k] - vessel_model.x) ** 2 + (py[k] - vessel_model.y) ** 2
            if xte < xte_min:
                xte_min = xte
                closest_index = k
        return xte_min, closest_index

    def calc_xte_improved(self, px, py, vesselx, vessely):
        # split the path in checkpoints first and determine closest checkpoint
        step_size = 100
        _closest_index = 0
        _second_closest_index = len(px)
        xte_min = 0
        while step_size != 0:
            closest_index = _closest_index
            second_closest_index = _second_closest_index
            xte_min = sys.maxsize;
            xte_second_closest = sys.maxsize;
            if second_closest_index < closest_index:
                # swap them places to allow the for loop to evaluate
                temp = second_closest_index
                second_closest_index = closest_index
                closest_index = temp
            for i in range(closest_index, second_closest_index, step_size):
                xte = (px[i] - vesselx) ** 2 + (py[i] - vessely) ** 2
                if xte < xte_second_closest:
                    if xte < xte_min:
                        xte_second_closest = xte_min
                        _second_closest_index = _closest_index
                        xte_min = xte
                        _closest_index = i
                    else:
                        xte_second_closest = xte
                        _second_closest_index = i
            step_size = math.floor(step_size / 10)
        return xte_min, _closest_index


    def get_heading_curve(self, px, py, k):
        if py[k+1] > py[k]:
            try:
                angle = math.atan((px[k+1] - px[k]) / (py[k+1] - py[k])) / math.pi * 180
            except ZeroDivisionError:
                angle = 90
        else:
            try:
                angle = math.atan((py[k+1] - py[k]) / (px[k+1] - px[k])) / math.pi * 180
                # the angle abs value is greater than 90 degrees
                if px[k+1] > px[k]:
                    angle = math.fabs(angle) + 90
                elif px[k+1] < px[k]:
                    # the angle is negative
                    angle += 90
                    angle *= -1
            except ZeroDivisionError:
                angle = 180
        return angle

    # Calculate a rot based on index i
    def __get_rot(self, i, rot):
        if i == 0:
            return rot + 10
        elif i == 1:
            return rot
        elif i == 2:
            return rot - 10

    def __normalize_course(self, angle):
        if angle >= 0:
            return angle % 360.0
        else:
            result = 360.0 - ((-angle) % 360.0)
            if result == 360.0:
                return 0.0
            else:
                return result

    # gives the result between 2 angles by normalizing them first
    def angular_diff(self, a1, a2):
        diff = a1 - a2
        diff = self.__normalize_course(diff + 180.0) - 180.0
        return math.fabs(diff)

