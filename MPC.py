import sys
import numpy as np
import copy
class MPC:
    #tuning params
    rot1_max = 20
    rot1_min = -20
    rot1_dot = 2
    rot_tmax = 0.65
    rot_tmin = 0.35
    rot_tdot = 0.1
    rot2_max = 20
    rot2_min = -20
    rot2_dot = 2
    prediction_horizon = 240
    heading_weight = 25;

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
                    cost = (temp_model.heading - 45) ** 2 * self.heading_weight;
                    cost += self.__calc_xte(px, py, temp_model)
                    if cost < min_cost:
                        min_cost = cost
                        best_rot = i
                        best_rot2 = k
                        best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return best_rot

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
                    cost = (temp_model.heading - 45) ** 2 * self.heading_weight;
                    cost += self.__calc_xte(px, py, temp_model)
                    if cost < min_cost:
                        min_cost = cost
                        best_rot = i
                        best_rot2 = k
                        best_trans = l
        print(best_rot)
        print(best_rot2)
        print(best_trans)
        return self.__get_rot(best_rot, vessel_model.rot)

    def __calc_xte(self, px, py, vessel_model):
        # calculate xte
        xte_min = sys.maxsize
        for k in range(len(px)):
            xte = (px[k] - vessel_model.x) ** 2 + (py[k] - vessel_model.y) ** 2
            if xte < xte_min:
                xte_min = xte
        return xte_min

    # Calculate a rot based on index i
    def __get_rot(self, i, rot):
        if i == 0:
            return rot + 1
        elif i == 1:
            return rot
        elif i == 2:
            return rot - 1


