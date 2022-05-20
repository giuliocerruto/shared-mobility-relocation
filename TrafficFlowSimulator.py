import random
import numpy as np
from time import time


class TrafficFlowSimulator:

    def __init__(self, vehicle_number: int, maximum_relocation: int, incoming_demand: list, outgoing_demand: list,
                 origin_destination_matrix: np.array):
        if vehicle_number < 0:
            raise ValueError('The number of vehicles can\'t be negative')

        self.__vehicle_number = vehicle_number

        if maximum_relocation < 0:
            raise ValueError('The maximum number of vehicles to be relocated can\'t be negative')

        self.__maximum_relocation = maximum_relocation

        if len(incoming_demand) != len(outgoing_demand):
            raise ValueError(
                'Incoming and outgoing demand list dimensions don\'t match (the number of time frames is different)')

        if len(incoming_demand[0]) != len(outgoing_demand[0]):
            raise ValueError(
                'Incoming and outgoing demand list dimensions don\'t match (the number of zones is different)')

        self.__time_frames_number = len(incoming_demand)
        self.__zones_number = len(incoming_demand[0])

        if not all(len(l) == self.__zones_number for l in incoming_demand):
            raise ValueError('Not all lists in the incoming demand list have same length')

        if not all(len(l) == self.__zones_number for l in outgoing_demand):
            raise ValueError('Not all lists in the outgoing demand list have same length')

        self.__incoming_demand = incoming_demand
        self.__outgoing_demand = outgoing_demand

        if origin_destination_matrix.shape[0] != self.__time_frames_number:
            raise ValueError(
                'The origin-destination matrix shape doesn\'t match that of the demand lists (the number of time frames is different)')

        if origin_destination_matrix.shape[1] != origin_destination_matrix.shape[2] or origin_destination_matrix.shape[
            1] != self.__zones_number:
            raise ValueError(
                'The origin-destination matrix shape doesn\'t match that of the demand lists (the number of zones is different or the matrices are not sqaure)')

        self.__origin_destination_matrix = origin_destination_matrix

        self.__time_frame_flows = {tf: list() for tf in range(self.__time_frames_number)}

        self.__simulate_system()

        return

    def __simulate_system(self):

        print('Generating traffic flows...')

        start_time = time()

        for t in range(self.__time_frames_number):
            self.__simulate_time_frame(t)

        print(f'Traffic flows generated ({time()-start_time:.0f} seconds)')

        return

    def __simulate_time_frame(self, time_frame: int):

        out_zones = {i for i in range(self.__zones_number) if
                     self.__outgoing_demand[time_frame][i] > 0}  # set of zones from which vehicles can leave
        in_zones = {i for i in range(self.__zones_number) if
                    self.__incoming_demand[time_frame][i] > 0}  # set of zones to which vehicles can get

        _out = self.__outgoing_demand[time_frame].copy()
        _in = self.__incoming_demand[time_frame].copy()

        counter = 0

        while len(out_zones) > 0 and len(in_zones) > 0 and counter < 50:

            start_zone = random.sample(out_zones, k=1)[0]
            arrival_zone = random.sample([j for j in range(self.__zones_number) if
                                          self.__origin_destination_matrix[time_frame][start_zone, j] > 0], k=1)[0]

            if arrival_zone in in_zones:

                self.__time_frame_flows[time_frame].append((start_zone, arrival_zone))

                _out[start_zone] -= 1
                if _out[start_zone] == 0:
                    out_zones.discard(start_zone)
                _in[arrival_zone] -= 1
                if _in[arrival_zone] == 0:
                    in_zones.discard(arrival_zone)

                counter = 0

            else:
                counter += 1

        return

    def get_flow_by_time_frame(self, time_frame: int):
        return self.__time_frame_flows[time_frame]
