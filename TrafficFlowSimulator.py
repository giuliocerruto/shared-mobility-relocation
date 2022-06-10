import random
import numpy as np
from time import time

random.seed(27)


class TrafficFlowSimulator:

    def __init__(self,
                 origin_destination_matrix: np.array):

        # if len(incoming_demand) != len(outgoing_demand):
        #     raise ValueError(
        #         'Incoming and outgoing demand list dimensions don\'t match (the number of time frames is different)')
        #
        # if len(incoming_demand[0]) != len(outgoing_demand[0]):
        #     raise ValueError(
        #         'Incoming and outgoing demand list dimensions don\'t match (the number of zones is different)')
        #
        # self.__time_frames_number = len(incoming_demand)
        # self.__zones_number = len(incoming_demand[0])
        #
        # if not all(len(l) == self.__zones_number for l in incoming_demand):
        #     raise ValueError('Not all lists in the incoming demand list have same length')
        #
        # if not all(len(l) == self.__zones_number for l in outgoing_demand):
        #     raise ValueError('Not all lists in the outgoing demand list have same length')
        #
        # self.__incoming_demand = incoming_demand
        # self.__outgoing_demand = outgoing_demand
        #
        # if origin_destination_matrix.shape[0] != self.__time_frames_number:
        #     raise ValueError(
        #         'The origin-destination matrix shape doesn\'t match that of the demand lists (the number of time frames is different)')
        #
        # if origin_destination_matrix.shape[1] != origin_destination_matrix.shape[2] or \
        #         origin_destination_matrix.shape[1] != self.__zones_number:
        #     raise ValueError(
        #         'The origin-destination matrix shape doesn\'t match that of the demand lists (the number of zones is different or the matrices are not sqaure)')

        self.__origin_destination_matrix = origin_destination_matrix

        self.__time_frames_number, self.__zones_number, _ = self.__origin_destination_matrix.shape

        # computing incoming and outgoing demand arrays
        self.__outgoing_demand = np.zeros((self.__time_frames_number, self.__zones_number), dtype=np.uint8)
        self.__incoming_demand = np.zeros((self.__time_frames_number, self.__zones_number), dtype=np.uint8)
        for t in range(self.__time_frames_number):
            self.__outgoing_demand[t] = self.__origin_destination_matrix[t].sum(axis=1)
            self.__incoming_demand[t] = self.__origin_destination_matrix[t].sum(axis=0)

        self.__time_frame_flows = {tf: list() for tf in range(self.__time_frames_number)}

        self.__simulate_system()

        return

    def __simulate_system(self):

        print('Generating traffic flows...')

        start_time = time()

        for t in range(self.__time_frames_number):
            self.__simulate_time_frame(time_frame=t)

        print(f'Traffic flows generated ({time()-start_time:.0f} seconds)')

        return

    def __simulate_time_frame(self,
                              time_frame: int):

        start_zones = []
        [start_zones.extend([i] * self.__outgoing_demand[time_frame][i]) for i in range(self.__zones_number)]

        random.shuffle(start_zones)

        od_matrix = np.copy(self.__origin_destination_matrix[time_frame])

        trips = [[] for i in range(self.__zones_number)]
        for i in range(self.__zones_number):
            for j in range(self.__zones_number):
                if od_matrix[i, j] > 0:
                    trips[i].extend([j] * od_matrix[i, j])

            random.shuffle(trips[i])

        for start_zone in start_zones:

            arrival_zone = trips[start_zone].pop(0)

            self.__time_frame_flows[time_frame].append((start_zone, arrival_zone))

        return

    def get_flow_by_time_frame(self, time_frame: int):
        return self.__time_frame_flows[time_frame]
