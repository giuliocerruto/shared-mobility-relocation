from StateSpace import *
from BacktrackingSolver import BacktrackingSolver
import time
import numpy as np

if __name__ == '__main__':

    N = 80  # total number of vehicles
    max_relocation = 15  # maximum number of vehicles which can be relocated in a time frame

    time_frames = 3  # number of time frames
    zones = 4  # number of zones
    # N_in = [np.array([0, 0, 0, 0]),
    #         np.array([8, 2, 5, 5]),
    #         np.array([8, 2, 5, 5])]
    # N_out = [np.array([0, 0, 0, 0]),
    #          np.array([4, 7, 4, 7]),
    #          np.array([4, 7, 4, 7])]
    N_in = [np.array([0, 0, 0, 0]),
            np.array([7, 5, 1, 3]),
            np.array([12, 50, 15, 3])]  # number of vehicles being left in each zone from one time frame to the next one
    N_out = [np.array([0, 0, 0, 0]),
             np.array([1, 13, 1, 1]),
             np.array([19, 25, 42, 10])]  # number of vehicles leaving each zone from one time frame to the next one
    S0 = tuple([int(N / zones)] * zones)  # initial state of the system

    ############################### STATE SPACE ###############################

    start_time = time.time()

    ss = StateSpace(n=N, max_relocation=max_relocation)

    time1 = time.time()
    print("--- %s minutes ---" % ((time1 - start_time) / 60))

    ############################### BACKTRACKING ALGORITHM ###############################

    bt = BacktrackingSolver(state_space=ss, max_relocation=max_relocation, time_frames=time_frames,
                            n_in=N_in, n_out=N_out, initial_state=S0)
    bt.compute_value_function()

    time2 = time.time()
    print("--- %s minutes ---" % ((time2 - time1) / 60))

    ############################### PRINTING RESULTS ###############################

    bt.print_solution()
