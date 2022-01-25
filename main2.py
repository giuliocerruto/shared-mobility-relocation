from StateSpace import *
from BacktrackingSolver import BacktrackingSolver
import time
import numpy as np

if __name__ == '__main__':

    N = 40  # total number of vehicles
    max_relocation = 10  # maximum number of vehicles which can be relocated in a time frame
    filename = 'state_space_' + str(N) + '_' + str(max_relocation)
    mode = 'save'  # or 'load'

    zones = 4  # number of zones

    if N == 20:
        N_in = [np.array([0, 0, 0, 0]),
                np.array([8, 2, 5, 5]),
                np.array([8, 2, 5, 5]),
                np.array([4, 6, 8, 2]),
                np.array([2, 2, 11, 5]),
                np.array([5, 7, 4, 4]),
                np.array([8, 2, 5, 5]),
                np.array([8, 6, 2, 4]),
                np.array([4, 2, 4, 6]),
                np.array([1, 1, 5, 5])]
        N_out = [np.array([0, 0, 0, 0]),
                 np.array([4, 7, 4, 7]),
                 np.array([4, 7, 4, 7]),
                 np.array([6, 8, 8, 2]),
                 np.array([4, 6, 6, 10]),
                 np.array([3, 4, 9, 9]),
                 np.array([8, 2, 5, 5]),
                 np.array([8, 2, 5, 5]),
                 np.array([4, 4, 4, 4]),
                 np.array([4, 4, 2, 2])]
    elif N == 40:
        N_in = [np.array([0, 0, 0, 0]),
                np.array([16, 4, 10, 10]),
                np.array([16, 4, 10, 10]),
                np.array([8, 12, 16, 4]),
                np.array([4, 4, 22, 10]),
                np.array([10, 14, 8, 8]),
                np.array([16, 4, 10, 10]),
                np.array([16, 12, 4, 8]),
                np.array([8, 4, 8, 12]),
                np.array([2, 2, 10, 10])]
        N_out = [np.array([0, 0, 0, 0]),
                 np.array([8, 14, 8, 14]),
                 np.array([8, 14, 8, 14]),
                 np.array([12, 16, 16, 4]),
                 np.array([8, 12, 12, 20]),
                 np.array([6, 8, 18, 18]),
                 np.array([16, 4, 10, 10]),
                 np.array([16, 4, 10, 10]),
                 np.array([8, 8, 8, 8]),
                 np.array([8, 8, 4, 4])]
    elif N == 80:
        #filename = 'state_space_80_8'
        N_in = [np.array([0, 0, 0, 0]),
                np.array([7, 5, 1, 3]),
                np.array([12, 50, 15, 3]),
                np.array([11, 40, 20, 5]),
                np.array([12, 50, 15, 3]),
                np.array([5, 12, 13, 10]),
                np.array([40, 7, 8, 25]),
                np.array([32, 5, 7, 36]),
                np.array([9, 14, 7, 15]),
                np.array([5, 3, 4, 6])]  # number of vehicles being left in each zone from one time frame to the next one
        N_out = [np.array([0, 0, 0, 0]),
                 np.array([1, 13, 1, 1]),
                 np.array([19, 25, 42, 10]),
                 np.array([8, 42, 12, 14]),
                 np.array([10, 42, 25, 19]),
                 np.array([4, 17, 14, 5]),
                 np.array([5, 40, 45, 5]),
                 np.array([4, 45, 42, 6]),
                 np.array([12, 13, 15, 5]),
                 np.array([6, 4, 3, 5])]  # number of vehicles leaving each zone from one time frame to the next one

    S0 = tuple([int(N / zones)] * zones)  # initial state of the system

    ############################### STATE SPACE ###############################

    start_time = time.time()

    if mode == 'save':
        ss = StateSpace(n=N, max_relocation=max_relocation)

    if mode == 'load':
        ss = StateSpace.load(filename)

    time1 = time.time()

    if mode == 'save':
        ss.save(filename)

    print("--- %s minutes ---" % ((time1 - start_time) / 60))

    ############################### BACKTRACKING ALGORITHM ###############################

    bt = BacktrackingSolver(state_space=ss, n_in=N_in, n_out=N_out, initial_state=S0)
    bt.compute_value_function()

    time2 = time.time()
    print("--- %s minutes ---" % ((time2 - time1) / 60))

    ############################### PRINTING RESULTS ###############################

    bt.print_solution('results.txt', time2-time1)
