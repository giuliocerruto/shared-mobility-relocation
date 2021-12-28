import numpy as np


def reachable_state(state, previous_set, max_rel, _in, out):  # finding if state is reachable from the previous state set

    for s in previous_set:
        if sum(positive_part(np.array(s) - out + _in - np.array(state))) <= max_rel:
            return True

    return False


def generate_states(S0, time_frames, max_relocation):  # generating state sets for each time frame

    states = [{S0: -1}] + [{}] * (time_frames - 1)

    for tf in range(1, time_frames):
        for i in range(N + 1):
            for j in range(N + 1 - i):
                for k in range(N + 1 - i - j):
                    if reachable_state((i, j, k, N - i - j - k), states[tf - 1].keys(), max_relocation, N_in[tf], N_out[tf]):
                        states[tf][(i, j, k, N - i - j - k)] = -1
        print('finito', tf)
    return states


def positive_part(ar):
    return [max(i, 0) for i in ar]


if __name__ == '__main__':

    N = 80  # total number of vehicles
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
    # demand = [np.array()] * (time_frames - 1)  # demand to satisfy for each zone in each time frame
    max_relocation = 15  # maximum number of vehicles which can be relocated in a time frame
    S0 = tuple([int(N / zones)] * zones)  # initial state of the system

    states = generate_states(S0, time_frames,
                             max_relocation)  # list of dicts for each time frame: key = state, value = state value

    # values of last time frame
    for state in states[time_frames - 1].keys():
        states[time_frames - 1][state] = [-sum(positive_part(N_out[time_frames - 1]-np.array(state))), None]

    for tf in range(time_frames - 2, -1, -1):
        for state in states[tf].keys():
            max_state = None
            max_val = -9999

            int_state = positive_part(np.array(state) + N_in[tf] - N_out[tf])

            for next_state in states[tf + 1].keys():
                if reachable_state(next_state, [state], max_relocation, N_in[tf], N_out[tf]):
                    if max_val < states[tf + 1][next_state][0]:
                        max_val = states[tf + 1][next_state][0]
                        max_state = next_state

            states[tf][state] = [-sum(positive_part(N_out[tf]-np.array(state))) + max_val, max_state]

    next_state = S0
    for tf in range(time_frames):
        print('tf ' + str(tf) + ') ', next_state, ' value: ', states[tf][next_state][0])
        next_state = states[tf][next_state][1]
