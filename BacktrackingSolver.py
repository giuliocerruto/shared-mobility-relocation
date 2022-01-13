from StateSpace import *
import numpy as np
from main import positive_part


class BacktrackingSolver:

    def __init__(self, state_space: StateSpace, max_relocation: int, time_frames: int,
                 n_in: np.array, n_out: np.array, initial_state: tuple):
        self.stateSpace = state_space
        self.maxRelocation = max_relocation
        self.timeFrames = time_frames
        self.N_in = n_in
        self.N_out = n_out
        self.initialState = self.stateSpace.get_state(initial_state)

    def compute_value_function(self):

        print('Computing value function...')

        # computing state values in the last time frame
        for state in self.stateSpace:
            state.set_value(self.timeFrames - 1, -sum(positive_part(self.N_out[self.timeFrames - 1] - state.x)))
            state.set_next(self.timeFrames - 1, None)

        # computing state values in the other time frames
        for tf in range(self.timeFrames - 2, 0, -1):  # TODO initial state can be chosen to be optimal setting 0 to -1
            for state in self.stateSpace:
                int_state = self.stateSpace.get_state(tuple(positive_part(state.x + self.N_in[tf] - self.N_out[tf])))

                max_state = None
                max_val = -9999

                if int_state:
                    for next_state in int_state.reachableBy:
                        if next_state.get_value(tf + 1) and max_val < next_state.get_value(tf + 1):
                            max_val = next_state.get_value(tf + 1)
                            max_state = next_state

                    state.set_value(tf, -sum(positive_part(self.N_out[tf] - state.x)) + max_val)
                    state.set_next(tf, max_state)

        # computing value of initial state
        max_state = None
        max_val = -9999
        for next_state in self.initialState.reachableBy:
            if next_state.get_value(1) and max_val < next_state.get_value(1):
                max_val = next_state.get_value(1)
                max_state = next_state

        self.initialState.set_value(0, max_val)
        self.initialState.set_next(0, max_state)

    def print_solution(self):
        next_state = self.initialState
        for tf in range(self.timeFrames):
            print('tf ' + str(tf) + ') ', next_state, ' value: ', next_state.get_value(tf))
            next_state = next_state.get_next(tf)
