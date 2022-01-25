from StateSpace import *
import numpy as np
from main import positive_part


class BacktrackingSolver:

    def __init__(self, state_space: StateSpace,
                 n_in: np.array, n_out: np.array, initial_state: tuple):
        self.__stateSpace = state_space
        self.__zones = self.__stateSpace.get_zone_n()
        self.__timeFrames = len(n_in)
        self.N_in = n_in
        self.N_out = n_out
        self.initialState = self.__stateSpace.get_state(initial_state)

    # def __correct_state(self, state):
    #     excess = sum(state) - self.stateSpace.N
    #     z = len(state)
    #     i = 0
    #
    #     while excess > 0:
    #         if state[i] > 0:
    #             state[i] -= 1
    #             excess -= 1
    #
    #         if i != z-1:
    #             i += 1
    #         else:
    #             i = 0
    #
    #     while excess < 0:
    #         state[i] += 1
    #         excess += 1
    #         if i != z-1:
    #             i += 1
    #         else:
    #             i = 0
    #
    #     return state

    # def __correct_state(self, state, excess_demand):
    #     excess = excess_demand
    #     indexes = list(range(self.__zones))
    #
    #     for i in range(self.__zones):
    #         if state[i] < 0:
    #             excess += state[i]
    #             state[i] = 0
    #             indexes.remove(i)
    #         elif state[i] == 0:
    #             indexes.remove(i)
    #
    #     il = len(indexes)
    #     i = 0
    #
    #     while excess > 0:
    #         state[indexes[i]] += 1
    #         excess -= 1
    #
    #         if i != il - 1:
    #             i += 1
    #         else:
    #             i = 0
    #
    #     while excess < 0:
    #         if state[indexes[i]] > 0:
    #             state[indexes[i]] -= 1
    #             excess += 1
    #
    #         if i != il - 1:
    #             i += 1
    #         else:
    #             i = 0
    #
    #     return tuple(state)

    def compute_value_function(self):

        print('Computing value function...')

        # computing state values in the last time frame
        for state in self.__stateSpace:
            state.set_value(self.__timeFrames - 1, sum(positive_part(self.N_out[self.__timeFrames - 1] - state.x)))
            state.set_next(self.__timeFrames - 1, None)

        # computing state values in the other time frames
        for tf in range(self.__timeFrames - 2, 0, -1):
            print(tf)
            excess_demand = sum(self.N_out[tf]) - sum(self.N_in[tf])

            for state in self.__stateSpace:
                state.compute_after_demand(self.N_out[tf], self.N_in[tf], excess_demand)

            for state in self.__stateSpace:

                min_state = None
                min_val = 9999

                for next_state in state.reachableBy:
                    next_state_p = self.__stateSpace.get_state(next_state.get_after_demand())

                    if next_state_p:
                        if min_val > next_state_p.get_value(tf + 1):
                            min_val = next_state_p.get_value(tf + 1)
                            min_state = next_state_p
                            min_rel = sum(positive_part(next_state_p - state))
                        elif min_val == next_state_p.get_value(
                                tf + 1):  # checking if this state requires less relocation
                            rel = sum(positive_part(next_state_p - state))  # computing current needed relocation
                            if rel < min_rel:
                                min_state = next_state_p
                                min_rel = rel

                state.set_value(tf, sum(positive_part(self.N_out[tf] - state.x)) + min_val)
                state.set_next(tf, min_state)

                # min_state = None
                # min_val = 9999
                #
                # for next_state in state.reachableBy:
                #     next_state_p = self.__stateSpace.get_state(
                #         self.__correct_state(next_state.x + self.N_in[tf] - self.N_out[tf], excess_demand))
                #
                #     if next_state_p:
                #         if min_val > next_state_p.get_value(tf + 1):
                #             min_val = next_state_p.get_value(tf + 1)
                #             min_state = next_state_p
                #             min_rel = sum(positive_part(next_state_p - state))
                #         elif min_val == next_state_p.get_value(tf + 1):  # checking if this state requires less relocation
                #             rel = sum(positive_part(next_state_p - state))  # computing current needed relocation
                #             if rel < min_rel:
                #                 min_state = next_state_p
                #                 min_rel = rel
                #
                # state.set_value(tf, sum(positive_part(self.N_out[tf] - state.x)) + min_val)
                # state.set_next(tf, min_state)

                # int_state = self.stateSpace.get_state(tuple(positive_part(state.x + self.N_in[tf] - self.N_out[tf])))
                #
                # min_state = None
                # min_val = 9999
                #
                # if int_state:
                #     for next_state in int_state.reachableBy:
                #         if next_state.get_value(tf + 1):
                #             if min_val > next_state.get_value(tf + 1):
                #                 min_val = next_state.get_value(tf + 1)
                #                 min_state = next_state
                #                 min_rel = sum(positive_part(next_state - int_state)) # TODO si potrebbe usare anche la norma 0 ma aumenta il tempo di calcolo
                #             elif min_val == next_state.get_value(
                #                     tf + 1):  # checking if this state requires less relocation
                #                 rel = sum(positive_part(next_state - int_state))  # computing current needed relocation
                #                 if rel < min_rel:
                #                     min_state = next_state
                #                     min_rel = rel
                #
                #
                #     state.set_value(tf, sum(positive_part(self.N_out[tf] - state.x)) + min_val)
                #     state.set_next(tf, min_state)

        # computing value of initial state
        min_state = None
        min_val = 9999
        for next_state in self.initialState.reachableBy:
            if next_state.get_value(1):
                if min_val > next_state.get_value(1):
                    min_val = next_state.get_value(1)
                    min_state = next_state
                    min_rel = sum(positive_part(next_state - self.initialState))
                elif min_val == next_state.get_value(tf + 1):  # checking if this state requires less relocation
                    rel = sum(positive_part(next_state - self.initialState))  # computing current needed relocation
                    if rel < min_rel:
                        min_state = next_state
                        min_rel = rel

        self.initialState.set_value(0, min_val)
        self.initialState.set_next(0, min_state)

    def print_solution(self, filename, time):
        next_state = self.initialState

        with open(filename, 'a') as f:
            print(file=f)
            print('N=' + str(self.__stateSpace.N) + ' max_relocation=' + str(self.__stateSpace.maxRelocation), file=f)
            print('--- ' + str(time / 60) + ' minutes ---', file=f)
            for tf in range(self.__timeFrames):
                print('tf ' + str(tf) + ') ', next_state, ' value: ', next_state.get_value(tf))
                print('tf ' + str(tf) + ') ', next_state, ' value: ', next_state.get_value(tf), file=f)
                next_state = next_state.get_next(tf)
            print(file=f)

# TODO Stima dei tempi con più data point
# TODO Variare la domanda (costante e non costante nel tempo)
