import numpy as np
from pulp import *


class RelocationOptimizer:

    def __init__(self, vehicle_number: int, maximum_relocation: int, incoming_demand: list, outgoing_demand: list,
                 origin_destination_matrix: np.array, optimization_horizon: int, look_ahead_horizon: int,
                 verbose: int = 1):

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

        if optimization_horizon < 0:
            raise ValueError('The optimization horizon can\'t be negative')

        if look_ahead_horizon < 0:
            raise ValueError('The look ahead horizon can\'t be negative')

        if self.__time_frames_number < optimization_horizon:
            raise ValueError('Not enough time frame data are provided to optimize over the optimization horizon')

        if optimization_horizon < look_ahead_horizon * 2 - 1:
            raise ValueError('The optimization horizon should be at least 2 x look ahead horizon - 1')

        self.__look_ahead_horizon = look_ahead_horizon
        self.__optimization_horizon = optimization_horizon

        if verbose != 0 and verbose != 1:
            raise ValueError('Verbose can be either 0 (deactivated) or 1 (activated)')

        self.__verbose = verbose

        self.__initial_optimization_method = None
        self.__neighbourhood_search = None
        self.__next_state_simulation = None
        self.__max_seconds_same_incumbent = None
        self.__mc_simulations = None
        self.__procedure_set = False

    def set_optimization_procedure(self, initial_optimization_method: str = 'LP+rounding',
                                   neighbourhood_search: bool = True, next_state_simulation: str = 'Monte Carlo',
                                   max_seconds_same_incumbent: int = 60 * 60 * 3, mc_simulations: int = 50):

        # first_optimization_method: 'LP+rounding', 'MILP', 'LP+MILP'
        # next_state_simulation: 'Monte_Carlo', 'best_case'

        if initial_optimization_method not in ['LP+rounding', 'MILP', 'LP+MILP']:
            raise ValueError('The initial optimization method can either be \'LP+rounding\', \'MILP\' or \'LP+MILP\'')

        self.__initial_optimization_method = initial_optimization_method
        self.__neighbourhood_search = neighbourhood_search

        if next_state_simulation not in ['Monte_Carlo', 'best_case']:
            raise ValueError('The next state simulation method can either be \'Monte_Carlo\' or \'best_case\'')

        self.__next_state_simulation = next_state_simulation

        self.__max_seconds_same_incumbent = max_seconds_same_incumbent
        self.__mc_simulations = mc_simulations
        self.__procedure_set = True

    def optimize(self):

        if not self.__procedure_set:
            raise Exception('The optimization procedure has still to be defined')

        self.__initial_optimization()

    def __initial_optimization(self):

        if self.__initial_optimization_method == 'LP+rounding':
            self.__lp_problem()

        elif self.__initial_optimization_method == 'MILP':

        elif self.__initial_optimization_method == 'LP+MILP':

    def __lp_problem(self):
        model = LpProblem(sense=LpMaximize)

        # declaring variables
        cat = 'Continuous'  # 'Integer' or 'Continuous'
        s = [[LpVariable(name='s_' + str(i) + '_' + str(t), lowBound=0, cat=cat) for i in range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        EI = [
            [LpVariable(name='EI_' + str(i) + '_' + str(t), lowBound=0, upBound=self.__incoming_demand[t][i], cat=cat) for i in range(self.__zones_number)]
            for t in range(self.__look_ahead_horizon)]
        EO = [
            [LpVariable(name='EO_' + str(i) + '_' + str(t), lowBound=0, upBound=N_out[t][i], cat=cat) for i in range(self.__zones_number)]
            for t in range(self.__look_ahead_horizon)]
        r = [[LpVariable(name='r_' + str(i) + '_' + str(t), lowBound=-R_max, upBound=R_max, cat=cat) for i in range(self.__zones_number)]
             for
             t in range(self.__look_ahead_horizon)]
        w = [[LpVariable(name='w_' + str(i) + '_' + str(t), lowBound=0, cat=cat) for i in range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]

        for i in range(self.__zones_number):
            s[0][i].lowBound = 0
            s[0][i].upBound = 0
        i = 0
        for _ in range(N):
            s[0][i].lowBound += 1
            s[0][i].upBound += 1
            if i == self.__zones_number - 1:
                i = 0
            else:
                i += 1

        # objective function
        model += lpSum(EO)  # g * lpSum(EO) - c * lpSum(w)

        # inequality constraints
        # for i in range(S):
        #     for t in range(T):
        #         model += (d[t][i] >= N_out[t][i] - EO[t][i], 'unsatisfied_demand_nonnegativity_' + str(i) + '_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (lpSum(w[t]) <= R_max, 'max_relocation_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (w[t][i] >= r[t][i], 'max_relocation_nonnegativity_' + str(i) + '_' + str(t))

        # for i in range(S):
        #     for t in range(T):
        #         model += (EO[t][i] <= s[t][i], 'effective_outgoing_bound_' + str(i) + '_' + str(t))
        # for i in range(S):
        #     for t in range(T - 1):
        #         model += (EO[t][i] <= 0.5 * (s[t][i] + s[t + 1][i]), 'effective_outgoing_bound_' + str(i) + '_' + str(t))
        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (
                EO[t][i] <= s[t][i] + 0.5 * (EI[t][i] + r[t][i]), 'effective_outgoing_bound_' + str(i) + '_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (-r[t][i] <= s[t][i], 'relocation_bound_' + str(i) + '_' + str(t))

        # for t in range(T):
        #     model += (lpSum(EO[t]) <= N, 'effective_outgoing_bound_' + str(t))

        # equality constraints
        for i in range(self.__zones_number):
            for t in range(1, self.__look_ahead_horizon):
                model += (s[t][i] == s[t - 1][i] + EI[t - 1][i] - EO[t - 1][i] + r[t - 1][i],
                          'state_transition_' + str(i) + '_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (lpSum(r[t]) == 0, 'tot_relocation_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (lpSum(s[t]) == N, 'mass_conservation_' + str(t))

        # for t in range(T):
        #     model += (lpSum(EI[t]) - lpSum(EO[t]) == 0, 'effective_incoming_outgoing_matching_' + str(t))
        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (EI[t][i] == lpSum(EO[t][j] * p[t][j, i] for j in range(self.__zones_number)),
                          'effective_incoming_outgoing_matching_' + str(i) + '_' + str(t))

        # for i in range(S):
        #     for t in range(T):
        #         model += (
        #             EO[t][i] + d[t][i] == N_out[t][i],
        #             'effective_outgoing_unsatisfied_demand_matching_' + str(i) + '_' + str(t))

        status = model.solve()