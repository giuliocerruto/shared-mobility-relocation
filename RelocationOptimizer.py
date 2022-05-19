import numpy as np
from pulp import *
from mip import *
import random
import copy
from math import e
from time import time
import pandas as pd
import plotnine as p9
from TrafficFlowSimulator import TrafficFlowSimulator

random.seed(27)

TOLERANCE = 1e-5


def custom_round(value: float) -> int:
    rounded = round(value)

    if value < 0 and abs(value - rounded + 0.5) < 1e-5:
        rounded -= 1
    if abs(value - 0.5) < 1e-5:
        rounded += 1

    return int(rounded)


class RelocationOptimizer:

    def __init__(self, vehicle_number: int, maximum_relocation: int, incoming_demand: list, outgoing_demand: list,
                 origin_destination_matrix: np.array, optimization_horizon: int, look_ahead_horizon: int,
                 traffic_flow_simulator: TrafficFlowSimulator, verbose: bool = True):

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

        if self.__time_frames_number < optimization_horizon + look_ahead_horizon:
            raise ValueError('Not enough time frame data are provided to optimize over the optimization horizon')

        self.__look_ahead_horizon = look_ahead_horizon
        self.__optimization_horizon = optimization_horizon

        self.__traffic_flow_simulator = traffic_flow_simulator

        if not isinstance(verbose, bool):
            raise ValueError('Verbose can be either False (deactivated) or True (activated)')

        self.__verbose = verbose

        # variables to set the optimization procedure
        self.__initial_optimization_method = None
        self.__perform_neighbourhood_search = None
        self.__next_state_simulation = None
        self.__max_seconds_same_incumbent = None
        self.__mc_simulations = None
        self.__max_iterations = None
        self.__max_iterations_no_improvement = None
        self.__relocation_solution = list()
        self.__procedure_set = False

        # variables to compute statistics
        self.__statistics = pd.DataFrame(index=list(range(self.__optimization_horizon)) + ['overall'],
                                         columns=['demand', 'objective', 'efficiency', 'start_time', 'stop_time',
                                                  'elapsed_time'])

        for tf in range(self.__optimization_horizon):
            self.__statistics.at[tf, 'demand'] = np.sum(self.__outgoing_demand[tf])

        self.__statistics.at['overall', 'demand'] = np.sum(self.__statistics['demand'])
        self.__statistics.at['overall', 'objective'] = 0

        return

    def set_optimization_procedure(self, initial_optimization_method: str = 'LP+rounding',
                                   perform_neighbourhood_search: bool = True,
                                   next_state_simulation: str = 'Monte_Carlo',
                                   max_seconds_same_incumbent: int = 60 * 60 * 3, mc_simulations: int = 50,
                                   max_iterations: int = 100, max_iterations_no_improvement: int = 10):

        # first_optimization_method: 'LP+rounding', 'MILP', 'LP+MILP'
        # next_state_simulation: 'Monte_Carlo', 'best_case'

        if initial_optimization_method not in ['LP+rounding', 'MILP', 'LP+MILP']:
            raise ValueError('The initial optimization method can either be \'LP+rounding\', \'MILP\' or \'LP+MILP\'')

        self.__initial_optimization_method = initial_optimization_method
        self.__perform_neighbourhood_search = perform_neighbourhood_search

        if next_state_simulation not in ['Monte_Carlo', 'best_case']:
            raise ValueError('The next state simulation method can either be \'Monte_Carlo\' or \'best_case\'')

        self.__next_state_simulation = next_state_simulation

        self.__max_seconds_same_incumbent = max_seconds_same_incumbent

        if mc_simulations <= 0:
            raise ValueError('The number of Monte Carlo Simulations can\'t be negative or zero')

        self.__mc_simulations = mc_simulations

        if max_iterations <= 0:
            raise ValueError('The maximum number of iterations in the neighbourhood search can\'t be negative or zero')

        self.__max_iterations = max_iterations

        if max_iterations_no_improvement <= 0:
            raise ValueError('The maximum number of iterations in the neighbourhood search can\'t be negative or zero')

        self.__max_iterations_no_improvement = max_iterations_no_improvement

        self.__procedure_set = True

        return

    def optimize(self):

        if not self.__procedure_set:
            raise Exception('The optimization procedure has still to be defined')

        if self.__verbose:
            print('Starting optimization procedure...')

        s0 = [0] * self.__zones_number

        i = 0
        for _ in range(self.__vehicle_number):
            s0[i] += 1

            if i == self.__zones_number - 1:
                i = 0
            else:
                i += 1

        current_system_state = s0

        self.__statistics.at['overall', 'start_time'] = time()

        for t in range(self.__optimization_horizon):

            if self.__verbose:
                print(f'Computing optimal relocation for time-frame {t}')
                print()

            self.__statistics.at[t, 'start_time'] = time()

            r = self.__optimization_round(starting_time_frame=t, initial_state=current_system_state)
            self.__relocation_solution.append(r)

            # TODO creare metodo per valutare domanda soddisfatta a partire da lista viaggi tenendo in considerazione la relocation
            current_system_state, satisifed_demand = self.__time_frame_transition(s=current_system_state, r=r,
                                                                                  time_frame=t)

            # current_system_state = [current_system_state[i] - EO[i] + EI[i] + r[i] for i in range(self.__zones_number)]

            self.__statistics.at[t, 'stop_time'] = time()

            self.__compute_time_frame_statistics(t, satisifed_demand)

            if self.__verbose:
                print()
                print(
                    f'Satisfied demand over time-frame {t} is {self.__statistics.at[t, "objective"]} out of {self.__statistics.at[t, "demand"]:.0f} ({self.__statistics.at[t, "stop_time"] - self.__statistics.at[t, "start_time"]:.0f} seconds elapsed - {self.__statistics.at[t, "stop_time"] - self.__statistics.at["overall", "start_time"]:.0f} in total)')
                [print() for _ in range(3)]

        # overall statistics
        self.__statistics.at['overall', 'stop_time'] = time()
        self.__statistics.at['overall', 'elapsed_time'] = self.__statistics.at['overall', 'stop_time'] - \
                                                          self.__statistics.at[
                                                              'overall', 'start_time']
        self.__statistics.at['overall', 'efficiency'] = self.__statistics.at['overall', 'objective'] / \
                                                        self.__statistics.at[
                                                            'overall', 'demand']

        return

    def __compute_time_frame_statistics(self, time_frame: int, satisfied_demand: float):

        self.__statistics.at[time_frame, 'elapsed_time'] = self.__statistics.at[time_frame, 'stop_time'] - \
                                                           self.__statistics.at[time_frame, 'start_time']

        self.__statistics.at[time_frame, 'objective'] = satisfied_demand
        self.__statistics.at['overall', 'objective'] += satisfied_demand

        self.__statistics.at[time_frame, 'efficiency'] = self.__statistics.at[time_frame, 'objective'] / \
                                                         self.__statistics.at[time_frame, 'demand']

        return

    def __optimization_round(self, starting_time_frame: int, initial_state: list) -> list:

        r = self.__initial_optimization(starting_time_frame=starting_time_frame, initial_state=initial_state)

        obj = self.__compute_objective(r=r, initial_state=initial_state, starting_time_frame=starting_time_frame)

        if self.__verbose:
            print(f'The satisfied demand after solving the optimization problem is {obj:.2f}')
            print()

        if self.__perform_neighbourhood_search:
            r = self.__neighbourhood_search(r0=r, initial_state=initial_state, starting_time_frame=starting_time_frame)

        return r[0]

    def __initial_optimization(self, starting_time_frame: int, initial_state: list) -> list:

        if self.__initial_optimization_method == 'LP+rounding':
            r = self.__lp_problem(starting_time_frame=starting_time_frame, initial_state=initial_state)

            r = self.__relocation_rounding(r=r, initial_state=initial_state)

        elif self.__initial_optimization_method == 'MILP':
            r = self.__milp_problem(starting_time_frame=starting_time_frame, initial_state=initial_state)

        elif self.__initial_optimization_method == 'LP+MILP':
            r = self.__lp_problem(starting_time_frame=starting_time_frame, initial_state=initial_state)

            zero_relocation_list = list()

            for t in range(self.__look_ahead_horizon):
                for i in range(self.__zones_number):
                    if abs(r[t][i].x) < TOLERANCE:
                        zero_relocation_list.append((t, i))

            r = self.__milp_problem(starting_time_frame=starting_time_frame, initial_state=initial_state,
                                    fix_zero_relocation=zero_relocation_list)

        return r

    def __lp_problem(self, starting_time_frame: int, initial_state: list) -> list:

        if self.__verbose:
            print('Solving LP problem')

        model = LpProblem(sense=LpMaximize)

        # declaring variables
        cat = 'Continuous'  # 'Integer' or 'Continuous'
        s = [[LpVariable(name='s_' + str(i) + '_' + str(t), lowBound=0, cat=cat) if t != 0 else
              initial_state[i] for i in range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        EI = [[LpVariable(name='EI_' + str(i) + '_' + str(t), lowBound=0,
                          upBound=self.__incoming_demand[t + starting_time_frame][i], cat=cat) for i in
               range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        EO = [[LpVariable(name='EO_' + str(i) + '_' + str(t), lowBound=0,
                          upBound=self.__outgoing_demand[t + starting_time_frame][i], cat=cat) for i in
               range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        r = [[LpVariable(name='r_' + str(i) + '_' + str(t), lowBound=-self.__maximum_relocation,
                         upBound=self.__maximum_relocation, cat=cat) for i in range(self.__zones_number)] for t in
             range(self.__look_ahead_horizon)]
        w = [[LpVariable(name='w_' + str(i) + '_' + str(t), lowBound=0, cat=cat) for i in range(self.__zones_number)]
             for t in range(self.__look_ahead_horizon)]

        # objective function
        model += lpSum(EO)  # g * lpSum(EO) - c * lpSum(w)

        for t in range(self.__look_ahead_horizon):
            model += (lpSum(w[t]) <= self.__maximum_relocation, 'max_relocation_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (w[t][i] >= r[t][i], 'max_relocation_nonnegativity_' + str(i) + '_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (
                    EO[t][i] <= s[t][i] + 0.5 * (EI[t][i] + r[t][i]),
                    'effective_outgoing_bound_' + str(i) + '_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (-r[t][i] <= s[t][i], 'relocation_bound_' + str(i) + '_' + str(t))

        # equality constraints
        for i in range(self.__zones_number):
            for t in range(1, self.__look_ahead_horizon):
                model += (s[t][i] == s[t - 1][i] + EI[t - 1][i] - EO[t - 1][i] + r[t - 1][i],
                          'state_transition_' + str(i) + '_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (lpSum(r[t]) == 0, 'tot_relocation_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (lpSum(s[t]) == self.__vehicle_number, 'mass_conservation_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (EI[t][i] == lpSum(
                    EO[t][j] * self.__origin_destination_matrix[t + starting_time_frame][j, i] for j in
                    range(self.__zones_number)), 'effective_incoming_outgoing_matching_' + str(i) + '_' + str(t))

        status = model.solve(PULP_CBC_CMD(msg=False))

        if status != 1:
            raise Exception('One optimization problem could not be solved')

        return [[r[t][i].value() for i in range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]

    def __milp_problem(self, starting_time_frame: int, initial_state: list, fix_zero_relocation: list = None) -> list:

        if self.__verbose:
            print('Solving MILP problem')

        model = Model(sense=MAXIMIZE)

        # declaring variables
        var_type = INTEGER  # INTEGER or CONTINUOUS
        s = [[model.add_var(name='s_' + str(i) + '_' + str(t), lb=0,
                            var_type=CONTINUOUS) if t != 0 else initial_state[i] for i in
              range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        EI = [[model.add_var(name='EI_' + str(i) + '_' + str(t), lb=0,
                             ub=self.__incoming_demand[t + starting_time_frame][i], var_type=CONTINUOUS) for i in
               range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        EO = [[model.add_var(name='EO_' + str(i) + '_' + str(t), lb=0,
                             ub=self.__outgoing_demand[t + starting_time_frame][i], var_type=CONTINUOUS) for i in
               range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]
        w = [[model.add_var(name='w_' + str(i) + '_' + str(t), lb=0, var_type=CONTINUOUS) for i in
              range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]

        if fix_zero_relocation is None:
            r = [[model.add_var(name='r_' + str(i) + '_' + str(t), lb=-self.__maximum_relocation,
                                ub=self.__maximum_relocation, var_type=var_type) for i in range(self.__zones_number)]
                 for t
                 in range(self.__look_ahead_horizon)]
        else:
            r = [[model.add_var(name='r_' + str(i) + '_' + str(t), lb=-self.__maximum_relocation,
                                ub=self.__maximum_relocation, var_type=var_type) if (t,
                                                                                     i) not in fix_zero_relocation else 0
                  for i in range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]

        # objective function
        model.objective = xsum(EO[t][i] for t in range(self.__look_ahead_horizon) for i in range(self.__zones_number))

        # inequality constraints
        for t in range(self.__look_ahead_horizon):
            model += (
                xsum(w[t][i] for i in range(self.__zones_number)) <= self.__maximum_relocation,
                'max_relocation_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (w[t][i] >= r[t][i], 'max_relocation_nonnegativity_' + str(i) + '_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (
                    EO[t][i] <= s[t][i] + 0.5 * (EI[t][i] + r[t][i]),
                    'effective_outgoing_bound_' + str(i) + '_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (-r[t][i] <= s[t][i], 'relocation_bound_' + str(i) + '_' + str(t))

        # equality constraints
        for i in range(self.__zones_number):
            for t in range(1, self.__look_ahead_horizon):
                model += (s[t][i] == s[t - 1][i] + EI[t - 1][i] - EO[t - 1][i] + r[t - 1][i],
                          'state_transition_' + str(i) + '_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (xsum(r[t][i] for i in range(self.__zones_number)) == 0, 'tot_relocation_' + str(t))

        for t in range(self.__look_ahead_horizon):
            model += (
                xsum(s[t][i] for i in range(self.__zones_number)) == self.__vehicle_number,
                'mass_conservation_' + str(t))

        for i in range(self.__zones_number):
            for t in range(self.__look_ahead_horizon):
                model += (EI[t][i] == xsum(
                    EO[t][j] * self.__origin_destination_matrix[t + starting_time_frame][j, i] for j in
                    range(self.__zones_number)), 'effective_incoming_outgoing_matching_' + str(i) + '_' + str(t))

        model.verbose = 0
        status = model.optimize(max_seconds_same_incumbent=self.__max_seconds_same_incumbent)

        if status != OptimizationStatus.OPTIMAL:
            print(sum(initial_state), initial_state)
            print(any([initial_state[i]<0 for i in range(self.__zones_number)]))
            raise Exception('One optimization problem could not be solved')

        return [[r[t][i].x for i in range(self.__zones_number)] for t in range(self.__look_ahead_horizon)]

    def __relocation_rounding(self, r: list, initial_state: list) -> list:

        rounded_solution = list()

        for t in range(self.__look_ahead_horizon):
            rounding = list(map(custom_round, r[t]))

            if t == 0:
                for i in range(self.__zones_number):
                    if initial_state[i] + rounding[i] < 0:
                        rounding[i] -= initial_state[i] + rounding[i]

            pos_rounded_indexes = [i for i in range(self.__zones_number) if
                                   (rounding[i] - r[t][i]) > TOLERANCE]  # identifying indexes of rounded entries
            neg_rounded_indexes = [i for i in range(self.__zones_number) if (r[t][i] - rounding[i]) > 1e-5]

            pos_rel = [i for i in pos_rounded_indexes if rounding[i] > 0]
            neg_rel = [i for i in neg_rounded_indexes if rounding[i] < 0]

            pos_excess = max(sum([val for val in rounding if val > 0]) - self.__maximum_relocation, 0)
            neg_excess = max(sum([-val for val in rounding if val < 0]) - self.__maximum_relocation, 0)

            pos_roundings = list(
                map(lambda x: x[0], sorted([(i, abs(r[t][i] - rounding[i])) for i in pos_rel],
                                           key=lambda x: x[1], reverse=True)))
            neg_roundings = list(
                map(lambda x: x[0], sorted([(i, abs(r[t][i] - rounding[i])) for i in neg_rel],
                                           key=lambda x: x[1], reverse=True)))

            while pos_excess > 0:

                if len(pos_roundings) > 0:
                    ind = pos_roundings.pop(0)
                else:
                    ind = random.sample(range(self.__zones_number), k=1)[0]

                if t == 0 and initial_state[ind] + rounding[ind] > 0:
                    rounding[ind] -= 1
                else:
                    rounding[ind] -= 1

                pos_excess -= 1

            while neg_excess > 0:

                if len(neg_roundings) > 0:
                    ind = neg_roundings.pop(0)
                else:
                    ind = random.sample(range(self.__zones_number), k=1)[0]

                if t == 0 and initial_state[ind] + rounding[ind] > 0:
                    rounding[ind] += 1
                else:
                    rounding[ind] += 1

                neg_excess -= 1

            unbalance = sum([val for val in rounding if val > 0]) - sum([-val for val in rounding if val < 0])

            while unbalance > 0:

                if len(pos_roundings) > 0:
                    ind = pos_roundings.pop(0)
                else:
                    ind = random.sample(range(self.__zones_number), k=1)[0]

                if t == 0 and initial_state[ind] + rounding[ind] > 0:
                    rounding[ind] -= 1
                else:
                    rounding[ind] -= 1

                unbalance -= 1

            while unbalance < 0:

                if len(neg_roundings) > 0:
                    ind = neg_roundings.pop(0)
                else:
                    ind = random.sample(range(self.__zones_number), k=1)[0]

                if t == 0 and initial_state[ind] + rounding[ind] > 0:
                    rounding[ind] += 1
                else:
                    rounding[ind] += 1

                unbalance += 1

            rounded_solution.append(rounding)

        return rounded_solution

    def __neighbourhood_search(self, r0: list, initial_state: list, starting_time_frame: int) -> list:

        if self.__verbose:
            print('Starting neighbourhood search')

        incumbent_r = r0
        incumbent_objective = self.__compute_objective(r=r0, initial_state=initial_state,
                                                       starting_time_frame=starting_time_frame)

        iters = 0
        iters_no_improvement = 0

        while iters < self.__max_iterations and iters_no_improvement < self.__max_iterations_no_improvement:

            t = random.sample(range(self.__look_ahead_horizon), k=1)[0]

            if self.__verbose:
                print(
                    f'Performing iteration {iters + 1} on time-frame {starting_time_frame + t} ({time() - self.__statistics.at[starting_time_frame, "start_time"]:.0f} seconds)')

            incumbent_r, incumbent_objective, improved = self.__time_frame_search(t, incumbent_r, incumbent_objective,
                                                                                  initial_state, starting_time_frame)

            if not improved:
                iters_no_improvement += 1
            else:
                iters_no_improvement = 0

            if self.__verbose:

                if improved:
                    print(
                        f'Iteration {iters + 1} out of {self.__max_iterations} increased the objective to {incumbent_objective:.0f}')
                    print()
                else:
                    print(
                        f'Iteration {iters + 1} out of {self.__max_iterations} decreased the objective to {incumbent_objective:.0f}')
                    print()

            iters += 1

        return incumbent_r

    # FIRST IMPROVEMENT
    def __time_frame_search(self, t: int, incumbent_r: list, incumbent_objective: int, initial_state: list,
                            starting_time_frame: int, iteration: int):

        max_relocation_reached = np.sum([incumbent_r[t][k] for k in range(self.__zones_number) if
                                         incumbent_r[t][k] > 0]) == self.__maximum_relocation

        if max_relocation_reached:
            pos_relocation_idx = [i for i in range(self.__zones_number) if incumbent_r[t][i] > 0]
            neg_relocation_idx = [i for i in range(self.__zones_number) if incumbent_r[t][i] < 0]
            zero_relocation_idx = [i for i in range(self.__zones_number) if incumbent_r[t][i] == 0]

            indexes = list()
            indexes.extend([(i, j) for i in pos_relocation_idx for j in pos_relocation_idx if j != i])
            indexes.extend([(i, j) for i in zero_relocation_idx for j in pos_relocation_idx])
            indexes.extend([(i, j) for i in neg_relocation_idx for j in range(self.__zones_number) if j != i])
        else:
            indexes = [(i, j) for i in range(self.__zones_number) for j in range(self.__zones_number) if j != 1]

        random.shuffle(indexes)

        sample_size = len(indexes)
        perc = -1

        for it, (i, j) in enumerate(indexes):

            if self.__verbose:
                adv = round((it / sample_size) * 100)
                if adv != perc and adv % 5 == 0:
                    perc = adv
                    print(f'{perc}% of neighbourhood explored')

            r_copy = copy.deepcopy(incumbent_r)
            r_copy[t][i] += 1
            r_copy[t][j] -= 1

            new_objective = self.__compute_objective(r_copy, initial_state, starting_time_frame)

            # FIRST IMPROVEMENT
            #     if new_objective > incumbent_objective:
            #         if self.__verbose:
            #             print(f'New optimum found after exploring {adv}% of the neighbourhood ({it} neighbours)')
            #
            #         return r_copy, new_objective, True
            # return incumbent_r, incumbent_objective, False

            if new_objective > incumbent_objective:

                return r_copy, new_objective, True

            elif np.random.binomial(1, self.__compute_acceptance_probability(iteration, new_objective,
                                                                             incumbent_objective)):
                return r_copy, new_objective, False

        return incumbent_r, incumbent_objective, False

    # def __time_frame_search(self, t: int, incumbent_r: list, incumbent_objective: int, initial_state: list,
    #                         starting_time_frame: int, iteration: int) -> (list, float, bool):
    #
    #     max_relocation_reached = np.sum([incumbent_r[t][k] for k in range(self.__zones_number) if
    #                                      incumbent_r[t][k] > 0]) == self.__maximum_relocation
    #
    #     if max_relocation_reached:
    #         pos_relocation_idx = [i for i in range(self.__zones_number) if incumbent_r[t][i] > 0]
    #         neg_relocation_idx = [i for i in range(self.__zones_number) if incumbent_r[t][i] < 0]
    #         zero_relocation_idx = [i for i in range(self.__zones_number) if incumbent_r[t][i] == 0]
    #
    #         indexes = list()
    #         indexes.extend([(i, j) for i in pos_relocation_idx for j in pos_relocation_idx if j != i])
    #         indexes.extend([(i, j) for i in zero_relocation_idx for j in pos_relocation_idx])
    #         indexes.extend([(i, j) for i in neg_relocation_idx for j in range(self.__zones_number) if j != i])
    #     else:
    #         indexes = [(i, j) for i in range(self.__zones_number) for j in range(self.__zones_number) if j != i]
    #
    #     random.shuffle(indexes)
    #
    #     sample_size = len(indexes)
    #     look_up_sample_size = int(sample_size / e)
    #
    #     look_up_max = 0
    #
    #     for i, j in indexes[:look_up_sample_size]:
    #         r_copy = copy.deepcopy(incumbent_r)
    #         r_copy[t][i] += 1
    #         r_copy[t][j] -= 1
    #
    #         new_objective = self.__compute_objective(r_copy, initial_state, starting_time_frame)
    #
    #         if new_objective > look_up_max:
    #             look_up_max = new_objective
    #
    #     for i, j in indexes[look_up_sample_size:]:
    #         r_copy = copy.deepcopy(incumbent_r)
    #         r_copy[t][i] += 1
    #         r_copy[t][j] -= 1
    #
    #         new_objective = self.__compute_objective(r_copy, initial_state, starting_time_frame)
    #
    #         if new_objective > look_up_max:
    #             if new_objective > incumbent_objective:
    #                 return r_copy, new_objective, True
    #             else:
    #                 return r_copy, new_objective, False
    #
    #         elif np.random.binomial(1, self.__compute_acceptance_probability(iteration, new_objective, look_up_max)):
    #             if new_objective > incumbent_objective:
    #                 return r_copy, new_objective, True
    #             else:
    #                 return r_copy, new_objective, False
    #
    #     return incumbent_r, incumbent_objective, False

    def __compute_acceptance_probability(self, iteration: int, new_objective: float, current_objective: int) -> float:
        d = 10  # TODO trovare valore ottimale (provare a risolvere lp per upper bound?)
        return np.exp(-(new_objective - current_objective) / (d / np.log(iteration + 1)))

    def __compute_objective(self, r: list, initial_state: list, starting_time_frame: int) -> float:

        if self.__next_state_simulation == 'best_case':

            s = initial_state
            total_objective = 0

            for time_frame in range(self.__look_ahead_horizon):
                EO, EI = self.__system_lp_problem(r=r[time_frame], s=s, time_frame=time_frame,
                                                  starting_time_frame=starting_time_frame)
                EO, EI = self.__round_flows(EO=EO, EI=EI, r=r[time_frame], s=s)

                total_objective += sum(EO)

                s = [s[i] - EO[i] + EI[i] + r[time_frame][i] for i in range(self.__zones_number)]

        elif self.__next_state_simulation == 'Monte_Carlo':
            total_objective = self.__monte_carlo_simulation(r, initial_state, starting_time_frame)

        return total_objective

    def __round_flows(self, EO: list, EI: list, r: list, s: list) -> (list, list):
        rounded_out = list(map(round, EO))
        rounded_in = list(map(round, EI))

        unbalance = sum(rounded_out) - sum(rounded_in)

        if unbalance > 0:
            indexes = self.__get_rounding_indexes(EO, rounded_out)

            while unbalance > 0:

                if len(indexes) > 0:
                    ind = indexes.pop(0)
                else:
                    ind = random.sample(range(self.__zones_number), k=1)[0]

                if s[ind] - (rounded_out[ind] - 1) + rounded_in[ind] + r[ind] >= 0:
                    rounded_out[ind] -= 1
                    unbalance -= 1

        elif unbalance < 0:
            indexes = self.__get_rounding_indexes(EI, rounded_in)

            while unbalance < 0:

                if len(indexes) > 0:
                    ind = indexes.pop(0)
                else:
                    ind = random.sample(range(self.__zones_number), k=1)[0]

                if s[ind] - rounded_out[ind] + (rounded_in[ind] - 1) + r[ind] >= 0:
                    rounded_in[ind] -= 1
                    unbalance += 1

        return rounded_out, rounded_in

    def __get_rounding_indexes(self, array: list, rounded_array: list) -> list:

        rounded_indexes = [(i, (rounded_array[i] - array[i])) for i in range(len(array)) if
                           (rounded_array[i] - array[i]) > 1e-5]  # identifying indexes of rounded entries

        rounded_indexes.sort(key=lambda x: x[1], reverse=True)

        return list(map(lambda x: x[0], rounded_indexes))

    def __system_lp_problem(self, r: list, s: list, time_frame: int, starting_time_frame: int) -> (list, list):

        m = LpProblem(sense=LpMaximize)

        # declaring variables
        cat = 'Continuous'  # 'Integer' or 'Continuous'
        EO = [LpVariable(name='EO_' + str(i), upBound=self.__outgoing_demand[starting_time_frame + time_frame][i],
                         cat=cat) for i in range(self.__zones_number)]
        EI = [LpVariable(name='EI_' + str(i), upBound=self.__incoming_demand[starting_time_frame + time_frame][i],
                         cat=cat) for i in range(self.__zones_number)]

        # objective function
        m += lpSum(EO)

        for i in range(self.__zones_number):
            m += (EI[i] == lpSum(
                EO[j] * self.__origin_destination_matrix[starting_time_frame + time_frame][j, i] for j in
                range(self.__zones_number)), 'effective_incoming_outgoing_matching_' + str(i))
            m += (EO[i] <= s[i] + 0.5 * (EI[i] + r[i]), 'effective_outgoing_bound_' + str(i))
            m += (s[i] + EI[i] - EO[i] + r[i] >= 0, 'next_state_' + str(i))

        m.solve(PULP_CBC_CMD(msg=False))

        return [EO[i].value() for i in range(self.__zones_number)], [EI[i].value() for i in range(self.__zones_number)]

    def __monte_carlo_simulation(self, r, initial_state, starting_time_frame) -> float:

        total_objective = 0

        for simulation in range(self.__mc_simulations):
            s = initial_state

            for time_frame in range(self.__look_ahead_horizon):
                s, EO, EI = self.__single_tf_run(s=s, r=r[time_frame], time_frame=starting_time_frame + time_frame)

                total_objective += sum(EO)

        return total_objective / self.__mc_simulations

    def __single_tf_run(self, s: list, r: list, time_frame: int) -> (list, list):

        s_r = [s[i] + r[i] for i in range(self.__zones_number)]

        outgoing = [0] * self.__zones_number
        incoming = [0] * self.__zones_number

        out_zones = {i for i in range(self.__zones_number) if
                     self.__outgoing_demand[time_frame][i] > 0}  # set of zones from which vehicles can leave
        in_zones = {i for i in range(self.__zones_number) if
                    self.__incoming_demand[time_frame][i] > 0}  # set of zones to which vehicles can get

        _out = self.__outgoing_demand[time_frame].copy()
        _in = self.__incoming_demand[time_frame].copy()

        counter = 0

        while len(out_zones) > 0 and len(in_zones) > 0 and counter < 40:
            start_zone = random.sample(out_zones, k=1)[0]
            arrival_zone = random.sample([j for j in range(self.__zones_number) if
                                          self.__origin_destination_matrix[time_frame][start_zone, j] > 0], k=1)[0]

            if s_r[start_zone] > 0 and arrival_zone in in_zones:
                s_r[start_zone] -= 1
                s_r[arrival_zone] += 1

                outgoing[start_zone] += 1
                incoming[arrival_zone] += 1

                _out[start_zone] -= 1
                if _out[start_zone] == 0:
                    out_zones.discard(start_zone)
                _in[arrival_zone] -= 1
                if _in[arrival_zone] == 0:
                    in_zones.discard(arrival_zone)

                counter = 0
            else:
                counter += 1

        return s_r, outgoing, incoming

    def __time_frame_transition(self, s: list, r: list, time_frame: int):

        flows = self.__traffic_flow_simulator.get_flow_by_time_frame(time_frame)

        s_r = [s[i] + r[i] for i in range(self.__zones_number)]

        outgoing = [0] * self.__zones_number

        for start_zone, arrival_zone in flows:
            a = s_r[start_zone]
            b = s_r[arrival_zone]
            if s_r[start_zone] > 0:
                s_r[start_zone] -= 1
                s_r[arrival_zone] += 1

                outgoing[start_zone] += 1

        return s_r, np.sum(outgoing)

    def get_optimal_relocation(self) -> list:
        return self.__relocation_solution

    def get_total_satisfied_demand(self) -> int:
        return self.__statistics.at['overall', 'demand']

    def result_summary(self):
        print(
            f'The total satisfied demand is {self.__statistics.at["overall", "objective"]} out of {self.__statistics.at["overall", "demand"]:.0f} ({self.__statistics.at["overall", "efficiency"]:.2%} efficiency)')
        print(f'The whole optimization process took {self.__statistics.at["overall", "elapsed_time"]:.0f} seconds')
        [print() for _ in range(3)]

        for t in range(self.__optimization_horizon):
            print(f'Time-frame {t}:')
            print(
                f'\tSatisfied demand is {self.__statistics.at[t, "objective"]} out of {self.__statistics.at[t, "demand"]:.0f} ({self.__statistics.at[t, "efficiency"]:.2%} efficiency)')
            print(f'\tThe optimization process took {self.__statistics.at[t, "elapsed_time"]:.0f} seconds')
            print()

    def plot_relocation(self, filename: str):

        df = pd.DataFrame(data=[[t, i, self.__relocation_solution[t][i], self.__relocation_solution[t][i] > 0] for i in
                                range(self.__zones_number) for t in
                                range(self.__optimization_horizon) if self.__relocation_solution[t][i] != 0],
                          columns=['time_frame', 'zone', 'relocation', 'pos_relocation'])

        p = p9.ggplot(df) + p9.facet_wrap(facets='time_frame', nrow=self.__optimization_horizon // 4 + 1,
                                          labeller=lambda t: 'time-frame ' + str(t))
        p = p + p9.aes(x='zone', y='relocation') + p9.labs(x='zone', y='relocation value',
                                                           title='Relocation by time-frame')
        p = p + p9.geom_point(p9.aes(color='pos_relocation'), show_legend=False, alpha=0.9) + p9.geom_hline(
            yintercept=0, linetype='dashed', alpha=0.7)
        p9.ggsave(plot=p, filename=filename, device='png', dpi=320)

        return

    def get_overall_statistics(self):
        return self.__statistics[['efficiency', 'elapsed_time']].loc['overall']  # access by adding ['efficiency']

    def get_statistics(self):
        return self.__statistics[['efficiency', 'elapsed_time']]