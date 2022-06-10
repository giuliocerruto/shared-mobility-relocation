import pandas as pd
from RelocationOptimizer import RelocationOptimizer
from TrafficFlowSimulator import TrafficFlowSimulator
import numpy as np
import plotnine as p9


class StatisticsCollector:

    def __init__(self,
                 vehicle_number: int,
                 maximum_relocation: int,
                 origin_destination_matrix: np.array,
                 optimization_horizon_values: list,
                 look_ahead_horizon_values: list,
                 initial_optimization_method: str = 'LP+rounding',
                 perform_neighbourhood_search: bool = True,
                 next_state_simulation: str = 'Monte_Carlo',
                 max_seconds_same_incumbent: float = 60 * 60 * 3,
                 mc_simulations: int = 50,
                 max_iterations: int = 100,
                 max_iterations_no_improvement: int = 10,
                 temperature_function=lambda it: 1 / it,
                 traffic_flow_simulator: TrafficFlowSimulator = None,
                 verbose: bool = True):

        if vehicle_number < 0:
            raise ValueError('The number of vehicles can\'t be negative')

        self.__vehicle_number = vehicle_number

        if maximum_relocation < 0:
            raise ValueError('The maximum number of vehicles to be relocated can\'t be negative')

        self.__maximum_relocation = maximum_relocation

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

        # if origin_destination_probability.shape[0] != self.__time_frames_number:
        #     raise ValueError(
        #         'The origin-destination probability matrix shape doesn\'t match that of the demand lists (the number of time frames is different)')
        #
        # if origin_destination_probability.shape[1] != origin_destination_probability.shape[2] or \
        #         origin_destination_probability.shape[
        #             1] != self.__zones_number:
        #     raise ValueError(
        #         'The origin-destination probability matrix shape doesn\'t match that of the demand lists (the number of zones is different or the matrices are not sqaure)')
        #
        # self.__origin_destination_probability = origin_destination_probability

        self.__optimization_horizon_values = optimization_horizon_values
        self.__look_ahead_horizon_values = look_ahead_horizon_values

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

        if not isinstance(verbose, bool):
            raise ValueError('Verbose can be either False (deactivated) or True (activated)')

        self.__traffic_flow_simulator = traffic_flow_simulator

        self.__temperature_function = temperature_function

        self.__verbose = verbose

        self.__statistics = pd.DataFrame(
            columns=['optimization_horizon', 'look_ahead_horizon', 'time_frame', 'efficiency', 'efficiency_upper_bound',
                     'elapsed_time'])

        self.__filename = None  # file to save results

        return

    def run(self):

        for optimization_horizon in self.__optimization_horizon_values:
            for look_ahead_horizon in self.__look_ahead_horizon_values:

                ro = RelocationOptimizer(vehicle_number=self.__vehicle_number,
                                         maximum_relocation=self.__maximum_relocation,
                                         origin_destination_matrix=self.__origin_destination_matrix,
                                         optimization_horizon=optimization_horizon,
                                         look_ahead_horizon=look_ahead_horizon,
                                         traffic_flow_simulator=self.__traffic_flow_simulator,
                                         verbose=self.__verbose)

                ro.set_optimization_procedure(initial_optimization_method=self.__initial_optimization_method,
                                              perform_neighbourhood_search=self.__perform_neighbourhood_search,
                                              next_state_simulation=self.__next_state_simulation,
                                              max_seconds_same_incumbent=self.__max_seconds_same_incumbent,
                                              mc_simulations=self.__mc_simulations,
                                              max_iterations=self.__max_iterations,
                                              max_iterations_no_improvement=self.__max_iterations_no_improvement,
                                              temperature_function=self.__temperature_function)

                ro.optimize()

                if self.__verbose:
                    ro.result_summary()

                ro.plot_relocation(
                    filename='relocation_oh_' + str(optimization_horizon) + '_lah_' + str(look_ahead_horizon))

                stats = ro.get_statistics()

                for index, row in stats.iterrows():
                    self.__statistics = self.__statistics.append(
                        {'optimization_horizon': int(optimization_horizon),
                         'look_ahead_horizon': int(look_ahead_horizon),
                         'time_frame': index,
                         'efficiency': float(row['efficiency']),
                         'efficiency_upper_bound': float(row['efficiency_upper_bound']),
                         'elapsed_time': float(row['elapsed_time'])},
                        ignore_index=True)

                for column in ['efficiency', 'efficiency_upper_bound', 'elapsed_time']:
                    stats[column] = pd.to_numeric(stats[column])

                if self.__filename is not None:
                    with open(self.__filename + '.txt', 'a') as f:
                        for idx, row in stats.iterrows():
                            print(
                                f'{self.__initial_optimization_method};{self.__perform_neighbourhood_search};{self.__next_state_simulation};{int(optimization_horizon)};{int(look_ahead_horizon)};{idx};{row["efficiency"]};{row["efficiency_upper_bound"]};{row["elapsed_time"]}',
                                file=f)

        for column in ['efficiency', 'efficiency_upper_bound', 'elapsed_time']:
            self.__statistics[column] = pd.to_numeric(self.__statistics[column])

        return

    def plot_efficiency(self, filename: str):

        p = p9.ggplot(self.__statistics) + p9.aes(x='look_ahead_horizon', y='efficiency')

        # upper bounds
        p = p + p9.geom_boxplot(data=self.__statistics[self.__statistics['time_frame'] != 'overall'],
                                mapping=p9.aes(group='look_ahead_horizon', y='efficiency_upper_bound'), alpha=0.3,
                                color='lightsteelblue', fatten=0, fill='lightsteelblue', width=0.2, outlier_alpha=0.5)
        p = p + p9.geom_line(data=self.__statistics[self.__statistics['time_frame'] == 'overall'], linetype='-.',
                             mapping=p9.aes(group='time_frame', y='efficiency_upper_bound'), alpha=0.4,
                             color='lightsteelblue')
        p = p + p9.geom_point(data=self.__statistics[self.__statistics['time_frame'] == 'overall'],
                              color='blue', shape=7,
                              size=4, alpha=0.3, mapping=p9.aes(y='efficiency_upper_bound'))

        # actual values
        p = p + p9.geom_boxplot(data=self.__statistics[self.__statistics['time_frame'] != 'overall'],
                                mapping=p9.aes(group='look_ahead_horizon'), alpha=0.3, color='deepskyblue', fatten=0,
                                fill='deepskyblue', width=0.3, outlier_alpha=0.5)

        p = p + p9.geom_line(data=self.__statistics[self.__statistics['time_frame'] == 'overall'], linetype='-.',
                             mapping=p9.aes(group='time_frame'), alpha=0.6)
        p = p + p9.geom_point(data=self.__statistics[self.__statistics['time_frame'] == 'overall'], color='indianred',
                              size=4, alpha=1)
        p = p + p9.labs(x='look ahead horizon', y='efficiency', title='Efficiency by look ahead horizon')

        p9.ggsave(plot=p, filename=filename, device='png', dpi=320)

        return

    def plot_elapsed_time(self, filename: str):

        p = p9.ggplot(self.__statistics) + p9.aes(x='look_ahead_horizon', y='elapsed_time')
        p = p + p9.geom_boxplot(data=self.__statistics[self.__statistics['time_frame'] != 'overall'],
                                mapping=p9.aes(group='look_ahead_horizon'), alpha=0.3, color='deepskyblue', fatten=0,
                                fill='deepskyblue', width=0.3, outlier_alpha=0.5)

        p = p + p9.geom_line(data=self.__statistics[self.__statistics['time_frame'] == 'overall'], linetype='-.',
                             mapping=p9.aes(group='time_frame'), alpha=0.6)
        p = p + p9.geom_point(data=self.__statistics[self.__statistics['time_frame'] == 'overall'], color='indianred',
                              size=4, alpha=1)
        p = p + p9.labs(x='look ahead horizon', y='elapsed time (s)', title='Elapsed time by look ahead horizon')

        p9.ggsave(plot=p, filename=filename, device='png', dpi=320)

        return

    def save_results(self, filename: str):
        self.__filename = filename

        return
