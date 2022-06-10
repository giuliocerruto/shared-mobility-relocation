import numpy as np
from RelocationOptimizer import RelocationOptimizer
from TrafficFlowSimulator import TrafficFlowSimulator
from StatisticsCollector import StatisticsCollector
import cProfile, pstats
from pstats import SortKey
import pickle


def temperature_function(iteration, max_iterations=0):
    T0 = 1
    alpha = 0.9
    return T0 * (alpha ** iteration)


def main():
    N = 396  # total number of vehicles
    R_max = 10  # maximum number of vehicles which can be relocated in a time frame

    od_mat = np.load('origin_destination_matrix.npz')['x'].astype(np.uint8)

    # N_in = np.load('N_in.npz')['x'].astype(np.int8)
    # # N_in = np.vstack((0 * N_in[0, :], N_in))
    # N_out = np.load('N_out.npz')['x'].astype(np.int8)
    # # N_out = np.vstack((0 * N_out[0, :], N_out))
    # od_mat = np.load('od_mat.npz')['matrix'].astype(np.uint8)
    # # od_mat = np.insert(od_mat, 0, 0 * od_mat[0], axis=0)
    # p = np.load('od_mat.npz')['prob_matrix']
    # # p = np.insert(p, 0, 0 * p[0], axis=0)

    tfs = TrafficFlowSimulator(origin_destination_matrix=od_mat)
    #
    # ro = RelocationOptimizer(vehicle_number=N, maximum_relocation=R_max,
    #                          origin_destination_matrix=od_mat, optimization_horizon=2,
    #                          look_ahead_horizon=2, traffic_flow_simulator=tfs, verbose=True)
    #
    # ro.set_optimization_procedure(initial_optimization_method='LP+rounding', perform_neighbourhood_search=True,
    #                               next_state_simulation='best_case', max_seconds_same_incumbent=60 * 60 * 1,
    #                               mc_simulations=5, max_iterations=50, max_iterations_no_improvement=5,
    #                               temperature_function=temperature_function)
    #
    # ro.optimize()

    # with open('tempi_mc', 'wb') as f:
    #     pickle.dump(ro.tempi(), f)

    # # print(ro.get_optimal_relocation())
    # # print(ro.get_total_satisfied_demand())
    # ro.result_summary()
    # ro.plot_relocation(filename='relocation')



    sc = StatisticsCollector(vehicle_number=N,
                             maximum_relocation=R_max,
                             origin_destination_matrix=od_mat,
                             optimization_horizon_values=[10],
                             look_ahead_horizon_values=[1, 2, 4, 8, 10],
                             initial_optimization_method='LP+rounding',
                             perform_neighbourhood_search=True,
                             next_state_simulation='best_case',
                             max_seconds_same_incumbent=60 * 60 * 0.5,
                             mc_simulations=50,
                             max_iterations=50,
                             max_iterations_no_improvement=20,
                             traffic_flow_simulator=tfs,
                             temperature_function=temperature_function)
    sc.save_results('results_N_' + str(N) + '_Rmax_' + str(R_max))
    sc.run()
    sc.plot_efficiency('efficiency_N_' + str(N) + '_Rmax_' + str(R_max))
    sc.plot_elapsed_time('elapsed_time_N_' + str(N) + '_Rmax_' + str(R_max))


if __name__ == '__main__':
    # profiler = cProfile.Profile()
    # profiler.enable()
    main()
    # profiler.disable()
    # stats = pstats.Stats(profiler).sort_stats(SortKey.TIME)
    # stats.dump_stats('stats.dmp')
