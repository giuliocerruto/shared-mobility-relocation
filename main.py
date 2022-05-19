import numpy as np
from RelocationOptimizer import RelocationOptimizer
from TrafficFlowSimulator import TrafficFlowSimulator
from StatisticsCollector import StatisticsCollector

if __name__ == '__main__':
    N = 396  # total number of vehicles
    R_max = 10  # maximum number of vehicles which can be relocated in a time frame

    N_in = np.load('N_in.npz')['x']
    # N_in = np.vstack((0 * N_in[0, :], N_in))
    N_out = np.load('N_out.npz')['x']
    # N_out = np.vstack((0 * N_out[0, :], N_out))
    p = np.load('od_mat.npz')['prob_matrix']
    # p = np.insert(p, 0, 0 * p[0], axis=0)

    tfs = TrafficFlowSimulator(vehicle_number=N, maximum_relocation=R_max, incoming_demand=N_in, outgoing_demand=N_out,
                               origin_destination_matrix=p)

    ro = RelocationOptimizer(vehicle_number=N, maximum_relocation=R_max, incoming_demand=N_in, outgoing_demand=N_out,
                             origin_destination_matrix=p, optimization_horizon=2, look_ahead_horizon=2,
                             traffic_flow_simulator=tfs, verbose=False)

    ro.set_optimization_procedure(initial_optimization_method='LP+rounding', perform_neighbourhood_search=False,
                                  next_state_simulation='best_case', max_seconds_same_incumbent=60 * 60 * 1,
                                  mc_simulations=5, max_iterations=200, max_iterations_no_improvement=20)

    ro.optimize()

    # print(ro.get_optimal_relocation())
    # print(ro.get_total_satisfied_demand())
    ro.result_summary()
    ro.plot_relocation()

    # TODO parallelizzare?
