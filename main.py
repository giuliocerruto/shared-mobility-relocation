import numpy as np
from RelocationOptimizer import RelocationOptimizer

if __name__ == '__main__':
    N = 396  # total number of vehicles
    R_max = 10  # maximum number of vehicles which can be relocated in a time frame

    N_in = np.load('N_in.npz')['x']
    N_in = np.vstack((0 * N_in[0, :], N_in))
    N_out = np.load('N_out.npz')['x']
    N_out = np.vstack((0 * N_out[0, :], N_out))
    p = np.load('od_mat.npz')['prob_matrix']
    p = np.insert(p, 0, 0 * p[0], axis=0)

    ro = RelocationOptimizer(vehicle_number=N, maximum_relocation=R_max, incoming_demand=N_in, outgoing_demand=N_out,
                             origin_destination_matrix=p, optimization_horizon=20, look_ahead_horizon=10)

