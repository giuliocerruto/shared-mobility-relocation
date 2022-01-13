from math import floor
import numpy as np
from main import positive_part


class State:

    def __init__(self, st: np.array):
        self.x = st
        self.reachableBy = list()
        self.__valuet = dict()
        self.__next = dict()

    def is_reachable_by(self, state: "State", max_relocation: int) -> bool:
        return sum(positive_part(state.x - self.x)) < max_relocation

    def reachable_by(self, st2: "State"):
        self.reachableBy.append(st2)

    def get_reachable_states(self):
        return self.reachableBy

    def set_value(self, t: int, value: float):
        self.__valuet[t] = value

    def get_value(self, t: int) -> float:
        return self.__valuet.get(t)

    def set_next(self, t: int, next_s: "State"):
        self.__next[t] = next_s

    def get_next(self, t: int):
        return self.__next[t]

    def __str__(self):
        return str(tuple(self.x))


class StateSpace:

    def __init__(self, n: int, max_relocation: int):

        self.__states = dict()

        for i in range(floor(0.75 * n) + 1):
            for j in range(min(n + 1 - i, floor(0.75 * n) + 1)):
                for k in range(min(n + 1 - i - j, floor(0.75 * n) + 1)):
                    if (n - i - j - k) < (floor(0.75 * n) + 1):
                        self.__states[(i, j, k, n - i - j - k)] = State(np.array((i, j, k, n - i - j - k)))

        # for i in range(n + 1):
        #     for j in range(n + 1 - i):
        #         for k in range(n + 1 - i - j):
        #             self.__states[(i, j, k, n - i - j - k)] = State(np.array((i, j, k, n - i - j - k)))

        print('State space generated (' + str(len(self.__states)) + ' states)')

        print('Computing adjacency state of the system...')

        stl = list(self.__states.values())

        for idx1, st1 in enumerate(stl):
            for idx2 in range(idx1):
                if st1.is_reachable_by(stl[idx2], max_relocation):
                    st1.reachable_by(stl[idx2])
                    stl[idx2].reachable_by(st1)

    def __iter__(self):
        return iter(self.__states.values())

    def get_state(self, st: tuple):
        return self.__states.get(st)
