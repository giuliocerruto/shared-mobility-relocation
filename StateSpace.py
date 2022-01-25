from math import floor
import numpy as np
from main import positive_part


def toint(num: str) -> int:
    return int(num)


def str_to_tuple(stt: str):
    return tuple(map(toint, stt.strip('(').strip(')').split(', ')))


class State:

    def __init__(self, st):
        self.x = np.array(st, dtype='int32')
        self.reachableBy = list()
        self.__valuet = dict()
        self.__next = dict()
        self.x_demand = None

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

    def __add__(self, other: "State"):
        return self.x + other.x

    def __sub__(self, other: "State"):
        return self.x - other.x

    def compute_after_demand(self, _out, _in, excess_demand):
        self.x_demand = self.x + _in - _out
        z = len(_out)
        excess = excess_demand
        indexes = list(range(z))

        for i in range(z):
            if self.x_demand[i] < 0:
                excess += self.x_demand[i]
                self.x_demand[i] = 0
                indexes.remove(i)
            elif self.x_demand[i] == 0:
                indexes.remove(i)

        il = len(indexes)
        i = 0

        while excess > 0:
            self.x_demand[indexes[i]] += 1
            excess -= 1

            if i != il - 1:
                i += 1
            else:
                i = 0

        while excess < 0:
            if self.x_demand[indexes[i]] > 0:
                self.x_demand[indexes[i]] -= 1
                excess += 1

            if i != il - 1:
                i += 1
            else:
                i = 0

    def get_after_demand(self):
        return tuple(self.x_demand)


class StateSpace:

    def __init__(self, n: int = -1, max_relocation: int = -1):

        self.__states = dict()
        self.N = n

        if isinstance(max_relocation, int):

            self.maxRelocation = max_relocation

            for i in range(floor(0.75 * n) + 1):
                for j in range(min(n + 1 - i, floor(0.75 * n) + 1)):
                    for k in range(min(n + 1 - i - j, floor(0.75 * n) + 1)):
                        if (n - i - j - k) < (floor(0.75 * n) + 1):
                            self.__states[(i, j, k, n - i - j - k)] = State((i, j, k, n - i - j - k))

            # for i in range(n + 1):
            #     for j in range(n + 1 - i):
            #         for k in range(n + 1 - i - j):
            #             self.__states[(i, j, k, n - i - j - k)] = State((i, j, k, n - i - j - k))

            print('State space generated (' + str(len(self.__states)) + ' states)')

            print('Computing adjacency state of the system...')

            stl = list(self.__states.values())

            for idx1, st1 in enumerate(stl):
                for idx2 in range(idx1):
                    if st1.is_reachable_by(stl[idx2], max_relocation):
                        st1.reachable_by(stl[idx2])
                        stl[idx2].reachable_by(st1)
            self.__zone_n = len(list(self.__states.keys())[0])
        else:
            self.maxRelocation = int(max_relocation)

    def __iter__(self):
        return iter(self.__states.values())

    def get_state(self, st: tuple):
        return self.__states.get(st)

    def get_zone_n(self):
        return self.__zone_n

    # def save(self, filename: str):
    #     with open(filename + '_list.txt', 'w') as f1:
    #         with open(filename + '_neighs.txt', 'w') as f2:
    #             for state in self.__states:
    #                 f1.write(str(state) + '\n')
    #                 f2.write(str(state) + ';')
    #
    #                 for neigh in self.__states[state].reachableBy:
    #                     f2.write(str(neigh) + ';')
    #
    #                 f2.write('\n')

    def save(self, filename: str):
        print('Saving state space to file...')
        state_dict = {val: idx for idx, val in enumerate(self.__states)}

        with open(filename + '_list.txt', 'w') as f1:
            with open(filename + '_neighs.txt', 'w') as f2:
                for state in state_dict:
                    f1.write(str(state_dict[state]) + ' ' + str(state) + '\n')
                    f2.write(str(state_dict[state]) + ' ')

                    for neigh in self.__states[state].reachableBy:
                        f2.write(str(state_dict[tuple(neigh.x)]) + ' ')

                    f2.write('\n')

    def __set_state_list(self, sl: list):
        self.__states = {s: State(s) for s in sl}
        self.__zone_n = len(sl[0])

    # @classmethod
    # def load(cls, filename: str) -> "StateSpace":
    #     ss = cls()
    #     sl = list()
    #
    #     with open(filename + '_list.txt', 'r') as f:
    #         for line in f:
    #             sl.append(str_to_tuple(line.rstrip(' \n,')))
    #
    #     ss.__set_state_list(sl)
    #
    #     with open(filename + '_neighs.txt', 'r') as f:
    #         for line in f:
    #             statestr = line.strip('\n').split(';')
    #             for tup in statestr[1:-1]:
    #                 ss.get_state(str_to_tuple(statestr[0])).reachable_by(ss.get_state(str_to_tuple(tup)))
    #
    #     print('State space loaded (' + str(len(ss)) + ' states)')
    #
    #     return ss

    @classmethod
    def load(cls, filename: str) -> "StateSpace":
        print('Loading state space...')
        max_relocation = filename.split('_')[-1]
        state_list = list()

        with open(filename + '_list.txt', 'r') as f:
            for line in f:
                line = line.split(' ', 1)
                state_list.append(str_to_tuple(line[1].rstrip(' \n,')))

        ss = cls(n=sum(state_list[0]), max_relocation=max_relocation)
        ss.__set_state_list(state_list)

        with open(filename + '_neighs.txt', 'r') as f:
            for line in f:
                statestr = list(map(toint, line.strip('\n').split(' ')[:-1]))
                for s in statestr[1:-1]:
                    ss.get_state(state_list[statestr[0]]).reachable_by(ss.get_state(state_list[s]))

        print('State space loaded (' + str(len(ss)) + ' states)')

        return ss

    def __len__(self):
        return len(self.__states)
