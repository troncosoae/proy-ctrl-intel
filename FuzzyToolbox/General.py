import numpy as np


def ramp_function_generator(a, b):
    def ramp_function(x):
        return np.max([
            np.min([
                (1 - 0)/(b - a)*(x - a) + 0,
                1
            ]),
            0
        ])
    return ramp_function


def trapezoid_function_generator(a, b, c, d):
    def trapezoid_function(x):
        return np.max([
            np.min([
                (1 - 0)/(b - a)*(x - a) + 0,
                1,
                (0 - 1)/(d - c)*(x - c) + 1
            ]),
            0
        ])
    return trapezoid_function


def inv_trapezoid_function_generator(a, b, c, d):
    def inv_trapezoid_function(x):
        return np.min([
            np.max([
                (0 - 1)/(b - a)*(x - a) + 1,
                0,
                (1 - 0)/(d - c)*(x - c) + 0
            ]),
            1
        ])
    return inv_trapezoid_function


def triangle_function_generator(a, b, c):
    def triangle_function(x):
        return np.max([
            np.min([
                (1 - 0)/(b - a)*(x - a) + 0,
                1,
                (0 - 1)/(c - b)*(x - b) + 1
            ]),
            0
        ])
    return triangle_function


if __name__ == '__main__':
    func = ramp_function_generator(2, 1)
    print(func(0))
    print(func(1.5))
    print(func(2.5))
    func = triangle_function_generator(1, 2, 3)
    print(func(0.5))
    print(func(1.5))
    print(func(2))
    print(func(2.5))
    print(func(3.5))
