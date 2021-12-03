import numpy as np

from Simulation.Simulation import SimulationBox
from FuzzyToolbox.General import *


class FuzzyPID(SimulationBox):
    def __init__(self, key, **kwargs):
        super().__init__(
            key, ['P_r', 'P_g', 'omega_g', 'omega_nom', 'v_W'],
            ['beta_r', 'tau_gr']
        )

        self.chars = {
            'Ng': kwargs.get('Ng', 95),
            'ro': kwargs.get('ro', 1.225),  # ro
            'R': kwargs.get('R', 57.5),  # R (m)
            'Cp_max': kwargs.get('Cp_max', 0.6338),
            'lmbda_opt': kwargs.get('lmbda_opt', 12.12),
            'nu_g': kwargs.get('nu_g', 0.98),  # 0.98
            'omega_delta': kwargs.get('omega_delta', 15),
            'P_delta': kwargs.get('P_delta', 0.5e5),
        }

        self.fuzzy_sets = {
            's1': {
                'func': ramp_function_generator(-1e5, -1e6),
                'val': 0/4*np.pi/2},
            's2': {
                'func': triangle_function_generator(-1e6, -1e3, 0),
                'val': 1/4*np.pi/2},
            's3': {
                'func': triangle_function_generator(-1e5, 0, 1e5),
                'val': 2/4*np.pi/2},
            's4': {
                'func': triangle_function_generator(0, 1e3, 1e6),
                'val': 3/4*np.pi/2},
            's5': {
                'func': ramp_function_generator(1e5, 1e6),
                'val': 4/4*np.pi/2},
        }

        self.last_e = 0

    def advance(self, input_values):
        super().advance(input_values)

        R = self.chars['R']
        ro = self.chars['ro']
        Ng = self.chars['Ng']
        Cp_max = self.chars['Cp_max']
        lmbda_opt = self.chars['lmbda_opt']

        omega_g = input_values['omega_g']
        omega_nom = input_values['omega_nom']
        P_r = input_values['P_r']
        P_g = input_values['P_g']
        v_W = input_values['v_W']

        K_opt = 1/2*ro*2*np.pi*R**2*R**3*Cp_max/(lmbda_opt**3)
        tau_gr = K_opt*(omega_g/Ng)**2

        e = P_g - P_r
        de = e - self.last_e
        self.last_e = e

        results = {}
        sum_restuls = 0
        sum_pond_vals = 0
        for fs_key in self.fuzzy_sets:
            fs = self.fuzzy_sets[fs_key]
            res = fs['func'](e)
            results[fs_key] = {
                'res': res
            }
            sum_restuls += res
            sum_pond_vals += res*fs['val']
        beta_r = sum_pond_vals/sum_restuls

        print(results)

        # if e < -1e6:
        #     x = 0
        # elif -1e6 <= e < -1e5:
        #     x = 1/4
        # elif 1e5 < e <= 1e6:
        #     x = 3/4
        # elif 1e6 < e:
        #     x = 4/4
        # else:
        #     x = 2/4
        # beta_r = x*np.pi/2

        print(beta_r)

        return {
            'tau_gr': tau_gr,
            'beta_r': beta_r,
        }
