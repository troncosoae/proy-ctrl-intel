import numpy as np

from Simulation.Simulation import SimulationBox


class ExpertPID(SimulationBox):
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

        if e < -1e6:
            x = 0
        elif -1e6 <= e < -1e5:
            x = 1/4
        elif 1e5 < e <= 1e6:
            x = 3/4
        elif 1e6 < e:
            x = 4/4
        else:
            x = 2/4
        beta_r = x*np.pi/2

        print(beta_r)

        return {
            'tau_gr': tau_gr,
            'beta_r': beta_r,
        }


class PaperController(SimulationBox):
    def __init__(self, key, Ts, **kwargs):
        super().__init__(
            key, ['P_r', 'P_g', 'omega_g', 'omega_nom', 'v_W'],
            ['beta_r', 'tau_gr', 'ctrl_mode'])
        self.kp = -0.0015  #
        self.kd = -0.0008  #
        self.Ts = Ts

        self.last_e = 0
        self.last_u = 0  # to add -> beta = 0
        self.control_mode = 1

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

    def control_mode_1(self, omega_g):
        R = self.chars['R']
        ro = self.chars['ro']
        Ng = self.chars['Ng']
        Cp_max = self.chars['Cp_max']
        lmbda_opt = self.chars['lmbda_opt']
        K_opt = 1/2*ro*2*np.pi*R**2*R**3*Cp_max/(lmbda_opt**3)
        tau_gr = K_opt*(omega_g/Ng)**2
        return {
            'beta_r': 0,
            'tau_gr': tau_gr,
            'ctrl_mode': self.control_mode,
        }

    def control_mode_2(self, omega_g, P_r, omega_nom):
        nu_g = self.chars['nu_g']
        e = omega_g - omega_nom
        de = (e - self.last_e)/self.Ts
        du = self.kp*e + self.kd*de
        u = self.last_u + du
        u = np.max([0, np.min([np.pi/2, u])])
        # beta_r = 0.5*(np.arctan(-u) + np.pi/2)
        beta_r = u
        print('de', de, 'e', e, 'du', du, 'u', u)
        print('beta_r', beta_r)
        self.last_e = e
        self.last_u = u
        tau_gr = P_r/(nu_g*omega_g)
        return {
            'beta_r': beta_r,
            'tau_gr': tau_gr,
            'ctrl_mode': self.control_mode,
        }

    def control_mode_3(self, omega_g):
        R = self.chars['R']
        ro = self.chars['ro']
        Ng = self.chars['Ng']
        Cp_max = self.chars['Cp_max']
        lmbda_opt = self.chars['lmbda_opt']
        K_opt = 1/2*ro*2*np.pi*R**2*R**3*Cp_max/(lmbda_opt**3)
        tau_gr = K_opt*(omega_g/Ng)**2
        return {
            'beta_r': np.pi/2,
            'tau_gr': tau_gr,
            'ctrl_mode': self.control_mode,
        }

    def advance(self, input_values):
        super().advance(input_values)
        print(input_values)
        omega_g = input_values['omega_g']
        omega_nom = input_values['omega_nom']
        P_r = input_values['P_r']
        P_g = input_values['P_g']
        v_W = input_values['v_W']

        omega_delta = self.chars['omega_delta']
        P_delta = self.chars['P_delta']

        print('abs e', np.abs(P_g - P_r), 'P_delta', P_delta)
        if np.abs(P_g - P_r) <= P_delta and (
                self.control_mode == 1 or self.control_mode == 3):
            # TODO: que pasa cuando se pasa
            self.control_mode = 2
        elif P_r > P_delta + P_g and (
                self.control_mode == 2 or self.control_mode == 3):
            self.control_mode = 1
            self.last_u = 0  # beta_r = 0
            self.last_e = 0
        elif P_r + P_delta < P_g and (
                self.control_mode == 1 or self.control_mode == 2):
            self.control_mode = 3
            self.last_u = np.pi/2  # beta_r = pi/2
            self.last_e = 0

        if self.control_mode == 1:
            print('control mode 1')
            return self.control_mode_1(omega_g)
        elif self.control_mode == 2:
            print('control mode 2')
            return self.control_mode_2(omega_g, P_r, omega_nom)
        elif self.control_mode == 3:
            print('control mode 3')
            return self.control_mode_3(omega_g)


class TurbineController(SimulationBox):

    def __init__(self, key, kp, ki, kd, Ts):
        SimulationBox.__init__(
            self, key, ['omega_g', 'omega_gr', 'P_r', 'P_g'],
            ['tau_gr', 'beta_r'])

        self.kp0 = kp[0]
        self.ki0 = ki[0]
        self.kd0 = kd[0]

        self.kp1 = kp[1]
        self.ki1 = ki[1]
        self.kd1 = kd[1]

        self.Ts = Ts

        self.last_omega_g_error = 0
        self.last_tau_gr = 0

        self.P_g_error_seq_N = int(1/Ts)
        self.P_g_error_seq = np.zeros(self.P_g_error_seq_N)
        self.last_avg_P_g_error = 0
        self.last_P_g_error = 0
        self.last_beta_r = 0

    def advance(self, input_values):
        super().advance(input_values)

        omega_g_error = input_values['omega_g'] - input_values['omega_gr']
        tau_gr = self.last_tau_gr + self.kp0*omega_g_error - \
            self.kp0*self.last_omega_g_error + self.ki0*self.Ts*omega_g_error

        self.last_tau_gr = tau_gr
        self.last_omega_g_error = omega_g_error

        P_g_error = input_values['P_g'] - input_values['P_r']
        avg_P_g_error = np.mean(self.P_g_error_seq)
        beta_r = self.last_beta_r + self.kp1*avg_P_g_error - \
            self.kp1*self.last_avg_P_g_error + self.ki1*self.Ts*avg_P_g_error

        self.last_P_g_error = P_g_error
        self.last_avg_P_g_error = avg_P_g_error
        self.P_g_error_seq[1:self.P_g_error_seq_N] = self.P_g_error_seq[
            0:self.P_g_error_seq_N-1]
        self.P_g_error_seq[0] = P_g_error
        self.last_beta_r = beta_r

        return {
            'tau_gr': tau_gr,
            'beta_r': np.arctan(beta_r)
        }


class PIDController(SimulationBox):

    def __init__(
            self, key, ref_name, ctrl_v_name, man_v_name, kp, ki, kd, Ts,
            man_v_offset=0):
        SimulationBox.__init__(
            self, key, [ref_name, ctrl_v_name], [man_v_name])
        self.ref_name = ref_name
        self.ctrl_v_name = ctrl_v_name
        self.man_v_name = man_v_name

        self.man_v_offset = man_v_offset

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.int_error = 0
        self.last_error = 0

    def advance(self, input_values):
        super().advance(input_values)
        error = input_values[self.ctrl_v_name] - input_values[self.ref_name]
        self.int_error += error*self.Ts
        der_error = (error - self.last_error)/self.Ts

        # update error values
        self.last_error = error

        print(self.key, error, error*self.kp)

        u = error*self.kp + self.int_error*self.ki + der_error*self.kd
        u += self.man_v_offset
        return {self.man_v_name: u}


class LQRController(SimulationBox):

    def __init__(self, key, inputs_keys, man_v_name, K):
        SimulationBox.__init__(
            self, key, inputs_keys, [man_v_name])
        self.man_v_name = man_v_name

        self.K = K

    def advance(self, input_values):
        super().advance(input_values)

        u = 0
        for k in self.inputs_keys:
            u += self.K[k]*input_values[k]

        return {self.man_v_name: u}


class LQRSubmodelController(SimulationBox):

    def __init__(self, key, inputs_keys, man_v_name, K1, K2, K3):
        SimulationBox.__init__(
            self, key, inputs_keys, [man_v_name])
        self.man_v_name = man_v_name

        self.K1 = K1
        self.K2 = K2
        self.K3 = K3

    def advance(self, input_values):
        super().advance(input_values)

        K = self.K1
        if (np.abs(input_values['theta']) > np.pi/8 and
                np.abs(input_values['theta_dot']) < 1.5):
            # print('K2')
            K = self.K2
        elif (np.abs(input_values['theta']) <= np.pi/8 and
                np.abs(input_values['theta_dot']) >= 1.5):
            K = self.K3
            # print('K3')
        # else:
        #     print('K1')

        u = 0
        for k in self.inputs_keys:
            u += K[k]*input_values[k]

        return {self.man_v_name: u}
