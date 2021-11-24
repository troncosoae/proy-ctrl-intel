import numpy as np

from Simulation.Simulation import SimulationBox


class Mapper(SimulationBox):
    def __init__(
            self, key, inputs_keys, outputs_keys, mapper_functions,
            mapper_keys):
        super().__init__(key, inputs_keys, outputs_keys)
        self.mapper_functions = mapper_functions
        self.mapper_keys = mapper_keys

    def advance(self, input_values):
        super().advance(input_values)
        output = {
            self.mapper_keys[key]: self.mapper_functions[key](
                input_values[key])
            for key in self.inputs_keys
        }
        print(output)
        return output


class PIDVarController(SimulationBox):
    def __init__(
            self, key, ref_name, ctrl_v_name, man_v_name, kp, ki, kd, Ts):
        SimulationBox.__init__(
            self, key, [ref_name, ctrl_v_name], [man_v_name])
        self.ref_name = ref_name
        self.ctrl_v_name = ctrl_v_name
        self.man_v_name = man_v_name

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.int_error = 0
        self.last_error = 0
        self.last_u = 0

    def advance(self, input_values):
        super().advance(input_values)
        error = input_values[self.ctrl_v_name] - input_values[self.ref_name]
        self.int_error += error*self.Ts
        der_error = (error - self.last_error)/self.Ts

        # update error values
        self.last_error = error

        du = error*self.kp + self.int_error*self.ki + der_error*self.kd
        u = self.last_u + du
        self.last_u = u
        return {self.man_v_name: u}


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

        # print(self.key, error, error*self.kp)

        u = error*self.kp + self.int_error*self.ki + der_error*self.kd
        u += self.man_v_offset
        print('u', u)
        return {self.man_v_name: u}
