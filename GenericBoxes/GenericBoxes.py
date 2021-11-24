from Simulation.Simulation import SimulationBox


class Derivative(SimulationBox):
    def __init__(self, key, inputs_keys, Ts):
        outputs_keys = ['der__' + key for key in inputs_keys]
        print(inputs_keys, outputs_keys)
        super().__init__(key, inputs_keys, outputs_keys)
        self.Ts = Ts
        self.prev_inputs = {
            key: 0 for key in inputs_keys
        }

    def advance(self, input_values):
        super().advance(input_values)
        outputs = {
            'der__' + key: (input_values[key] - self.prev_inputs[key])/self.Ts
            for key in self.inputs_keys
        }
        print(outputs)
        self.prev_inputs = input_values
        return outputs
