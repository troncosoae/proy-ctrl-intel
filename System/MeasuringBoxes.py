import matplotlib.pyplot as plt
import pandas as pd

from Simulation.Simulation import SimulationBox


class BasicMeasurer(SimulationBox):
    def __init__(self, key, inputs_keys):
        SimulationBox.__init__(self, key, inputs_keys, [])

    def advance(self, input_values):
        super().advance(input_values)
        # print(input_values)
        return {}


class PlottingMeasurer(BasicMeasurer):
    def __init__(self, key, inputs_keys, Ts):
        BasicMeasurer.__init__(self, key, inputs_keys)
        self.Ts = Ts
        self.t = []
        self.historical_values = {}

    def advance(self, input_values):
        super().advance(input_values)
        if len(self.t) == 0:
            self.t.append(0)
        else:
            self.t.append(self.t[-1] + self.Ts)
        print(self.t[-1] + self.Ts, input_values)
        for key in self.inputs_keys:
            value = input_values[key]
            if key not in self.historical_values:
                self.historical_values[key] = [value]
            else:
                self.historical_values[key].append(value)
        return {}

    def plot_values(self, keys=set(), exclude=set()):
        if len(keys) == 0:
            keys = set(self.historical_values.keys())
        for key in exclude:
            keys.remove(key)

        for key in keys:
            plt.plot(self.t, self.historical_values[key], label=key)
        plt.xlabel('t')
        plt.legend()
        plt.show()

    def export_values(self, path, keys=set(), exclude=set()):
        if len(keys) == 0:
            keys = set(self.historical_values.keys())
        for key in exclude:
            keys.remove(key)

        data = {}
        for key in keys:
            data[key] = self.historical_values[key]
        data['t'] = self.t

        data_df = pd.DataFrame(data)

        data_df.to_csv(path)


class IndexTracker(SimulationBox):
    def __init__(self, key, Ts):
        inputs_keys = ['P_g', 'P_r']
        outputs_keys = ['J1']
        super().__init__(key, inputs_keys, outputs_keys)

        self.Ts = Ts
        self.j1 = 0

    def advance(self, input_values):
        super().advance(input_values)
        P_g = input_values['P_g']
        P_r = input_values['P_r']

        self.j1 += self.Ts*(P_r - P_g)**2

        return {
            'J1': self.j1
        }
