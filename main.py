import numpy as np
import argparse

from Simulation.Simulation import Simulation, get_close_sim_for_box
from System.SystemBoxes import BladePitchSystem, \
    DriveTrainModel, GeneratorConverterModel, \
    RandomWindModel, StepWindModel, ConstantWindModel
from Simulation.PygameBoxes import PlottingTurbineWindow
from System.MeasuringBoxes import PlottingMeasurer
from Control.ControlBoxes import PIDController, Mapper, PIDVarController


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--csv_path', type=str)
    args = parser.parse_args()

    Ts = 0.001
    pygame_fs = 1/Ts

    sim = Simulation()

    # wind_model = RandomWindModel('wind_model', Ts)
    # wind_model = StepWindModel('wind_model', Ts, 1)
    # wind_model = StepWindModel('wind_model', Ts, 5, mean=5, std=3)
    # wind_model = StepWindModel('wind_model', Ts, 5, mean=15, std=3)
    wind_model = ConstantWindModel('wind_model', 8)
    # wind_model = ConstantWindModel('wind_model', 15)
    # wind_model = ConstantWindModel('wind_model', 0.001)
    blade_pitch_system = BladePitchSystem('bp_sys', Ts)
    drive_train_model = DriveTrainModel(
        'dt_model', Ts)
    generator_converter_model = GeneratorConverterModel('gc_model', Ts)
    measurer = PlottingMeasurer(
        'meas',
        [
            'v_W', 'beta_m', 'beta_r', 'omega_r',
            'omega_g', 'P_g', 'tau_g', 'tau_r', 'tau_gr',
            'theta_d',
            # 'P_r',
            'omega_nom',
            'x',
        ],
        Ts)
    pygame_tracker = PlottingTurbineWindow(
        'pygame', {
            'omega_g': (255, 255, 0),
            'omega_nom': (255, 255, 100),
            'omega_r': (0, 255, 255),
            'beta_r': (255, 0, 0),
            # 'P_g': (0, 255, 0),
            # 'P_r': (100, 255, 100),
            'v_W': (0, 0, 255),
        },
        -0.5, 1.5, pygame_fs, get_close_sim_for_box(sim))
    pid_controller = PIDController(
        'pid', 'omega_nom', 'omega_g', 'x', -20, 0, 0.1, Ts)
    pid_mapper = Mapper(
        'map_pid', ['x'], ['beta_r'],
        {'x': lambda x: 0.5*np.arctan(-x*0.2) + np.pi/4},
        {'x': 'beta_r'}
    )

    sim.add_box(wind_model)
    sim.add_box(
        blade_pitch_system, {'beta_r': np.pi/2, 'omega_r': 0})
    sim.add_box(drive_train_model, {'tau_g': 10})
    sim.add_box(generator_converter_model, {'tau_gr': 10})
    sim.add_box(pid_controller, {'omega_nom': 14})
    sim.add_box(pid_mapper)

    sim.add_box(measurer)
    sim.add_box(pygame_tracker)

    sim.run()

    pygame_tracker.quit_pygame()

    measurer.plot_values({
        'tau_g', 'P_g', 'beta_m', 'v_W', 'omega_r', 'omega_g',
        'beta_r',
        'omega_nom',
    })
    measurer.plot_values({
        'omega_nom', 'omega_g',
    })
    measurer.plot_values({
        'beta_r', 'beta_m',
    })
