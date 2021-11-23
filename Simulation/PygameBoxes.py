from matplotlib import pyplot
import numpy as np
import pygame
from pygame import gfxdraw
from pygame.transform import scale

from Simulation.Simulation import SimulationBox


class TurbineWindow(SimulationBox):
    def __init__(self, key, inputs_keys, fs, close_function, **kwargs):
        SimulationBox.__init__(self, key, inputs_keys, [])
        pygame.init()
        self.fs = fs
        self.clock = pygame.time.Clock()
        self.width = kwargs.get('width', 1000)
        self.height = kwargs.get('height', 600)
        self.window = pygame.display.set_mode((self.width, self.height))
        self.close_function = close_function

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print('Q')
                self.close()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print('esc')
                    self.close()
                if event.key == pygame.K_r:
                    print('R')
                if event.key == pygame.K_q:
                    print('Q')

    def refresh_window(self):
        pygame.display.update()

    def advance(self, input_values):
        super().advance(input_values)
        self.clock.tick(self.fs)
        self.handle_events()
        self.refresh_window()
        return {}

    def close(self):
        self.close_function()

    def quit_pygame(self):
        pygame.quit()


class PlottingTurbineWindow(TurbineWindow):
    def __init__(
            self, key, inputs_color_dict, min, max, fs, close_function,
            **kwargs):
        self.inputs_color_dict = inputs_color_dict
        inputs_keys = list(inputs_color_dict.keys())
        TurbineWindow.__init__(
            self, key, inputs_keys, fs, close_function, **kwargs)
        self.values = {}
        self.min = min
        self.max = max

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print('Q')
                self.close()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print('esc')
                    self.close()
                if event.key == pygame.K_r:
                    print('R')
                if event.key == pygame.K_q:
                    print('Q')
                if event.key == pygame.K_p:
                    print('P')
                    input()

    def advance(self, input_values):
        for key in input_values:
            if key in self.inputs_keys:
                self.values[key] = input_values[key]
        return super().advance(input_values)

    def get_positions(self):
        positions = {
            key: int(self.height - self.height/(self.min - self.max)*self.min +
                     self.values[key]*self.height/(self.min - self.max))
            for key in self.values}

        return positions

    def refresh_window(self):
        positions = self.get_positions()
        self.window.scroll(dx=-1)
        pygame.draw.lines(
            self.window, (0, 0, 0), False,
            [(self.width-1, 0), (self.width-1, self.height)], 1
        )
        for key in positions:
            if 0 <= positions[key] < self.height:
                gfxdraw.pixel(
                    self.window, self.width-1, positions[key],
                    self.inputs_color_dict[key])

        return super().refresh_window()


class PendulumWindow(SimulationBox):
    def __init__(self, key, inputs_keys, fs, close_function, **kwargs):
        SimulationBox.__init__(self, key, inputs_keys, [])

        pygame.init()
        self.clock = pygame.time.Clock()
        self.fs = fs
        self.pixel_m_ratio = 100
        self.width = kwargs.get('width', 1000)
        self.height = kwargs.get('height', 600)
        self.window = pygame.display.set_mode((self.width, self.height))
        self.close_function = close_function

    def xy_coordinates(self, x, theta):
        # print(f't:{theta:.2f}\tx:{x:.2f}')
        y_pend2car = np.cos(theta)*0.67
        x_pend2car = np.sin(theta)*0.67
        x_car = x
        y_car = 0
        x_pend = x_pend2car + x_car
        y_pend = y_pend2car + y_car
        return x_pend, y_pend, x_car, y_car

    def map_xy2window(self, x_pend, y_pend, x_car, y_car):
        w = self.width
        h = self.height
        return (
            (x_pend*self.pixel_m_ratio + w/2) % w,
            h/2 - y_pend*self.pixel_m_ratio,
            (x_car*self.pixel_m_ratio + w/2) % w,
            h/2 - y_car*self.pixel_m_ratio
        )

    def refresh_window(self, x, theta):
        x_pend, y_pend, x_car, y_car = self.xy_coordinates(x, theta)
        x_pend, y_pend, x_car, y_car = self.map_xy2window(
            x_pend, y_pend, x_car, y_car)

        self.window.fill((0, 0, 0))

        pygame.draw.lines(
            self.window, (10, 255, 255), False,
            [(x_pend, y_pend), (x_car, y_car)], 2
        )
        pygame.draw.lines(
            self.window, (255, 255, 255), False,
            [(self.width/2, 0), (self.width/2, self.height)], 1
        )
        pygame.draw.circle(
            self.window, (255, 10, 255),
            (x_pend, y_pend), 7
        )
        pygame.draw.circle(
            self.window, (255, 255, 10),
            (x_car, y_car), 10
        )

        pygame.display.update()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print('Q')
                self.close()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print('esc')
                    self.close()
                if event.key == pygame.K_r:
                    print('R')
                if event.key == pygame.K_q:
                    print('Q')

    def advance(self, input_values):
        super().advance(input_values)
        self.clock.tick(self.fs)
        self.handle_events()
        self.refresh_window(input_values['x'], input_values['theta'])
        # print(input_values)
        return {}

    def close(self):
        self.close_function()

    def quit_pygame(self):
        pygame.quit()
