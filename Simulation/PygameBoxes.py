from matplotlib import pyplot
import numpy as np
import pygame
from pygame import gfxdraw

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
        self.surface = pygame.Surface((self.width, self.height))
        self.window.blit(self.surface, (0, 0))
        pygame.display.update()
        self.values = {}
        self.min = min
        self.max = max
        self.last_scale = 1

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

        print(self.values)
        arr_vals = [self.values[key] for key in self.values]
        print(arr_vals)
        np_vals = np.asarray(arr_vals, dtype=np.float32)
        print(np_vals)
        print(np.max(np_vals), np.min(np_vals))
        print(np.max([np.max(np_vals), -1*np.min(np_vals), 10]))
        abs_max = np.max([np.max(np_vals), -1*np.min(np_vals), 1])
        scale = (self.height)/(2*abs_max)
        print(scale)

        # scale positions
        positions = {
            key: int(self.height/2 - self.values[key]*scale)
            for key in self.values}

        print(positions)
        new_surface_size = (self.width, int(self.height*scale/self.last_scale))
        self.last_scale = scale
        offset = int((self.height - new_surface_size[1])/2)

        print(new_surface_size, offset)

        return positions, new_surface_size, offset

    def refresh_window(self):
        positions, new_surface_size, offset = self.get_positions()

        self.surface.scroll(dx=-1)
        pygame.draw.lines(
            self.surface, (0, 0, 0), False,
            [(self.width-1, 0), (self.width-1, self.height)], 1
        )
        gfxdraw.pixel(
            self.surface, self.width-1, int(self.height/2),
            (255, 255, 255))
        for key in positions:
            if 0 <= positions[key] < self.height:
                gfxdraw.pixel(
                    self.surface, self.width-1, positions[key],
                    self.inputs_color_dict[key])

        new_surface = pygame.Surface(new_surface_size)
        pygame.transform.scale(self.surface, new_surface_size, new_surface)

        self.surface.fill((0, 0, 0))
        self.surface.blit(new_surface, (0, offset))
        self.window.fill((0, 0, 0))
        self.window.blit(self.surface, (0, 0))

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
