#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change camera position

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function
from pygame import mixer

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
from carla import ColorConverter as cc

import argparse
import os
import sys
import time
import collections
import datetime
import logging
import math
import weakref
import csv
import re
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, PageBreak
from reportlab.lib import colors
import datetime
import time

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_w
    from pygame.locals import K_a
    from pygame.locals import K_s
    from pygame.locals import K_d
    from pygame.locals import K_q
    from pygame.locals import K_m
    from pygame.locals import K_COMMA
    from pygame.locals import K_PERIOD
    from pygame.locals import K_p
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_r
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
    from pygame.locals import K_h
    from pygame.locals import K_SLASH
    from pygame.locals import K_c
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_0
    from pygame.locals import K_9


except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


dataFile = open("data.csv",'w',newline='') ###create and open a csv file
rowsHeader = ['x','y','accelaration x','accelaration y','accelaration z','speed','object ahead','distance to object ahead','object behind','distance to object behind','collision','collision with','blinkers'] #title of headers
writer = csv.writer(dataFile) 
writer.writerow(rowsHeader)#write headers in the first row

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):

    restarted = False

    def __init__(self, carla_world, hud, args, display_manager):
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.display_manager = display_manager #display manager 
        self.radar_sensor = None
        self.camera_manager = None
        self.obstacle_detector = None
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.obstacle_detector_behind = None
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self):

        if self.restarted:
            return
        self.restarted = True

        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get the ego vehicle
        while self.player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    self.player = vehicle
                    break
        
        self.player_name = self.player.type_id

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        #self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma,self.display_manager) #display manager

        self.obstacle_detector = SafetyDistance(self.player)
        self.obstacle_detector_behind = SafetyDistanceBehind(self.player)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        self.world.wait_for_tick()

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False

        self.hud.tick(self, clock)
        return True

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor,
            self.obstacle_detector.sensor,
            self.obstacle_detector_behind.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            
            self.player.destroy()

        #evaluations.generateReport()

        dataFile.close()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE
        self._steer_cache = 0.0
        world.player.set_autopilot(self._autopilot_enabled)
        world.player.set_light_state(self._lights)
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self._control.gear = world.player.get_control().gear
                    world.hud.notification('%s Transmission' %
                                            ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification(
                        'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    current_lights ^= carla.VehicleLightState.Special1
                elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    current_lights ^= carla.VehicleLightState.HighBeam
                elif event.key == K_l:
                    # Use 'L' key to switch between lights:
                    # closed -> position -> low beam -> fog
                    if not self._lights & carla.VehicleLightState.Position:
                        world.hud.notification("Position lights")
                        current_lights |= carla.VehicleLightState.Position
                    else:
                        world.hud.notification("Low beam lights")
                        current_lights |= carla.VehicleLightState.LowBeam
                    if self._lights & carla.VehicleLightState.LowBeam:
                        world.hud.notification("Fog lights")
                        current_lights |= carla.VehicleLightState.Fog
                    if self._lights & carla.VehicleLightState.Fog:
                        world.hud.notification("Lights off")
                        current_lights ^= carla.VehicleLightState.Position
                        current_lights ^= carla.VehicleLightState.LowBeam
                        current_lights ^= carla.VehicleLightState.Fog
                elif event.key == K_i:
                    current_lights ^= carla.VehicleLightState.Interior
                elif event.key == K_z:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                elif event.key == K_x:
                    current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0
            # Set automatic control-related vehicle lights
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else: # Remove the Brake flag
                current_lights &= ~carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else: # Remove the Reverse flag
                current_lights &= ~carla.VehicleLightState.Reverse
            if current_lights != self._lights: # Change the light state only if necessary
                self._lights = current_lights
                world.player.set_light_state(carla.VehicleLightState(self._lights))
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
       
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))

    def parse_events(self, world, clock):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                #elif event.button == 1: #square
                 #    world.hud.toggle_info()
                #elif event.button == 2: #circle
                #    world.camera_manager.toggle_camera()
                #elif event.button == 3:
                 #   world.next_weather()
                # elif event.button == 3: #triangle
                #     print('3')
                #     world.camera_manager.toggle_camera()

                # elif event.button == self._reverse_idx:
                #     self._control.gear = 1 if self._control.reverse else -1
                #     print('reverse')

                # elif event.button == 23: #red button press
                #     print('23')
                #     world.camera_manager.next_sensor()

                # elif event.button == 4: #right pedal shifter
                #     print('4')
                
                elif event.button == 5:
                    print('5')

                # elif event.button == 6: #R2
                #     print('6')

                # elif event.button == 7:#L2
                #     print('7')

                # elif event.button == 8: #share
                #     print('8')
                
                # elif event.button == 9:#option
                #     print('9')

                # elif event.button == 11:#L3
                #     print('11')

                # elif event.button == 12:
                #     print('12')

                # elif event.button == 13:
                #     print('13')

                elif event.button == 14: #make this drive
                     self._control.gear = 1 
                     print('Drive')

                elif ((event.button == 15) and (self._reverse_idx)):#make this reverse (gearbox)
                    self._control.gear = -1 #if self._control.reverse else -1
                    print('Reverse')

                # elif event.button == 16:
                #     print('16')

                elif event.button == 17: #making this as parking
                    self._control.gear = 0
                    print('Parked')
                
                # elif event.button == 18:
                #     print('18')

                # elif event.button == 19: #+
                #     print('19')

                # elif event.button == 20: #-
                #     print('20')

                # elif event.button == 21: #red button CW
                #     print('21')

                # elif event.button == 22: #red button ACW
                #     print('22')

                # elif event.button == 24: 
                #     print('24')    
                

                # ==============================================================================
                # -- Blinkers Implemented ---------------------------------------------------------
                # ==============================================================================
                if isinstance(self._control, carla.VehicleControl):
                    if event.button == 5: 
                        # if light_state in self.left_light_states:
                        #     current_lights ^= carla.VehicleLightState.RightBlinker
                        print('left blinker')
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.button == 4: 
                        print('right blinker')
                        current_lights ^= carla.VehicleLightState.RightBlinker

                # ==============================================================================
                # ==============================================================================

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()

                

                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                 # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]
        
        # print([float(self._joystick.get_button(i)) for i in
        #              range(self._joystick.get_numbuttons())])

        
        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 0.6 # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.4  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        
        self.speedList = []
        self.accel = []
        self.colls = []
        self.locs = []
        self.last_sec = 0
        self.last_actor = None
        self.last_coll = None
        self.last_dist = 0.0
        self.last_actor_behind = None
        self.last_dist_behind = 0.0
        ''' left blinker states:
            leftblinker 33 35 163
            reverse:
            96,97,99,227

            high beam:
            36, 37, 39, 167

            high beam + reverse:
            100, 101, 103, 231 
        '''
        
        ''' right blinker states:
            rightblinker 17 19 147
            reverse:
            80, 81, 83, 211

            high beam:
            20, 21, 23, 151

            high beam + reverse:
            84, 85, 87, 215, 
        '''
        self.left_light_states = [carla.VehicleLightState.LeftBlinker, 33, 35, 163, 96, 97, 99, 227, 36, 37, 39, 167, 100, 101, 103, 231]
        self.right_light_states = [carla.VehicleLightState.RightBlinker, 17, 19, 147, 80, 81, 83, 211,  20, 21, 23, 151, 84, 85, 87, 215]
        self.count = 0
        self.last_time = -1

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        a =  world.imu_sensor.accelerometer
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        acceleration = world.imu_sensor.accelerometer

        speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        acceleration = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
        objectname = None
        objectdist = 0.0
        valueList = []
        blinker = "No Blinker"
       

        #We will ignore roads and roadlines
        if ((self.last_actor == world.obstacle_detector.other_actor and self.last_dist == world.obstacle_detector.distance) 
            or world.obstacle_detector.other_actor == 'static.road' or world.obstacle_detector.other_actor == 'static.roadLine'):
            world.obstacle_detector.distance = 0.0
            world.obstacle_detector.other_actor = 'None'
        else:
            self.last_actor = world.obstacle_detector.other_actor
            self.last_dist = world.obstacle_detector.distance

        #logic to get objects infront of car + distance of object from car
        if  world.obstacle_detector.other_actor is not None:
            name = re.sub(r'^[^.]*\.', '', world.obstacle_detector.other_actor)
            objectname = name
            objectdist = round(world.obstacle_detector.distance,2)
            #print(round(world.obstacle_detector.distance,2), name)
        else:
            objectname = world.obstacle_detector.other_actor
            objectdist = round(world.obstacle_detector.distance,2)
            #print(round(world.obstacle_detector.distance,2), world.obstacle_detector.other_actor)

        #We will ignore roads and roadlines
        if ((self.last_actor_behind == world.obstacle_detector_behind.other_actor and self.last_dist_behind == world.obstacle_detector_behind.distance) 
            or world.obstacle_detector_behind.other_actor == 'static.road' or world.obstacle_detector_behind.other_actor == 'static.roadLine'):
            world.obstacle_detector_behind.distance = 0.0
            world.obstacle_detector_behind.other_actor = 'None'
        else:
            self.last_actor_behind = world.obstacle_detector_behind.other_actor
            self.last_dist_behind = world.obstacle_detector_behind.distance

        #logic to get objects infront of car + distance of object from car
        if  world.obstacle_detector_behind.other_actor is not None:
            name_behind = re.sub(r'^[^.]*\.', '', world.obstacle_detector_behind.other_actor)
            objectname_behind = name_behind
            objectdist_behind = round(world.obstacle_detector_behind.distance,2)
            #print(round(world.obstacle_detector.distance,2), name)
        else:
            objectname_behind = world.obstacle_detector_behind.other_actor
            objectdist_behind = round(world.obstacle_detector_behind.distance,2)
            #print(round(world.obstacle_detector.distance,2), world.obstacle_detector.other_actor)

        #determinig collision(s)
        col = 0
        coll_name = world.collision_sensor._other
        if (self.last_coll == coll_name): 
            col = 0
            coll_name = 'No Collision'
        elif (coll_name == 'No Collision'):
            col = 0
        else:
            col = 1
        self.last_coll = coll_name

################################################ Printing out info in a list.###########################################################################################


        if ((int(self.simulation_time) - self.last_sec) == 1):
                ls = world.player.get_light_state()
                if ls in self.left_light_states:
                    blinker = "Left Blinker"
                elif ls in self.right_light_states:
                    blinker = "Right Blinker"
                myList = [round(t.location.x,2), round(t.location.y,2), a[0],a[1],a[2], round(speed,2),objectname,objectdist,objectname_behind,objectdist_behind,col,coll_name,blinker]
                #print(myList)     
                writer.writerow(myList)

        self.last_sec = int(self.simulation_time)

        if ((int(self.simulation_time) - self.last_time) == 1):
                    if (round(speed)==0 and world.player.get_control().hand_brake):
                        self.count = self.count + 1
                    else:
                        self.count = 0

        # evaluations.speedTest(world.player,self)
        # evaluations.parkTest(world.player, self.count, self)
        # # evaluations.collisions( world.collision_sensor,collision)
        # evaluations.instructortest(world.player, self)
        # evaluations.sidewalk_detection(world.player,self)

        self.last_time = int(self.simulation_time)



#################################################### Change Lane check function    ###########################################################
        def change_lane_check(pos_x,pos_y):
            distance_y = 90
            if abs(pos_y)-distance_y <=6: #check if its less than 6 distance
                mixer.init()    
                mixer.music.load(r"C:\Users\b00083281\Desktop\PythonAPI\scenario_runner-0.9.13\srunner\scenariomanager\scenarioatomics\Audio_Files\lane_change.mp3")  # Loading the song    
                mixer.music.set_volume(0.7)  # Setting the volume     
                mixer.music.play() # Start playing the song
                #print("hello, change lane")


        
 ################################################### Checking for lane change condition, for town 3 #####################################################

        if((t.location.x > -90 and t.location.x<-85) and (t.location.y >-100 and t.location.y <-75)):
            change_lane_check(t.location.x,t.location.y)
            # if instruction_test() == False:
            #     fail_instructor = True


##########################################################Checking for lane change condition, for town 5################################################
        if((t.location.x > 30 and t.location.x<40) and (t.location.y >49 and t.location.y <62)):
            change_lane_check(t.location.x,t.location.y)

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % speed,
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % a,
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        self._info_text += [
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            ('Manual:', c.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)
# ==============================================================================
# --Evaluation Metrics----------------------------------------------------------
# ==============================================================================

class Evaluations(object):


    def __init__(self):
        self.left_light_states = [carla.VehicleLightState.LeftBlinker, 33, 35, 163, 96, 97, 99, 227, 36, 37, 39, 167, 100, 101, 103, 231]
        self.right_light_states = [carla.VehicleLightState.RightBlinker, 17, 19, 147, 80, 81, 83, 211,  20, 21, 23, 151, 84, 85, 87, 215]   
        self.no_blinker__fail = False
        self.wrong_blinker_fail = False
        self.collision_fail= False
        self.instructor_fail = False
        self.time_fail = False
        self.result = 'pass'
        self.start_time  = datetime.datetime.now().replace(microsecond=0)
        self.sidewalk_fail= False
        #----------------------------------
        self.number_of_speed_exceeded = 0
        self.number_of_col = 0
        self.number_of_blinkers_missed = 0
        self.number_of_wrong_blinkers = 0
        self.number_of_inst_violation = 0
        self.number_of_sidewalk_hit = 0
        self.num_of_duration_exceeded=0
        #----------------------------------
        self.over_speeding_advice = "No Comment Here"
        self.blinkers_advice = "No Comment Here"
        self.timelimit_advice = "No Comment Here"
        self.collision_advice = "No Comment Here"
        self.instviolation_advice = "No Comment Here"
        self.sidewalk_advice = "No Comment Here"
        self.wrong_blinkers_advice = "No Comment Here"


        #----------------------------------
        self.speed_flag = False
    
    def speedTest(self, car, hud):
        v = car.get_velocity()
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),2)
        if (speed > 30):
            self.speed_fail = True
            hud.notification('Slow Down!')
            if (self.speed_flag == False):
                self.number_of_speed_exceeded += 1
                self.speed_flag = True
        else:
            self.speed_flag = False

            # print('Over Speed!')


    # def parkTest(self, car, count, hud):
    #     v = car.get_velocity()
    #     speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),2)
    #     t = car.get_transform()
    #     if(((t.location.x >185 and t.location.x <187.5) and (t.location.y > 21 and t.location.y < 24)) and (t.rotation.yaw > 85 and t.rotation.yaw < 95) and count>3):
            
    #         hud.notification('You Have Parked Successfully')
    #         time.sleep(5)
    #         self.generateReport() 
    #         pygame.quit()
    #         print ('Parked')
    #     #else:
    #         #print('Not parked')

    # def collisions(self,impulse):
    #     time_current = time.time()
    #     if(self.last_time == None):
    #         self.number_of_col=self.number_of_col+1
    #     elif(time_current - self.last_time >0.4 and impulse > 5):
    #         self.number_of_col=self.number_of_col+1
    #     self.last_time = time_current

    #     self.collision_fail =True
        
    def check_blinker(self,car,event):
        # print("INSIDE Blinker")
        light_state = car.get_light_state()
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        
        if(text[0]=="'Solid'"): #checks for if the line is "Solid". True is Pass, False is Fail
            if light_state in self.left_light_states: #invading line with left blinker
                print('Invade with LB')
                self.wrong_blinker_fail= True
                self.number_of_wrong_blinkers =+ 1
                return False

            elif light_state in self.right_light_states: #invading line with right blinker
                print('Invade lane RB')
                self.wrong_blinker_fail= False
                return True
            
            else: #invading line with no blinker
                print('Invade lane NB')
                self.no_blinker__fail = True
                self.number_of_blinkers_missed =+ 1
                return False
    
    # def sidewalk_detection(self, car,hud):
    #     t = car.get_transform()
    #     if ((t.location.x >= 183 and (t.location.x < 184) )and (t.location.y >-77 and t.location.y <99)):
    #         hud.notification('You have hit the sidewalk')
    #         self.number_of_sidewalk_hit += 1
    #         print ('sidewalk')

    # def instructortest(self, car, hud):
    #     v = car.get_velocity()
    #     speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),2)
    #     t = car.get_transform()
    #     if (t.location.x >185 and t.location.x <210) and (t.location.y >40 and t.location.y <72):
    #         self.instructor_fail = True #flags if the the driver goes beyond the designated parking instruction
    #         print("Instruction violated")
    #         hud.notification('Instruction violated')
    #         self.number_of_inst_violation += 1
    #         self.generateReport()
    #         # time.sleep(7)
    #         pygame.quit()
    
    def time_fail(self, duration):
        
        if (duration > 300): #300
            print("time exceeded of 5 minutes")
            self.time_fail = True
            self.num_of_duration_exceeded = 1
        

    def generateReport(self):
        # # Get today's date, start time, and end time from the OS system
        today = datetime.date.today().strftime('%Y-%m-%d')
        end_time = datetime.datetime.now().replace(microsecond=0)

        duration = datetime.timedelta(hours=(end_time.hour - self.start_time.hour),minutes=(end_time.minute - self.start_time.minute),seconds=(end_time.second - self.start_time.second))
        duration_seconds = int(duration.total_seconds())

        Evaluations.time_fail(self, duration_seconds)

        
        cartype = "Dodge Charger"

       
        # Define other data for the table 1
        num_faults = 0
        immediate_fail = 'No'

        #################################### fail-variables#################################
        blinker_test = "No"
        wrong_blinker_test = "No"
        instructions_test = "No"
        Overspeeding_test= "No"
        collisons_test = "No"
        time_test = "No"
        sidewalk_test = "No"


        if(self.no_blinker__fail == True):
            blinker_test = "yes"
            if (blinker_test == "yes"):
                self.blinkers_advice = "Based on your driving data, it appears \n that you could benefit from using your \n blinkers more often. Using your blinkers\n can help other drivers anticipate your \nmovements and improve overall safety on the\n road. We recommend getting into the habit of \nusing your blinkers well in advance of making \nany turns or changing lanes. Remember, proper \nuse of blinkers is not only important for \nyour own safety, but also for the safety \nof others around you."
            else:
                self.blinkers_advice = "No Comment Here"
            self.no_blinker__fail = False

        if(self.wrong_blinker_fail == True): 
            wrong_blinker_test = "yes"
            if (wrong_blinker_test == "yes"):
                self.wrong_blinkers_advice = "Based on your driving data, it appears\n that you may have used the wrong blinkers\n while making turns or changing lanes. We \nrecommend reviewing the rules of the road and \npracticing using the appropriate blinkers to \nensure that you're signaling your intentions \ncorrectly. "
            else:
                self.wrong_blinkers_advice = "No Comment Here"
            self.wrong_blinker_fail = False

        if(self.speed_fail == True): #done
            Overspeeding_test = "yes"
            if (Overspeeding_test == "yes"):
                self.over_speeding_advice = "Based on your driving data, we recommend \n maintaining a speed of under 30 km/hr to\n improve your safety on the road. To achieve this, \n keep an eye on your speedometer and adjust \nyour speed accordingly. Remember, in real life, \ndriving slower can also help you save fuel,\n reduce wear and tear on your vehicle, \nand decrease emissions!"
            else:
                Overspeeding_test = "No Comment Here"
            self.speed_fail = False

        if(self.instructor_fail == True):#done
            instructions_test = "yes"
            if (instructions_test == "yes"):
                self.instviolation_advice = "Based on your driving data, it appears \n that you may have violated some instructions\n given by your driving instructor. We recommend \ncarefully reviewing the instructions you were \ngiven and practicing the appropriate driving\n techniques to ensure that you're following\n them correctly."
            else:
                self.instviolation_advice = "No Comment Here"
            immediate_fail = "yes"
            self.instructor_fail = False

        if(self.collision_fail == True):
            collisons_test = "yes"
            if (collisons_test == "yes"):
                self.collision_advice = "To reduce the risk of collisions, keep \na safe distance from other vehicles, be aware\n of your surroundings, and anticipate other drivers'\nactions. Obey traffic signals and speed limits,\n avoid distractions, and  never drive under \nthe influence. Prioritize safety at all times\n while driving. "
            else:
                self.collision_advice = "No Comment Here"
            immediate_fail = "yes"
            self.collision_fail = False

        if(self.time_fail == True):
            time_test = "yes"
            if (time_test == "yes"):
                self.timelimit_advice = "When parallel parking, aim to complete the \nmaneuver within 5 minutes to avoid blocking \ntraffic. If you're having difficulty, find another\n spot or come back later. Remember \nto prioritize safety and be considerate of\n other drivers. "
            else:
                self.timelimit_advice = "No Comment Here"
            immediate_fail = "yes"
            self.time_fail = False

        if(self.sidewalk_fail == True):
            sidewalk_test = "yes"

            if (sidewalk_test == "yes"):
                self.sidewalk_advice = "Based on your driving data, it appears \nthat you may have gone over a sidewalk \nwhile driving. We recommend being more mindful of\n your vehicle's position on the road \nand avoiding any maneuvers that may\n cause you to leave the roadway."
            else:
                self.sidewalk_advice = "No Comment Here"
            immediate_fail = "yes"
            self.sidewalk_fail = False

        #----------------------------------
        if (immediate_fail == 'yes'):
            self.result = 'Failed'
        
        num_faults = num_faults + self.number_of_col + self.number_of_speed_exceeded + self.number_of_inst_violation + self.number_of_sidewalk_hit+self.number_of_blinkers_missed+self.num_of_duration_exceeded + self.number_of_wrong_blinkers

        if(num_faults >= 4):
            self.result = 'Failed'

#-----------------------------------------------------------------------------------------

        data = [['Evaluation Table'],
                ['Today\'s Date:', today],
                ['Start Time:', self.start_time],
                ['End Time:', end_time],
                ['Duration:', duration], 
                ['Vehicle: ', cartype],
                ['Number of Faults:', num_faults],
                ['Immediate Fail:', immediate_fail]]

        # Create the table and apply style
        table = Table(data, colWidths=[340, 220])
        table.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black),
                                ('SPAN', (0,0), (1,0))]))

        # Define other data for the table 1
#-----------------------------------------------------------------------------------------
        #--make variables for this--
        #---------------------------

        data2 = [['Immediate Fail'],
                ['Collisions', collisons_test],
                ['Not Following Instructions', instructions_test],
                ['Exceed Time Limit', time_test],
                ['Sidewalk Hit(s)', sidewalk_test]
                ]

        # Create the table and apply style
        table2 = Table(data2, colWidths=[340, 220])
        table2.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black), 
                                ('SPAN', (0,0), (1,0))]))

        # Define other data for the table 3
        #---------------------------
        #--make variables for this--
#-----------------------------------------------------------------------------------------

        data3 = [['Faults','', 'Number of Faults'],
                ['1) Over Speeding', Overspeeding_test, self.number_of_speed_exceeded],
                ['2) Not using blinkers', blinker_test, self.number_of_blinkers_missed],
                #['3) Using Wrong blinkers', wrong_blinker_test, self.number_of_wrong_blinkers],
                #['4) Time Exceeded', time_test, duration ],
                ['5) Number of Collisions', collisons_test , self.number_of_col ],
                ['6) Following Instructions', instructions_test ,self.number_of_inst_violation],
                ['7) Sidewalk Hit(s)', sidewalk_test , self.number_of_sidewalk_hit]]

        # Create the table and apply style
        table3 = Table(data3, colWidths=[120, 220])
        table3.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black), 
                                ('SPAN', (0,0), (1,0))
                                
                                ]))

        # Define other data for the table 1
        #---------------------------
        #--make variables for this--
#-----------------------------------------------------------------------------------------

        data5 = [['Result', self.result]]

        # Create the table and apply style
        table5 = Table(data5, colWidths=[340, 220])
        table5.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black)]
                                ))
        
#-----------------------------------------------------------------------------------------

        data4 = [['Feedback Summary'],
                ['Faults', "Yes/No", 'Advice Given'],
                ['1) Over Speeding', Overspeeding_test , self.over_speeding_advice],
                ['2) Not using blinkers', blinker_test, self.blinkers_advice],
                ['3) Using Wrong blinkers', wrong_blinker_test, self.wrong_blinkers_advice],
                ['3) Time Exceeded', time_test , self.timelimit_advice ],
                ['4) Number of Collisions', collisons_test , self.collision_advice ],
                ['5) Following Instructions', instructions_test , self.instviolation_advice ],
                ['6) Sidewalk Hit(s)', sidewalk_test , self.sidewalk_advice]]

        # Create the table and apply style
        table4 = Table(data4, colWidths=[120, 220])
        table4.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black), 
                                ('SPAN', (0,0), (-1,0)),
                                ('VALIGN', (0,0), (-1,-1), 'MIDDLE')
                                
                                ]))

        # Create the PDF document and add the table
        doc = SimpleDocTemplate("Driving_Evaluation.pdf", pagesize=letter)
        doc.build([table, table2, table3, PageBreak(), table4, table5])

        print('PDF report generated successfully!')
               

# ==============================================================================
# -- ObstacleSensor ------------------------------------------------------------
# ==============================================================================


class SafetyDistance(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.distance = 0.0
        self.other_actor = 'None'
        self.last_distance = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.obstacle')
        bp.set_attribute('distance','20')
        bp.set_attribute('hit_radius','1.5')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.7, z=1.6)), attach_to=self._parent)
        
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: SafetyDistance._on_obstacle_event(weak_self, event))

    @staticmethod
    def _on_obstacle_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.distance = event.distance
        self.other_actor = event.other_actor.type_id

class SafetyDistanceBehind(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.distance = 0.0
        self.other_actor = 'None'
        self.last_distance = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.obstacle')
        bp.set_attribute('distance','20')
        bp.set_attribute('hit_radius','0.5')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.7, z=1.6),carla.Rotation(0,180,0)), attach_to=self._parent)
        
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: SafetyDistance._on_obstacle_event(weak_self, event))

    @staticmethod
    def _on_obstacle_event(weak_self, event):
        self = weak_self()
        if not self:
            return
    
        self.distance = event.distance
        self.other_actor = event.other_actor.type_id

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self._other = 'No Collision'
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self._other = actor_type
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, self._parent,event))

    @staticmethod
    def _on_invasion(weak_self, car,event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))
              
        evaluations.check_blinker(car,event) 


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class mirror_parameters:
    def __init__(self):
        self.left_yaw=-150
        self.left_pitch=0
        self.right_yaw=150
        self.right_pitch=0

mp = mirror_parameters()

class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction, display_man):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self.display_man = display_man
        self.display_pos = None
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        lx,ly,lz = [0.5, -1, 1.0]
        rx,ry,rz = [0.5, 1, 1.0]

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                #(carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
                #(carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                # (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
                # (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                # (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=rx, y=ry, z=rz), carla.Rotation(pitch=mp.right_pitch,yaw=mp.right_yaw)), Attachment.Rigid) ]
  
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB']]
        # self.sensors = [
        #     ['sensor.camera.rgb', cc.Raw, 'Camera RGB',{}]
        #     ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
        #         ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
        #         ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
        #         ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
        #         ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
        #         ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
        #         ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
        #         ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
        #         ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
        #         ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
        #             {'lens_circle_multiplier': '3.0',
        #             'lens_circle_falloff': '3.0',
        #             'chromatic_aberration_intensity': '0.5',
        #             'chromatic_aberration_offset': '0'}],
        #         ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
        # ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            bp.set_attribute('image_size_x', str(hud.dim[0]))
            bp.set_attribute('image_size_y', str(hud.dim[1]))
            bp.set_attribute('gamma', '2.2')
            item.append(bp)
        self.index = None
        # bp_library = world.get_blueprint_library()
        # for item in self.sensors:
        #     bp = bp_library.find(item[0])
        #     if item[0].startswith('sensor.camera'):
        #        bp.set_attribute('image_size_x', str(hud.dim[0]))
        #        bp.set_attribute('image_size_y', str(hud.dim[1]))
        #        if bp.has_attribute('gamma'):
        #            bp.set_attribute('gamma', str(gamma_correction))
        #        for attr_name, attr_value in item[3].items():
        #            bp.set_attribute(attr_name, attr_value)
        #     elif item[0].startswith('sensor.lidar'):
        #          self.lidar_range = 50
        #          for attr_name, attr_value in item[3].items():
        #              bp.set_attribute(attr_name, attr_value)
        #              if attr_name == 'range':
        #                 self.lidar_range = float(attr_value)

        #     item.append(bp)
        # self.index = None

        SensorManager(world, self.display_man, 'RGBCamera',self.hud, self._camera_transforms[0][0],#carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw)), 
                       self._parent, {}, display_pos=[0, 0] )
        SensorManager(world, self.display_man, 'RGBCamera',self.hud ,  carla.Transform(carla.Location(x=-0.1,y=-0.4, z=1.2), carla.Rotation(yaw=+00)), 
                        self._parent, {}, display_pos=[0, 1] )
        SensorManager(world, self.display_man, 'RGBCamera',self.hud , self._camera_transforms[1][0],#carla.Transform(carla.Location(x=rx, y=ry, z=rz), carla.Rotation(pitch=mp.right_pitch,yaw=mp.right_yaw)), 
                         self._parent, {}, display_pos=[0, 2] )

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

class SensorManager:
    def __init__(self, world, display_man, sensor_type, hud, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.hud = hud

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)
            self.hud.render(self.display_man.display)

    def destroy(self):
        self.sensor.destroy()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()


evaluations = Evaluations()

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    display_manager = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)
        sim_world = client.get_world()

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()
        
        display_manager = DisplayManager(grid_size=[1, 3], window_size=[args.width, args.height])

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args,display_manager)
       # controller = KeyboardControl(world, args.autopilot)




        ###############################for steering wheel################################
        #controller = DualControl(world, args.autopilot)

        controller = KeyboardControl(world,args.autopilot)


        sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock): #for keyboard
            #if controller.parse_events(world, clock)
                return
            if not world.tick(clock):
                return
            display_manager.render()
            # world.render(display)
            # pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            # prevent destruction of ego vehicle
            if args.keep_ego_vehicle:
                world.player = None
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot. This does not autocomplete the scenario')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='role name of ego vehicle to control (default: "hero")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',   #5770x1080
        help='window resolution (default: 1920x1080)') #5770x1080
    argparser.add_argument(
        '--keep_ego_vehicle',
        action='store_true',
        help='do not destroy ego vehicle on exit')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        print('exceptions')
        logging.exception(error)


if __name__ == '__main__':

    main()
