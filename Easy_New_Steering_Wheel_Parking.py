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
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

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


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import glob
import os
from sre_parse import FLAGS
import sys
from tkinter import BOTTOM
from xml.dom import NoModificationAllowedErr

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
import random
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle
from reportlab.lib import colors
import datetime
import time


if sys.version_info >= (3, 0):

    from configparser import ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)
    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args, display_manager):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
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
        self.display_manager = display_manager #display manager
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self.obstacle_front = None
        self.obstacle_behind = None
        self.obstacle_left = None
        self.obstacle_right = None
        self.obstacle_front_right_corner = None
        self.obstacle_rear_right_corner = None
        self.obstacle_front_left_corner = None
        self.obstacle_rear_left_corner = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(get_actor_blueprints(self.world, 'vehicle.dodge.charger_2020', self._actor_generation))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
            #self.evaluations =  Evaluations(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            #spawn_points = self.map.get_spawn_points()
            #-----------------------------------------------------------------------------------------
            #------------------Spawning at the specific point in the highway--------------------------
            #-----------------------------------------------------------------------------------------
            loc = carla.Location(190.2335662841797,-1.9647053480148315,1.4169906377792358)
            rot = carla.Rotation(0,90,0)
            spawn_point = carla.Transform(loc,rot)
            #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.obstacle_front = SafetyDistance(self.player, self.hud,'front')
        self.obstacle_behind = SafetyDistance(self.player, self.hud,'behind')
        self.obstacle_left = SafetyDistance(self.player, self.hud,'left')
        self.obstacle_right = SafetyDistance(self.player, self.hud,'right')
        self.obstacle_front_right_corner = SafetyDistance(self.player, self.hud,'front right corner')
        self.obstacle_rear_right_corner = SafetyDistance(self.player, self.hud,'rear right corner')
        self.obstacle_front_left_corner = SafetyDistance(self.player, self.hud,'front left corner')
        self.obstacle_rear_left_corner = SafetyDistance(self.player, self.hud,'rear left corner')
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma,self.display_manager)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor,
            self.obstacle_front.sensor,
            self.obstacle_behind.sensor,
            self.obstacle_left.sensor,
            self.obstacle_right.sensor,
            self.obstacle_front_right_corner,
            self.obstacle_rear_right_corner,
            self.obstacle_front_left_corner ,
            self.obstacle_rear_left_corner]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
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
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
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
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
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
            if isinstance(self._control, carla.VehicleControl):
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
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
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

    def _parse_walker_keys(self, keys, milliseconds, world):
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
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

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
                    # print('0')
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
                    #print('brake')
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    #print('rverse')
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    #print('set lights')
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                self._control.reverse = self._control.gear < 0
                self._control.hand_brake = self._control.gear == 0
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
        K1 = 0.6  # 0.55
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
        self.start_time = time.time()
        self.end_time = None
        self.last_time = -1
        self.count = 0

   
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

        # if ((self.last_actor == world.obstacle_detector.other_actor and self.last_dist == world.obstacle_detector.distance) 
        #     or world.obstacle_detector.other_actor == 'static.road' or world.obstacle_detector.other_actor == 'static.roadLine'):
        #     world.obstacle_detector.distance = 0.0
        #     world.obstacle_detector.other_actor = None
        # else:
        #     self.last_actor = world.obstacle_detector.other_actor
        #     self.last_dist = world.obstacle_detector.distance
      

        if ((int(self.simulation_time) - self.last_time) == 1):
            if (round(speed)==0 and world.player.get_control().hand_brake):
                self.count = self.count + 1
            else:
                self.count = 0

        evaluations.speedTest(world.player,self)
        evaluations.parkTest(world.player, self.count, self)
        # evaluations.collisions( world.collision_sensor,collision)
        evaluations.instructortest(world.player, self)
        evaluations.sidewalk_detection(world.player,self)

        self.last_time = int(self.simulation_time)

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'D'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
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

    ############Aligns the notification text in the center###############
    def set_text(self, text, color=(255, 64, 64), seconds=1.5):
        font = pygame.font.SysFont('Arial', 48, bold=True) # create new font object
        text_texture = font.render(text, True, color)
        self.surface = pygame.Surface(self.dim, pygame.SRCALPHA)
        self.seconds_left = seconds

        text_rect = text_texture.get_rect(centerx=self.surface.get_rect().centerx, top=-15)
        self.surface.blit(text_texture, text_rect)



    # def set_text(self, text, color=(255, 64, 64), seconds=1.5):
    #     text_texture = self.font.render(text, True, color)
    #     self.surface = pygame.Surface(self.dim, pygame.SRCALPHA)
    #     self.seconds_left = seconds
        
    #     text_rect = text_texture.get_rect(centerx=self.surface.get_rect().centerx, top=-10)
    #     self.surface.blit(text_texture, text_rect)


    # def set_text(self, text, color=(255, 64, 64), seconds=1.5):
    #     text_texture = self.font.render(text, True, color)
    #     self.surface = pygame.Surface(self.dim, pygame.SRCALPHA)
    #     self.seconds_left = seconds
    #     self.surface.fill((0, 0, 0, 0))
        
    #     text_rect = text_texture.get_rect(center=self.surface.get_rect().center)
    #     text_rect = text_texture.get_rect(center=self.surface.get_rect().top)
    #     text_rect.centerx = self.surface.get_rect().centerx
    #     self.surface.blit(text_texture, text_rect)
   


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

    ################ Flags ########################################## #if false didn't fail, if  true failed
    wrong_blinker_fail = False 
    no_blinker__fail = False 
    collision_fail = False
    instructor_fail = False
    time_fail = False
    speedTest = False
    result = 'pass'
    start_time  = datetime.datetime.now().replace(microsecond=0)
    last_time = None
    number_of_col= 0


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
        self.number_of_inst_violation = 0
        self.number_of_sidewalk_hit = 0
    
    def speedTest(self, car, hud):
        v = car.get_velocity()
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),2)
        if (speed > 20):
            hud.notification('Slow Down!')
            self.number_of_speed_exceeded += 1

            # print('Over Speed!')


    def parkTest(self, car, count, hud):
        v = car.get_velocity()
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),2)
        t = car.get_transform()
        if(((t.location.x >185 and t.location.x <187.5) and (t.location.y > 21 and t.location.y < 24)) and (t.rotation.yaw > 85 and t.rotation.yaw < 95) and count>3):
            
            hud.notification('You Have Parked Successfully')
            time.sleep(5)
            self.generateReport() 
            pygame.quit()
            print ('Parked')
        #else:
            #print('Not parked')

    def collisions(self,impulse):
        time_current = time.time()
        if(self.last_time == None):
            self.number_of_col=self.number_of_col+1
        elif(time_current - self.last_time >0.4 and impulse > 5):
            self.number_of_col=self.number_of_col+1
        self.last_time = time_current

        if(self.number_of_col >=3):
            self.collision_fail =True
        
    def check_blinker_parking(self,car,event):
        # print("INSIDE Blinker")
        light_state = car.get_light_state()
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        
        if(text[0]=="'Solid'"): #checks for if the line is "Solid". True is Pass, False is Fail
            if light_state in self.left_light_states: #invading line with left blinker
                print('Invade with LB')
                self.wrong_blinker_fail= True
                self.number_of_blinkers_missed =+ 1
                return False

            elif light_state in self.right_light_states: #invading line with right blinker
                print('Invade lane RB')
                return True
            
            else:                        #invading line with no blinker
                print('Invade lane NB')
                self.no_blinker__fail = True
                self.number_of_blinkers_missed =+ 1
                return False
    
    def sidewalk_detection(self, car,hud):
        t = car.get_transform()
        if ((t.location.x >= 183 and (t.location.x < 184) )and (t.location.y >-77 and t.location.y <99)):
            hud.notification('You have hit the sidewalk')
            self.number_of_sidewalk_hit += 1
            print ('sidewalk')

    def instructortest(self, car, hud):
        v = car.get_velocity()
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),2)
        t = car.get_transform()
        if (t.location.x >184 and t.location.x <191) and (t.location.y >40 and t.location.y <72):
            self.instructor_fail = True #flags if the the driver goes beyond the designated parking instruction
            print("Instruction violated")
            hud.notification('Instruction violated')
            self.number_of_inst_violation += 1
            # self.generateReport()
            # time.sleep(7)
            # pygame.quit()
    
    def time_fail(self, duration):
        
        if (duration > 300):
            print("time exceeded of 5 minutes")
            self.time_fail = True
        

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
        blinker_test = "No (0)"
        instructions_test = "No"
        Overspeeding_test= "No"
        collisons_test = "No"
        time_test = "No"
        sidewalk_test = "No"


        if(self.no_blinker__fail == True):
            blinker_test = "yes"
            
            self.no_blinker__fail = False

        if(self.wrong_blinker_fail == True): #why it failed
            blinker_test = "yes"
        
            self.wrong_blinker_fail = False

        if(self.speedTest == True):
            Overspeeding_test = "yes"
            
            self.speedTest = False

        if(self.instructor_fail == True):
            instructions_test = "yes"
            immediate_fail = "yes"
            
            self.instructor_fail = False

        if(self.collision_fail == True):
            collisons_test = "yes"
            immediate_fail = "yes"
            
            self.collision_fail = False

        if(self.time_fail == True):
            time_test = "yes"
            immediate_fail = "yes"
            
            self.time_fail = False

        if(self.sidewalk_fail == True):
            sidewalk_test = "yes"
            immediate_fail = "yes"
            
            self.sidewalk_fail = False

        #----------------------------------
        if (immediate_fail == 'yes'):
            self.result = 'Failed'
        
        num_faults = num_faults + self.number_of_col + self.number_of_speed_exceeded + self.number_of_inst_violation + self.number_of_sidewalk_hit

        data = [['Evaluation Table'],
                ['Today\'s Date:', today],
                ['Start Time:', self.start_time],
                ['End Time:', end_time],
                ['Duration:', duration], 
                ['Vehicle: ', cartype],
                ['Number of Faults:', num_faults],
                ['Immediate Fail:', immediate_fail]]

        # Create the table and apply style
        table = Table(data, colWidths=[120, 220])
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


        #---------------------------
        #--make variables for this--
        #---------------------------

        data2 = [['Immediate Fail'],
                ['Collisions', collisons_test],
                ['Not Following Instructions', instructions_test],
                ['Exceed Time Limit', time_test],
                ['Sidewalk Hit(s)', sidewalk_test]
                ]

        # Create the table and apply style
        table2 = Table(data2, colWidths=[120, 220])
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
        #---------------------------

       

        data3 = [['Number of Faults'],
                ['1) Over Speeding', Overspeeding_test + "(" + str(self.number_of_speed_exceeded) + ")"],
                ['2) Not using blinkers', blinker_test + "(" + str(self.number_of_blinkers_missed) + ")"],
                ['3) Time Exceeded', time_test ,duration ],
                ['4) Number of Collisions', collisons_test + "(" + self.number_of_col + ")"],
                ['5) Not Following Instructions', instructions_test + "(" + self.number_of_inst_violation + ")"],
                ['6) Sidewalk Hit(s)', sidewalk_test + "(" + self.number_of_sidewalk_hit + ")"]]

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
                                ('SPAN', (0,0), (1,0))]))

        # Define other data for the table 1
        #---------------------------
        #--make variables for this--
        #---------------------------
        data4 = [['Result', self.result]]

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
                                ('GRID', (0,0), (-1,-1), 1, colors.black)]
                                ))

        # Create the PDF document and add the table
        doc = SimpleDocTemplate("Parking_Evaluation.pdf", pagesize=letter)
        doc.build([table, table2, table3, table4])

        print('PDF report generated successfully!')
               


# ==============================================================================
# -- ObstacleDetector ----------------------------------------------------------
# ==============================================================================

class SafetyDistance(object):

    def __init__(self, parent_actor, hud, side):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        self.distance = 0
        self.other_actor = None
        self.side = side
        self.last_actor= None
        self.last_dist = None

        bound_x = self._parent.bounding_box.extent.x
        bound_y = self._parent.bounding_box.extent.y
        bound_z = self._parent.bounding_box.extent.z

        # self.sensor_transform = carla.Transform(carla.Location(x=1.6, z=1.7), carla.Rotation(yaw=0)) # Put this sensor on the windshield of the car.
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.obstacle')
        
        #x=bound_x, y=0, z=0.5
        #x=-bound_x, y=0, z=0.5
        #x=0, y=bound_y, z=0.5 carla.Rotation(pitch=-30,yaw=180)
        #x=0, y=-bound_y, z=0.5 carla.Rotation(pitch=-30,yaw=180)


        if(side == 'behind'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=-bound_x, y=0, z=bound_z),carla.Rotation(0,180,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif(side == 'left'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0, y=-bound_y, z=bound_z),carla.Rotation(0,-90,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif (side == 'right'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0, y=bound_y, z=bound_z),carla.Rotation(0,90,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif (side == 'front'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=bound_x, y=0, z=bound_z),carla.Rotation(0,0,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif(side == 'front right corner'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0, y=-bound_y, z=bound_z),carla.Rotation(0,45,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif(side == 'rear right corner'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0, y=-bound_y, z=bound_z),carla.Rotation(0,135,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif(side == 'front left corner'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0, y=-bound_y, z=bound_z),carla.Rotation(0,225,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        elif(side == 'rear left corner'):
            self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0, y=-bound_y, z=bound_z),carla.Rotation(0,315,0)), attach_to=self._parent)
            bp.set_attribute('distance','10')
            bp.set_attribute('hit_radius', '0.5')
        



        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.


        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: SafetyDistance._on_obstacle(weak_self, event,world))

    @staticmethod
    def _on_obstacle(weak_self, event,world):
        self = weak_self()
        if not self:
            return
        
        self.distance = 0
        self.other_actor = 'None'
     
        distance = event.distance
        #print('Distance: ',distance)
        #print(event.other_actor.type_id)
        v = self._parent.get_velocity()
        speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

        #print('Speed: ',speed)
        if (speed != 0):
            if(distance < 2.0):
                if(self.side == 'front'):
                    self.hud.notification('You are too close to the object in front of you')
                elif(self.side == 'behind'):
                    self.hud.notification('You are too close to the object behind of you')
                elif(self.side == 'left'):
                    self.hud.notification('You are too close to the object left of you')
                elif(self.side == 'right'):
                    self.hud.notification('You are too close to the object right of you')
                # elif(self.side == 'front right corner'):
                #     print("front right")
                #     self.hud.notification('You are too close to the object front right corner of the car')
                # elif(self.side == 'rear right corner'):
                #     print("rear right")
                #     self.hud.notification('You are too close to the object rear right of you')
                # elif(self.side == 'front left corner'):
                #     print("front left")
                #     self.hud.notification('You are too close to the object front left corner of the car')
                # elif(self.side == 'rear left corner'):
                #     self.hud.notification('You are too close to the object rear left of you')
                #     print("rear left")


    
        self.distance = event.distance
        self.other_actor = event.other_actor.type_id
        # # actor = (get_actor_display_name(event.other_actor)) 
        # print(event.other_actor.type_id)

        # if(get_actor_display_name(event.other_actor) != "vehicle.dodge.charger_2020"):
        #     print(event.other_actor)

        # if(self.side == 'front'):
        #     print('front', self.other_actor, self.distance)
        # elif(self.side == 'behind'):
        #     print('behind', self.other_actor, self.distance)
        # elif(self.side == 'left'):
        #     print('left', self.other_actor, self.distance)
        # elif(self.side == 'right'):
        #     print('right', self.other_actor, self.distance)

        
        # if ((self.last_actor == world.obstacle_detector.other_actor) 
        #     or world.obstacle_detector.other_actor == 'static.road' or world.obstacle_detector.other_actor == 'static.roadLine'):
        #    world.obstacle_detector.other_actor = 'None'
        # else:
        #     self.last_actor = world.obstacle_detector.other_actor

        # print(self.last_actor)
            # evaluations.sidewalk_detection(event)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    # col_list = []
    
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        self.col_list = []
        self.other_actor_id = None
        self.temp_id = None
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
       

        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
        # col_list.append(actor_type)
        evaluations.collisions(intensity)




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

      
        evaluations.check_blinker_parking(car,event) 
        # evaluations.sidewalk_detection(event)




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
        depth_velocity = []
        dv = []
        print("Objects Detected:", radar_data.get_detection_count())
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

            depth = detect.depth
            velo = detect.velocity
            dv.extend((depth,velo))
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
        depth_velocity.append(dv)
        print(depth_velocity)

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
        self.display_man = display_man
        self.display_pos = None
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        
        lx,ly,lz = [0.5, -1, 1.0]  ###########mirrors
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
        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]]
        #     ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
        #     ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
        #     ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
        #     ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
        #     ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
        #     ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
        #     ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
        #     ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
        #     ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
        #     ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
        #         {'lens_circle_multiplier': '3.0',
        #         'lens_circle_falloff': '3.0',
        #         'chromatic_aberration_intensity': '0.5',
        #         'chromatic_aberration_offset': '0'}],
        #     ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
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
        # world = self._parent.get_world()
        # bp_library = world.get_blueprint_library()
        # for item in self.sensors:
        #     bp = bp_library.find(item[0])
        #     if item[0].startswith('sensor.camera'):
        #         bp.set_attribute('image_size_x', str(hud.dim[0]))
        #         bp.set_attribute('image_size_y', str(hud.dim[1]))
        #         if bp.has_attribute('gamma'):
        #             bp.set_attribute('gamma', str(gamma_correction))
        #         for attr_name, attr_value in item[3].items():
        #             bp.set_attribute(attr_name, attr_value)
        #     elif item[0].startswith('sensor.lidar'):
        #         self.lidar_range = 50

        #         for attr_name, attr_value in item[3].items():
        #             bp.set_attribute(attr_name, attr_value)
        #             if attr_name == 'range':
        #                 self.lidar_range = float(attr_value)

        #     item.append(bp)
        # self.index = None
        SensorManager(world, self.display_man, 'RGBCamera',self.hud, self._camera_transforms[0][0],#carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw)), 
                      self._parent, {}, display_pos=[0, 0] )
        SensorManager(world, self.display_man, 'RGBCamera',self.hud ,  carla.Transform(carla.Location(x=-0.1,y=-0.4, z=1.2), carla.Rotation(yaw=+00)), #
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

            # for key in sensor_options:
            #     camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        
        if sensor_type == 'leftcam':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '800')
            camera_bp.set_attribute('image_size_y', '600')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        
        if sensor_type == 'rightcam':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '800')
            camera_bp.set_attribute('image_size_y', '600')

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
    original_settings = None
    display_manager = None

    vehicles_list = []
    synchronous_master = False

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20000.0)
        client.load_world('Town05')



        sim_world = client.get_world()

        
        # display = pygame.display.set_mode(
        #     (args.width, args.height),
        #     pygame.HWSURFACE | pygame.DOUBLEBUF)
        # display.fill((0,0,0))
        # pygame.display.flip()
        
        display_manager = DisplayManager(grid_size=[1, 3], window_size=[args.width, args.height])
        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args,display_manager)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)

        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")
            
        #####################################################################
        ### code to spawn vehicles ##########################################
        #####################################################################

        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        #added sim_world instead of world
        blueprints = get_actor_blueprints(sim_world,'vehicle.dodge.charger_2020',args.generationv)#(sim_world, args.filterv, args.generationv)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
            blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        #locations = sim_world.get_map().get_spawn_points()
        #------------------------Getting spawn points from file----------------------------
        locations = []
        f = open('test.txt', 'r')
        for line in f:
            coord = re.findall(r"[-+]?(?:\d*\.*\d+)", line)
            point = carla.Transform(carla.Location(float(coord[0]), float(coord[1]), float(coord[2])), carla.Rotation(0,90,0))
            locations.append(point)
        

        number_of_spawn_points = len(locations)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(locations)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points
        
        SpawnActor = carla.command.SpawnActor
        #SetAutopilot = carla.command.SetAutopilot
        #FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(locations):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            #print(blueprint)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the parked cars
            #SpawnActor(blueprint, transform)
            batch.append(SpawnActor(blueprint, transform))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            all_vehicle_actors = sim_world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

    
        controller = DualControl(world, args.autopilot) #for keyboard
        #controller = KeyboardControl(world,args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            #if controller.parse_events(client, world, clock, args.sync): #for keyboard
            if controller.parse_events(world, clock):
                return
            world.tick(clock)
            display_manager.render()
            #world.render(display)
            #pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
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
        default='127.0.0.12',
        help='IP of the host server (default: 127.0.0.12)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1080', #5770x1080
        help='window resolution (default: 1920x1080)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=3,
        type=int,
        help='Number of vehicles (default: 3)')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
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


if __name__ == '__main__':

    main()
