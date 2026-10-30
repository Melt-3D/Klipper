# Custom modifications for Melt Melta
#
# Copyright (C) 2025-2026  Juraj Minaric <>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Adds the V and W gcode axes. They are offsets of stepper_z1 and
# stepper_z2 from stepper_z, so "G1 Z.." moves all three Z motors
# together and "G1 V.. W.." moves stepper_z1 / stepper_z2 on their own.
#
# While both offsets are zero the Z motors are "synced" (stock Klipper
# behaviour on the toolhead trapq).  The first move that changes V or W
# switches stepper_z1 and stepper_z2 to their own trapq, which gets the
# combined Z + offset motion of every move ("independent" mode).
import math, logging
import chelper

class ZOffsetAxis:
    def __init__(self, melt, config, gcode_id, stepper_name):
        self.melt = melt
        self.printer = config.get_printer()
        self.gcode_id = gcode_id
        self.stepper_name = stepper_name
        self.stepper = None
        self.synced_sk = None
        self.offset_min = config.getfloat(gcode_id.lower() + '_min', None)
        self.offset_max = config.getfloat(gcode_id.lower() + '_max', None)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
        # Request check_move/process_move calls for Z only moves too
        self.follows_z = True
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_set_position = ffi_lib.trapq_set_position
        self.stepper_kinematics = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
    def get_index(self):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_extra_axes().index(self)
    # Mode switching (toolhead must be flushed by the caller)
    def enter_independent(self, height, print_time):
        self.synced_sk = self.stepper.set_stepper_kinematics(
            self.stepper_kinematics)
        self.stepper.set_position([height, 0., 0.])
        self.stepper.set_trapq(self.trapq)
        self.trapq_set_position(self.trapq, print_time, height, 0., 0.)
    def enter_synced(self, toolhead_pos):
        toolhead = self.printer.lookup_object('toolhead')
        self.stepper.set_stepper_kinematics(self.synced_sk)
        self.stepper.set_trapq(toolhead.get_trapq())
        self.stepper.set_position(toolhead_pos[:3])
        self.synced_sk = None
    def set_height(self, height, print_time):
        self.stepper.set_position([height, 0., 0.])
        self.trapq_set_position(self.trapq, print_time, height, 0., 0.)
    # Toolhead extra axis interface
    def get_axis_gcode_id(self):
        return self.gcode_id
    def get_trapq(self):
        return self.trapq
    def check_move(self, move, ea_index):
        melt = self.melt
        melt.check_z_lock(move)
        if not move.axes_d[ea_index] and not melt.independent:
            # Z only move while synced - kinematics already checked it
            return
        offset = move.end_pos[ea_index]
        if ((self.offset_min is not None and offset < self.offset_min)
            or (self.offset_max is not None and offset > self.offset_max)):
            raise move.move_error("%s offset out of range" % (self.gcode_id,))
        # Offsets may be changed with Z unhomed (to recover from a
        # failed G28), the absolute height is only known once homed
        height = move.end_pos[2] + offset
        if melt.is_z_homed() and (height < melt.unlocked_z_min
                                  or height > melt.z_max):
            raise move.move_error("%s out of range" % (self.stepper_name,))
        # Limit to Z velocity/accel using the combined motor travel
        dist = move.axes_d[2] + move.axes_d[ea_index]
        if dist:
            ratio = move.move_d / abs(dist)
            move.limit_speed(melt.max_z_velocity * ratio,
                             melt.max_z_accel * ratio)
        if move.axes_d[ea_index]:
            melt.set_independent()
    def calc_junction(self, prev_move, move, ea_index):
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def process_move(self, print_time, move, ea_index):
        if not self.melt.independent:
            return
        start_pos = move.start_pos[2] + move.start_pos[ea_index]
        axis_r = move.axes_r[2] + move.axes_r[ea_index]
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          start_pos, 0., 0.,
                          axis_r, 0., 0.,
                          move.start_v, move.cruise_v, move.accel)

class Melt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}

        self.a_angle = 0.0
        self.b_angle = 0.0
        self.length = config.getfloat('length', 250.95, above=0.)
        self.width = config.getfloat('width', 285.9, above=0.)
        self.eject_coords = {"X": 0.0, "Y": 200.0, "Z": 10.0}

        # V/W axes
        self.independent = False
        self.z_min = self.z_max = 0.
        # Z below [stepper_z] position_min (down to unlocked_z_min) is
        # only allowed while V or W is non-zero
        self.unlocked_z_min = config.getfloat('unlocked_z_min', None)
        self.max_z_velocity = self.max_z_accel = 0.
        self.axes = [ZOffsetAxis(self, config, 'V', 'stepper_z1'),
                     ZOffsetAxis(self, config, 'W', 'stepper_z2')]
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("toolhead:set_position",
                                            self._handle_set_position)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        # Register commands
        gcode = self.printer.lookup_object('gcode')

        gcode.register_command('GET_OFFSET', self.cmd_GET_OFFSET, desc=self.cmd_GET_OFFSET_help)
        gcode.register_command('G14', self.cmd_G14, desc=self.cmd_G14_help)
        gcode.register_command('GET_ANGLE', self.cmd_GET_ANGLE, desc=self.cmd_GET_ANGLE_help)
        gcode.register_command('G15', self.cmd_G15, desc=self.cmd_G15_help)
    def register_stepper(self, config, mcu_stepper):
        self.steppers[mcu_stepper.get_name()] = mcu_stepper
    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]

    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        status = kin.get_status(None)
        self.z_min = status['axis_minimum'][2]
        if self.unlocked_z_min is None:
            self.unlocked_z_min = self.z_min
        elif self.unlocked_z_min > self.z_min:
            raise self.printer.config_error(
                "unlocked_z_min in section 'melt' must be at most"
                " [stepper_z] position_min")
        self.z_max = status['axis_maximum'][2]
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = getattr(kin, 'max_z_velocity', max_velocity)
        self.max_z_accel = getattr(kin, 'max_z_accel', max_accel)
        for axis in self.axes:
            axis.stepper = self.lookup_stepper(axis.stepper_name)
            toolhead.add_extra_axis(axis, 0.)
    def is_z_homed(self):
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        return 'z' in toolhead.get_kinematics().get_status(
            eventtime)['homed_axes']
    def get_offsets(self):
        pos = self.printer.lookup_object('toolhead').get_position()
        return [pos[axis.get_index()] for axis in self.axes]
    def check_z_lock(self, move):
        if move.end_pos[2] >= self.z_min:
            return
        if any(abs(move.end_pos[axis.get_index()]) > 0.000001
               for axis in self.axes):
            return
        if move.axes_d[2] or self.is_z_homed():
            raise move.move_error(
                "Z below %.3f needs a V or W offset" % (self.z_min,))
    def set_independent(self):
        if self.independent:
            return
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        print_time = toolhead.get_last_move_time()
        for axis in self.axes:
            axis.enter_independent(pos[2] + pos[axis.get_index()], print_time)
        self.independent = True
        logging.info("melt: Z motors switched to independent mode")
    def set_synced(self, reason):
        if not self.independent:
            return
        if any(abs(o) > 0.000001 for o in self.get_offsets()):
            raise self.printer.command_error(
                "Move V and W back to 0 before %s" % (reason,))
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        for axis in self.axes:
            axis.enter_synced(pos)
        self.independent = False
        logging.info("melt: Z motors switched to synced mode")
    def _handle_set_position(self):
        # Homing resets the kinematic Z range to [stepper_z] - extend it
        # down to unlocked_z_min, check_z_lock() enforces position_min
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_min, z_max = kin.limits[2]
        if z_min <= z_max:
            kin.limits[2] = (min(z_min, self.unlocked_z_min), z_max)
        # Toolhead Z was re-anchored - keep the offsets on top of it
        if not self.independent:
            return
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        print_time = toolhead.get_last_move_time()
        for axis in self.axes:
            axis.set_height(pos[2] + pos[axis.get_index()], print_time)
    def _handle_home_rails_begin(self, homing_state, rails):
        if 2 in homing_state.get_axes():
            self.set_synced("homing Z")
    def _handle_homing_move_begin(self, hmove):
        z_steppers = [axis.stepper for axis in self.axes]
        for mcu_endstop in hmove.get_mcu_endstops():
            if any(s in z_steppers for s in mcu_endstop.get_steppers()):
                self.set_synced("probing")
                return

    cmd_GET_OFFSET_help = "Get current V and W offset"
    def cmd_GET_OFFSET(self, gcmd):
        v_offset, w_offset = self.get_offsets()
        mode = "independent" if self.independent else "synced"
        if abs(v_offset) > 0.000001 or abs(w_offset) > 0.000001:
            z_min = self.unlocked_z_min
        else:
            z_min = self.z_min
        gcmd.respond_info("V offset: %.4f, W offset: %.4f (%s), Z min: %.3f"
                          % (v_offset, w_offset, mode, z_min))

    def _check_collision(self, angle, length, width):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()

        rad = math.radians(- angle)
        y_line = math.tan(rad) * (curpos[0] - (80)) + curpos[2]

        if 0 > y_line:
            raise self.printer.command_error("Prevented collision on A angle")

    def _angle_offsets(self, a_angle, b_angle):
        a_offset = math.tan(math.radians(a_angle)) * self.length / 2
        b_offset = math.tan(math.radians(b_angle)) * self.width / 2
        return a_offset, b_offset

    def angle_move(self, new_a_angle, new_b_angle, speed):
        # A tilt: stepper_z -a, stepper_z1 and stepper_z2 +a
        # B tilt: stepper_z1 -b, stepper_z2 +b
        toolhead = self.printer.lookup_object('toolhead')
        gcode_move = self.printer.lookup_object('gcode_move')
        a_offset, b_offset = self._angle_offsets(self.a_angle, self.b_angle)
        new_a_offset, new_b_offset = self._angle_offsets(new_a_angle,
                                                         new_b_angle)
        da = new_a_offset - a_offset
        db = new_b_offset - b_offset
        v_axis, w_axis = self.axes

        curpos = toolhead.get_position()
        curpos[2] -= da
        curpos[v_axis.get_index()] += 2 * da - db
        curpos[w_axis.get_index()] += 2 * da + db
        toolhead.move(curpos, speed)

        self.a_angle = new_a_angle
        self.b_angle = new_b_angle
        # stepper_z was lowered by da - keep the gcode Z position unchanged
        gcode_move.base_position[2] -= da
        gcode_move.reset_last_position()

    cmd_G14_help = "Separate movement of the Z axis steppers with \
                    angle as input"
    def cmd_G14(self, gcmd):
        speed = 20.0
        new_a_angle = 0.0
        new_b_angle = 0.0

        params = gcmd.get_command_parameters()

        if "A" in params:
            if "R" in params:
                new_a_angle = self.a_angle + float(params["A"])
            else:
                new_a_angle = float(params["A"])

        if "B" in params:
            if "R" in params:
                new_b_angle = self.b_angle + float(params["B"])
            else:
                new_b_angle = float(params["B"])

        if "F" in params:
            gcode_speed = float(params["F"])
            if gcode_speed <= 0.:
                raise gcmd.error("Invalid speed in '%s'"
                                    % (gcmd.get_commandline(),))
            speed = gcode_speed * (1. / 60.)

        self.angle_move(new_a_angle, new_b_angle, speed)

    cmd_GET_ANGLE_help = "Get current angle"
    def cmd_GET_ANGLE(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        a_offset, b_offset = self._angle_offsets(self.a_angle, self.b_angle)
        msg = f"A angle: {str(self.a_angle)}, B angle: {str(self.b_angle)}"
        gcode.respond_info(str(msg))
        msg = f"A offset: {str(a_offset)}, B offset: {str(b_offset)}"
        gcode.respond_info(str(msg))

    cmd_G15_help = "Automatically removes print from print bed"
    def cmd_G15(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        params = gcmd.get_command_parameters()
        #  Parse the gcode thats currently printing and save important values for later
        if 'A' in params:
            print_stats = self.printer.lookup_object('print_stats')
            filename = print_stats.get_status(0)['filename']
            if filename == "":
                raise gcmd.error("No file printing currently")
            filelines = []
            with open("/home/biqu/printer_data/gcodes/" + filename, "r") as file:
                filelines = file.read().split("\n")
            seen_layer = 0
            sum_x = 0.0
            amnt_x = 0
            self.eject_coords["X"] = 220.0
            for line in filelines:
                if seen_layer == 1:
                    splitline = line.split()
                    if len(splitline) > 2:
                        if splitline[1][0] == "X":
                            amnt_x += 1
                            sum_x += float(splitline[1][1:])
                        if splitline[2][0] == "Y":
                            self.eject_coords["Y"] = min(self.eject_coords["Y"], float(splitline[2][1:]))

                if line == ";LAYER_CHANGE":
                    seen_layer += 1
                    if seen_layer > 1:
                        break
            self.eject_coords["X"] = sum_x / amnt_x
            # gcode.respond_info(str(self.eject_coords))
            # raise gcmd.error("STOP")

        #  Eject the print
        elif 'E' in params:
            toolhead = self.printer.lookup_object("toolhead")
            curpos = toolhead.get_position()
            sensor = self.printer.lookup_object("extruder")
            reactor = self.printer.get_reactor()
            eventtime = reactor.monotonic()

            max_temp = 30.0

            while not self.printer.is_shutdown():
                temp = float(sensor.stats(eventtime)[1].split()[2][5:])
                if temp <= max_temp:
                    break
                eventtime = reactor.pause(eventtime + 1.)
            gcode.respond_info("under 30")

            self.eject_coords["X"] = 100

            curpos[0] = self.eject_coords["X"]
            curpos[1] = 200

            toolhead.move(curpos, 120)

            curpos[2] = self.eject_coords["Z"]

            toolhead.move(curpos, 80)

            curpos[1] = self.eject_coords["Y"]

            toolhead.move(curpos, 60)

            curpos[2] = 110

            toolhead.move(curpos, 80)

            self.angle_move(30, 0, 20)
            self.angle_move(0, 0, 20)

def load_config(config):
    return Melt(config)
