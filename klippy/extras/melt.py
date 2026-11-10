# Custom modifications for Melt Melta
#
# Copyright (C) 2025-2026  Juraj Minaric <>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import force_move, homing
import chelper

Z_MAX_HOMING_CURRENT = 0.5
Z_MAX_RETRACT_DIST = 20.
# HOME_ZMAX DEBUG=1: largest allowed difference (mm) in how far the Z
# motors travel before stalling. Must exceed the biggest real bed tilt.
Z_MAX_MAX_DIFF = 5.

# Watches the HOME_ZMAX endstops. Once the first motor stalls, any motor
# still moving after another max_diff mm of travel is stopped, so a false
# stall on one motor can't let the others twist the bed against the stops.
class ZMaxTriggerMonitor:
    def __init__(self, printer, endstops, max_diff, speed):
        self.reactor = printer.get_reactor()
        self.endstops = endstops
        self.max_diff = max_diff
        self.max_time = max_diff / speed
        self.pending = {}
        self.first = None
        self.aborted = []
        self.timer = None
    def start(self):
        # Hook home_start() on each endstop instance to get its completion
        for mcu_endstop, name in self.endstops:
            self._hook(mcu_endstop, name)
    def _hook(self, mcu_endstop, name):
        orig_home_start = mcu_endstop.home_start
        def home_start(*args, **kwargs):
            self._unhook(mcu_endstop)
            completion = orig_home_start(*args, **kwargs)
            self.pending[name] = mcu_endstop
            self.reactor.register_callback(
                lambda e: self._wait_trigger(completion, name))
            return completion
        mcu_endstop.home_start = home_start
    def _unhook(self, mcu_endstop):
        if 'home_start' in mcu_endstop.__dict__:
            del mcu_endstop.home_start
    def stop(self):
        for mcu_endstop, name in self.endstops:
            self._unhook(mcu_endstop)
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None
        self.pending.clear()
    def _wait_trigger(self, completion, name):
        completion.wait()
        if self.pending.pop(name, None) is None:
            return
        if self.first is None:
            self.first = name
            self.timer = self.reactor.register_timer(
                self._abort_pending, self.reactor.monotonic() + self.max_time)
    def _abort_pending(self, eventtime):
        for name, mcu_endstop in self.pending.items():
            mcu_endstop.home_abort()
            self.aborted.append(name)
        self.pending.clear()
        return self.reactor.NEVER
    def get_distances(self, hmove):
        # Travel (mm) of each stepper until its stall (or abort)
        dists = {}
        for sp in hmove.stepper_positions:
            pos = sp.trig_pos
            if sp.endstop_name in self.aborted:
                pos = sp.halt_pos
            if pos is None:
                continue
            dists[sp.stepper_name] = (abs(pos - sp.start_pos)
                                      * sp.stepper.get_step_dist())
        return dists
    def format_distances(self, dists):
        return "\n".join(
            "%s: %s after %.2f mm" % (
                n, "stopped" if n in self.aborted else "stalled", d)
            for n, d in sorted(dists.items()))

TMC_DRIVERS = ['tmc2209', 'tmc2130', 'tmc2208', 'tmc2240', 'tmc2660',
               'tmc5160']

class Melt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        # Per-stepper sensorless endstops used by HOME_ZMAX
        self.z_max_endstops = []
        self.z_tmc_names = {}
        self.z_max_errors = []
        self.z_motors_lost = True

        self.a_angle = 0.0
        self.b_angle = 0.0
        self.a_offset = 0.0
        self.b_offset = 0.0
        self.length = 250.95
        self.width = 285.9
        self.eject_coords = {"X": 0.0, "Y": 200.0, "Z": 10.0}

        self.z_offset = 0.0
        self.v_offset = 0.0

        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        # Register commands
        gcode = self.printer.lookup_object('gcode')

        gcode.register_command('G13', self.cmd_G13, desc=self.cmd_G13_help)
        gcode.register_command('GET_OFFSET', self.cmd_GET_OFFSET, desc=self.cmd_GET_OFFSET_help)
        gcode.register_command('G14', self.cmd_G14, desc=self.cmd_G14_help)
        gcode.register_command('GET_ANGLE', self.cmd_GET_ANGLE, desc=self.cmd_GET_ANGLE_help)
        gcode.register_command('G15', self.cmd_G15, desc=self.cmd_G15_help)
        gcode.register_command('HOME_ZMAX', self.cmd_HOME_ZMAX,
                               desc=self.cmd_HOME_ZMAX_help)
        gcode.register_command('TEST_ZMAX', self.cmd_TEST_ZMAX, desc=self.cmd_TEST_ZMAX_help)
        # Wrap G28 (melt loads after safe_z_home/homing_override, so this
        # runs in front of them)
        self.printer.load_object(config, 'homing')
        self.prev_G28 = gcode.register_command('G28', None)
        gcode.register_command('G28', self.cmd_G28)
    def register_stepper(self, config, mcu_stepper):
        name = mcu_stepper.get_name()
        self.steppers[name] = mcu_stepper
        if name.startswith('stepper_z'):
            self._setup_z_max_endstop(config, mcu_stepper)
            # Track when this Z motor is turned off (M84, idle timeout, ...)
            stepper_enable = self.printer.lookup_object('stepper_enable')
            stepper_enable.lookup_enable(name).register_state_callback(
                self._handle_z_enable)
    def _handle_z_enable(self, print_time, enabled):
        if not enabled:
            self.z_motors_lost = True
    def _offsets_nonzero(self):
        return any([self.a_angle, self.b_angle, self.a_offset, self.b_offset,
                    self.z_offset, self.v_offset])
    def cmd_G28(self, gcmd):
        # Home Z to max first if Z motors were off or any offset is set
        need_z = gcmd.get('Z', None) is not None
        if not any(gcmd.get(a, None) is not None for a in "XYZ"):
            need_z = True
        if need_z and (self.z_motors_lost or self._offsets_nonzero()):
            self.HOME_ZMAX(Z_MAX_HOMING_CURRENT, Z_MAX_RETRACT_DIST)
        self.prev_G28(gcmd)
    def _setup_z_max_endstop(self, config, mcu_stepper):
        # Give each Z stepper its own TMC virtual endstop so that during
        # HOME_ZMAX every motor stops only on its own stall
        name = mcu_stepper.get_name()
        driver = None
        for d in TMC_DRIVERS:
            if config.has_section('%s %s' % (d, name)):
                driver = d
                break
        if driver is None:
            self.z_max_errors.append("%s: no tmc driver section" % (name,))
            return
        tmc_config = config.getsection('%s %s' % (driver, name))
        if not any(tmc_config.get(opt, None) is not None
                   for opt in ['diag_pin', 'diag0_pin', 'diag1_pin']):
            self.z_max_errors.append("%s: missing diag_pin in [%s %s]"
                                     % (name, driver, name))
            return
        ppins = self.printer.lookup_object('pins')
        mcu_endstop = ppins.setup_pin(
            'endstop', '%s_%s:virtual_endstop' % (driver, name))
        mcu_endstop.add_stepper(mcu_stepper)
        self.z_max_endstops.append((mcu_endstop, name))
        self.z_tmc_names[name] = '%s %s' % (driver, name)
    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]

    def invert_dir(self, s):
        s.set_dir_inverted(not bool(s.get_dir_inverted()[0]))

    cmd_G13_help = "(Relative) Separate movement of the Z axis steppers"
    def cmd_G13(self, gcmd):
        dis_z = gcmd.get_float('Z')
        dis_v = gcmd.get_float('V')

        speed = 40

        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        prev_z = curpos[2]
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if
                        s.is_active_axis('z')]

        toolhead.wait_moves()
        self.invert_dir(z_steppers[0])
        curpos[2] += dis_v - self.v_offset
        toolhead.move(curpos, speed)
        toolhead.wait_moves()
        self.invert_dir(z_steppers[0])
        toolhead.wait_moves()

        curpos[2] = prev_z + dis_z - self.z_offset
        self.v_offset = dis_v
        self.z_offset = dis_z
        toolhead.set_position(curpos)
        toolhead.wait_moves()

    cmd_GET_OFFSET_help = "Get current offset"
    def cmd_GET_OFFSET(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        msg = f"Z offset: {str(self.z_offset)}, V offset: {str(self.v_offset)}"
        gcode.respond_info(str(msg))

    def _check_collision(self, angle, length, width):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()

        rad = math.radians(- angle)
        y_line = math.tan(rad) * (curpos[0] - (80)) + curpos[2]

        if 0 > y_line:
            raise self.printer.command_error("Prevented collision on A angle")

    # Tu nastane podla mna nejaka sracka ked sa pocita offset zase ked nie je angle 0
    def angle_move(self, new_a_angle, new_b_angle, speed):
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        prev_pos = toolhead.get_position()
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if
                s.is_active_axis('z')]
        change_a = False
        change_b = False

        toolhead.flush_step_generation()
        if new_a_angle != self.a_angle:
            change_a = True
            rad = math.radians(new_a_angle)
            za_offset = (math.tan(rad) * self.length) / 2

            self.invert_dir(z_steppers[1])
            self.invert_dir(z_steppers[2])
            curpos[2] -= za_offset - self.a_offset
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            self.invert_dir(z_steppers[1])
            self.invert_dir(z_steppers[2])

        if new_b_angle != self.b_angle:
            change_b = True
            rad = math.radians(new_b_angle)
            zb_offset = (math.tan(rad) * self.width) / 2

            z_steppers[0].set_trapq(None)
            self.invert_dir(z_steppers[2])
            curpos[2] -= zb_offset - self.b_offset
            toolhead.move(curpos, speed)
            toolhead.flush_step_generation()
            self.invert_dir(z_steppers[2])
            z_steppers[0].set_trapq(toolhead.get_trapq())

        if change_a:
            self.a_angle = new_a_angle
            self.a_offset = za_offset
        if change_b:
            self.b_angle = new_b_angle
            self.b_offset = zb_offset

        toolhead.set_position(prev_pos)

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
        msg = f"A angle: {str(self.a_angle)}, B angle: {str(self.b_angle)}"
        gcode.respond_info(str(msg))
        msg = f"A offset: {str(self.a_offset)}, B offset: {str(self.b_offset)}"
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

            self.angle_move(30, 0, 20, False)
            self.angle_move(0, 0, 20, False)

    def _set_z_currents(self, currents):
        # currents: {stepper_name: run_current}
        gcode = self.printer.lookup_object('gcode')
        for name, cur in currents.items():
            gcode.run_script_from_command(
                "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f" % (name, cur))

    cmd_HOME_ZMAX_help = "Home each Z stepper to max independently using " \
                        "sensorless homing [CURRENT=<amps>] [RETRACT=<mm>] " \
                        "[DEBUG=0|1] [MAX_DIFF=<mm>]"
    def cmd_HOME_ZMAX(self, gcmd):
        homing_current = gcmd.get_float('CURRENT', Z_MAX_HOMING_CURRENT,
                                        above=0.)
        retract_dist = gcmd.get_float('RETRACT', Z_MAX_RETRACT_DIST, minval=0.)
        debug = gcmd.get_int('DEBUG', 0, minval=0, maxval=1)
        max_diff = gcmd.get_float('MAX_DIFF', Z_MAX_MAX_DIFF, above=0.)
        self.HOME_ZMAX(homing_current, retract_dist, debug, max_diff)
    def HOME_ZMAX(self, homing_current, retract_dist, debug=False,
                  max_diff=Z_MAX_MAX_DIFF):
        if self.z_max_errors:
            raise self.printer.command_error("HOME_ZMAX unavailable: %s"
                                     % ("; ".join(self.z_max_errors),))
        if not self.z_max_endstops:
            raise self.printer.command_error(
                "HOME_ZMAX unavailable: no Z steppers found")
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        z_rail = kin.rails[2]
        z_min, z_max = z_rail.get_range()
        speed = z_rail.get_homing_info().speed
        retract_dist = min(retract_dist, z_max - z_min)
        # Reduced run current during the homing move
        prev_currents = {}
        for name, tmc_name in self.z_tmc_names.items():
            tmc = self.printer.lookup_object(tmc_name)
            prev_currents[name] = tmc.get_status()['run_current']
        self._set_z_currents({n: homing_current for n in prev_currents})
        # Pretend Z is at its minimum so a move to max covers full travel
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        pos[2] = z_min
        toolhead.set_position(pos, homing_axes="z")
        movepos = list(pos)
        movepos[2] = z_max
        # All endstops start together; each one halts only its own stepper
        # and the move ends once every endstop has triggered
        hmove = homing.HomingMove(self.printer, self.z_max_endstops)
        monitor = None
        if debug:
            monitor = ZMaxTriggerMonitor(self.printer, self.z_max_endstops,
                                         max_diff, speed)
        try:
            try:
                if monitor is not None:
                    monitor.start()
                hmove.homing_move(movepos, speed)
            except self.printer.command_error:
                if monitor is None or not monitor.aborted:
                    raise
                raise self.printer.command_error(
                    "HOME_ZMAX stopped: %s stalled more than %.2f mm before"
                    " %s (false stall?)\n%s" % (
                        monitor.first, max_diff, ", ".join(monitor.aborted),
                        monitor.format_distances(
                            monitor.get_distances(hmove))))
            finally:
                if monitor is not None:
                    monitor.stop()
                self._set_z_currents(prev_currents)
            if monitor is not None:
                # Also catches a stall that raced the abort timer
                dists = monitor.get_distances(hmove)
                text = monitor.format_distances(dists)
                if dists and max(dists.values()) - min(dists.values()) \
                        > max_diff:
                    raise self.printer.command_error(
                        "HOME_ZMAX: Z motors stalled more than %.2f mm"
                        " apart\n%s" % (max_diff, text))
                gcode = self.printer.lookup_object('gcode')
                gcode.respond_info("HOME_ZMAX trigger distances:\n" + text)
            # Back off from the top stops at normal current
            if retract_dist:
                movepos[2] -= retract_dist
                toolhead.move(movepos, speed)
                toolhead.wait_moves()
        finally:
            # Position is not known relative to the bed, require G28 Z
            kin.clear_homing_state("z")
        # Bed is now flat against the top stops
        self.a_angle = self.b_angle = 0.0
        self.a_offset = self.b_offset = 0.0
        self.z_offset = self.v_offset = 0.0
        self.z_motors_lost = False

    def TEST_ZMAX(self, gcmd):
        homing_current = gcmd.get_float('CURRENT', Z_MAX_HOMING_CURRENT,
                                        above=0.)
        retract_dist = gcmd.get_float('RETRACT', 100., minval=0.)
        debug = gcmd.get_int('DEBUG', 1, minval=0, maxval=1)
        max_diff = gcmd.get_float('MAX_DIFF', Z_MAX_MAX_DIFF, above=0.)


        gcode = self.printer.lookup_object('gcode')
        for i in range(5):
            msg = f"Iteration: {str(i)}"
            gcode.respond_info(str(msg))
            self.HOME_ZMAX(homing_current, retract_dist, debug, max_diff)

def load_config(config):
    return Melt(config)
