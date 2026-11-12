# Dual pressure advance: one advance while the extruder speeds up, another
# while it slows down
#
# Copyright (C) 2026  VcikeyTheCoder
# Based on klippy/kinematics/extruder.py from Klipper
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, os, subprocess
try:
    import chelper
except ImportError:
    from klippy import chelper

LIB_SOURCE = 'dual_pa.c'
LIB_NAME = 'libdual_pa.so'
LIB_HEADERS = ['compiler.h', 'itersolve.h', 'list.h', 'trapq.h']
LIB_CDEFS = """
    struct stepper_kinematics *dual_pa_alloc(void);
    void dual_pa_free(struct stepper_kinematics *sk);
    void dual_pa_set_smooth_time(struct stepper_kinematics *sk
        , double smooth_time);
    void dual_pa_copy_state(struct stepper_kinematics *dst
        , struct stepper_kinematics *src);
"""
COMPILE_CMD = "gcc -Wall -O2 -shared -fPIC -I%s -o %s %s"


######################################################################
# Advance planning (no Klipper objects, so it can be tested on its own)
######################################################################

class AdvanceState:
    def __init__(self):
        self.advance = 0.       # filament (mm) ahead of the nominal position
        self.end_time = -1.     # print time the last queued phase ends

# Split one extruder move into phases with their advance.
# Returns (print_time, accel_t, cruise_t, decel_t, start_e, advance, slope,
#          start_v, cruise_v, accel) for each phase, where the advance is
# advance(t) = advance + slope * t during the phase.
#   speeding up:   slope = push * filament accel
#   steady speed:  the advance moves from where it is to pull * speed,
#                  at |push - pull| * speed / settle_time
#   slowing down:  starts at pull * speed (anything not settled goes now),
#                  slope = -pull * filament accel, ends at pull * end speed
def plan_move(state, print_time, accel_t, cruise_t, decel_t, start_e,
              start_v, cruise_v, accel, push, pull, settle_time):
    if print_time > state.end_time + 0.000001:
        # Extruder was idle: start as Klipper does (advance = K * speed)
        state.advance = push * start_v
    adv = state.advance
    out = []
    t, e = print_time, start_e
    if accel_t:
        slope = push * accel
        out.append((t, accel_t, 0., 0., e, adv, slope,
                    start_v, cruise_v, accel))
        adv += slope * accel_t
        e += (start_v + .5 * accel * accel_t) * accel_t
        t += accel_t
    if cruise_t:
        target = pull * cruise_v
        diff = target - adv
        settle_t = 0.
        if abs(diff) > 1e-9 and settle_time > 0.:
            rate = abs(push - pull) * cruise_v / settle_time
            if rate <= 0.:
                rate = abs(diff) / settle_time
            settle_t = min(abs(diff) / rate, cruise_t)
            slope = math.copysign(rate, diff)
            out.append((t, 0., settle_t, 0., e, adv, slope,
                        cruise_v, cruise_v, accel))
            adv += slope * settle_t
            e += cruise_v * settle_t
            t += settle_t
        if settle_t < cruise_t:
            # Settled (or settle_time is 0): hold pull * speed
            adv = target
            rest = cruise_t - settle_t
            out.append((t, 0., rest, 0., e, adv, 0.,
                        cruise_v, cruise_v, accel))
            e += cruise_v * rest
            t += rest
    if decel_t:
        adv = pull * cruise_v
        slope = -pull * accel
        out.append((t, 0., 0., decel_t, e, adv, slope,
                    cruise_v, cruise_v, accel))
        adv += slope * decel_t
        t += decel_t
    state.advance = adv
    state.end_time = t
    return out


# Queue one toolhead move on the extruder trapq (PrinterExtruder.process_move)
def queue_move(state, append, trapq, print_time, move, ea_index,
               push, pull, settle_time):
    axis_r = move.axes_r[ea_index]
    accel = move.accel * axis_r
    start_v = move.start_v * axis_r
    cruise_v = move.cruise_v * axis_r
    start_e = move.start_pos[ea_index]
    if (axis_r > 0. and (move.axes_d[0] or move.axes_d[1])
            and (push or pull)):
        for (t, accel_t, cruise_t, decel_t, e, adv, slope,
             sv, cv, acc) in plan_move(state, print_time, move.accel_t,
                                       move.cruise_t, move.decel_t, start_e,
                                       start_v, cruise_v, accel,
                                       push, pull, settle_time):
            append(trapq, t, accel_t, cruise_t, decel_t, e, adv, 0.,
                   1., slope, 0., sv, cv, acc)
        return
    # Retract, unretract, extrude only: no advance (as Klipper)
    append(trapq, print_time, move.accel_t, move.cruise_t, move.decel_t,
           start_e, 0., 0., 1., 0., 0., start_v, cruise_v, accel)
    state.advance = 0.
    state.end_time = print_time + move.accel_t + move.cruise_t + move.decel_t


######################################################################
# C helper
######################################################################

def _mtime(path):
    try:
        return os.path.getmtime(path)
    except os.error:
        return 0.

def _load_lib():
    srcdir = os.path.dirname(os.path.realpath(__file__))
    src = os.path.join(srcdir, LIB_SOURCE)
    dest = os.path.join(srcdir, LIB_NAME)
    incdir = os.path.dirname(os.path.realpath(chelper.__file__))
    deps = [src] + [os.path.join(incdir, h) for h in LIB_HEADERS]
    if _mtime(dest) < max(_mtime(d) for d in deps):
        logging.info("dual_pa: building %s", LIB_NAME)
        tmp = os.path.join(srcdir, "_temp_" + LIB_NAME)
        cmd = COMPILE_CMD % (incdir, tmp, src)
        res = subprocess.run(cmd.split(), stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
        if res.returncode:
            raise Exception("dual_pa: unable to build %s:\n%s\n%s"
                            % (LIB_NAME, cmd, res.stdout.decode(
                                errors='replace')))
        os.rename(tmp, dest)
    ffi_main, ffi_lib = chelper.get_ffi()
    if not getattr(ffi_main, '_dual_pa_cdefs', False):
        ffi_main.cdef(LIB_CDEFS)
        ffi_main._dual_pa_cdefs = True
    return ffi_main, ffi_main.dlopen(dest)


######################################################################
# Klipper interface
######################################################################

class DualPressureAdvance:
    def __init__(self, config):
        self.printer = config.get_printer()
        parts = config.get_name().split()
        self.extruder_name = parts[1] if len(parts) > 1 else 'extruder'
        self.push = config.getfloat('push_pressure_advance', None, minval=0.)
        self.pull = config.getfloat('pull_pressure_advance', None, minval=0.)
        self.settle_time = config.getfloat('settle_time', .2, minval=0.,
                                           maxval=2.)
        self.smooth_time = config.getfloat('smooth_time', None, above=0.,
                                           maxval=.200)
        self.extruder = self.toolhead = None
        self.ffi_main = self.lib = None
        self.state = AdvanceState()
        self.steppers = {}      # ExtruderStepper -> dual_pa kinematics
        self.hooked = []        # ExtruderSteppers with wrapped methods
        self.active_smooth = 0.
        self.ready = False
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        gcode = self.printer.lookup_object('gcode')
        if self.extruder_name == 'extruder':
            gcode.register_mux_command("SET_DUAL_PA", "EXTRUDER", None,
                                       self.cmd_SET_DUAL_PA,
                                       desc=self.cmd_SET_DUAL_PA_help)
        gcode.register_mux_command("SET_DUAL_PA", "EXTRUDER",
                                   self.extruder_name, self.cmd_SET_DUAL_PA,
                                   desc=self.cmd_SET_DUAL_PA_help)
    # Setup
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        extruder = self.printer.lookup_object(self.extruder_name, None)
        if extruder is None or not hasattr(extruder, 'trapq_append'):
            raise self.printer.config_error(
                "[dual_pa]: '%s' is not an extruder" % (self.extruder_name,))
        es = extruder.extruder_stepper
        if es is None:
            raise self.printer.config_error(
                "[dual_pa]: '%s' has no stepper" % (self.extruder_name,))
        self.extruder = extruder
        self.ffi_main, self.lib = _load_lib()
        if self.push is None:
            self.push = es.config_pa
        if self.pull is None:
            self.pull = es.config_pa
        if self.smooth_time is None:
            self.smooth_time = es.config_smooth_time
        extruder.process_move = self._process_move
        for es in self._extruder_steppers():
            self._hook(es)
            if es.stepper.get_trapq() is extruder.get_trapq():
                self._attach(es)
        self._update_smooth_time()
        self._sync_status()
    def _handle_ready(self):
        # From here on SET_PRESSURE_ADVANCE sets push and pull. Before
        # this, the extruder's own connect handler applies its config
        # value, which must not replace the [dual_pa] values.
        self.ready = True
    def _extruder_steppers(self):
        out = [self.extruder.extruder_stepper]
        for name, obj in self.printer.lookup_objects('extruder_stepper'):
            es = getattr(obj, 'extruder_stepper', None)
            if es is not None and es not in out:
                out.append(es)
        return out
    def _hook(self, es):
        if es in self.hooked:
            return
        self.hooked.append(es)
        orig_sync = es.sync_to_extruder
        orig_set_pa = es._set_pressure_advance
        def sync_to_extruder(extruder_name):
            orig_sync(extruder_name)
            if es.stepper.get_trapq() is self.extruder.get_trapq():
                self._attach(es)
            else:
                self._detach(es)
        def set_pressure_advance(pressure_advance, smooth_time):
            orig_set_pa(pressure_advance, smooth_time)
            if es is self.extruder.extruder_stepper and self.ready:
                # SET_PRESSURE_ADVANCE: one advance for push and pull
                self._set_values(pressure_advance, pressure_advance,
                                 self.settle_time, smooth_time)
        es.sync_to_extruder = sync_to_extruder
        es._set_pressure_advance = set_pressure_advance
    def _attach(self, es):
        if es in self.steppers:
            return
        self.toolhead.flush_step_generation()
        sk = self.ffi_main.gc(self.lib.dual_pa_alloc(), self.lib.dual_pa_free)
        self.lib.dual_pa_set_smooth_time(sk, self.active_smooth)
        self.lib.dual_pa_copy_state(sk, es.stepper.get_stepper_kinematics())
        es.stepper.set_stepper_kinematics(sk)
        self.steppers[es] = sk
        self._check_scan_windows()
    def _detach(self, es):
        if es not in self.steppers:
            return
        self.toolhead.flush_step_generation()
        self.lib.dual_pa_copy_state(es.sk_extruder, self.steppers[es])
        es.stepper.set_stepper_kinematics(es.sk_extruder)
        del self.steppers[es]
        self._check_scan_windows()
    def _check_scan_windows(self):
        mq = self.printer.lookup_object('motion_queuing', None)
        if mq is not None:
            mq.check_step_generation_scan_windows()
    def _update_smooth_time(self):
        smooth = self.smooth_time if (self.push or self.pull) else 0.
        if smooth == self.active_smooth:
            return
        old = self.active_smooth
        self.toolhead.flush_step_generation()
        for sk in self.steppers.values():
            self.lib.dual_pa_set_smooth_time(sk, smooth)
        self.active_smooth = smooth
        mq = self.printer.lookup_object('motion_queuing', None)
        if mq is not None:
            mq.check_step_generation_scan_windows()
        elif hasattr(self.toolhead, 'note_step_generation_scan_time'):
            self.toolhead.note_step_generation_scan_time(
                smooth * .5, old_delay=old * .5)
    def _set_values(self, push, pull, settle_time, smooth_time):
        old_smooth = self.smooth_time
        def apply(print_time):
            self.push, self.pull = push, pull
            self.settle_time = settle_time
        if smooth_time != old_smooth or (
                bool(push or pull) != bool(self.push or self.pull)):
            # Smoothing changes need a full flush, like Klipper
            self.toolhead.flush_step_generation()
            apply(0.)
            self.smooth_time = smooth_time
            self._update_smooth_time()
        else:
            self.toolhead.register_lookahead_callback(apply)
        self._sync_status(push, pull, smooth_time)
    def _sync_status(self, push=None, pull=None, smooth=None):
        # Keep the extruder status (shown by Mainsail / Fluidd) close to
        # the truth: it reports the pull value
        es = self.extruder.extruder_stepper
        es.pressure_advance = self.pull if pull is None else pull
        es.pressure_advance_smooth_time = (
            self.smooth_time if smooth is None else smooth)
    # Move queuing (replaces PrinterExtruder.process_move)
    def _process_move(self, print_time, move, ea_index):
        extruder = self.extruder
        queue_move(self.state, extruder.trapq_append, extruder.trapq,
                   print_time, move, ea_index,
                   self.push, self.pull, self.settle_time)
        extruder.last_position = move.end_pos[ea_index]
    # Status and commands
    def get_status(self, eventtime):
        return {'push_pressure_advance': self.push,
                'pull_pressure_advance': self.pull,
                'settle_time': self.settle_time,
                'smooth_time': self.smooth_time,
                'steppers': len(self.steppers)}
    cmd_SET_DUAL_PA_help = "Set dual pressure advance parameters"
    def cmd_SET_DUAL_PA(self, gcmd):
        if self.lib is None:
            raise gcmd.error("dual_pa is not active")
        push = gcmd.get_float('PUSH', self.push, minval=0.)
        pull = gcmd.get_float('PULL', self.pull, minval=0.)
        settle_time = gcmd.get_float('SETTLE_TIME', self.settle_time,
                                     minval=0., maxval=2.)
        smooth_time = gcmd.get_float('SMOOTH_TIME', self.smooth_time,
                                     above=0., maxval=.200)
        self._set_values(push, pull, settle_time, smooth_time)
        msg = ("push_pressure_advance: %.6f\n"
               "pull_pressure_advance: %.6f\n"
               "settle_time: %.3f\n"
               "smooth_time: %.6f" % (push, pull, settle_time, smooth_time))
        self.printer.set_rollover_info(
            "dual_pa %s" % (self.extruder_name,), "dual_pa: %s" % (msg,))
        gcmd.respond_info(msg, log=False)

def load_config(config):
    return DualPressureAdvance(config)

def load_config_prefix(config):
    return DualPressureAdvance(config)
