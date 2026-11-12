#!/usr/bin/env python3
# Tests for dual_pa on Klipper's own C code (trapq.c, itersolve.c,
# kin_extruder.c). Run:  KLIPPER_DIR=~/klipper python3 tests/test_dual_pa.py
#
# Copyright (C) 2026  VcikeyTheCoder
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import bisect, math, os, subprocess, sys, tempfile, unittest
import cffi

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
KLIPPER_DIR = os.path.expanduser(os.environ.get('KLIPPER_DIR', '~/klipper'))
CHELPER = os.path.join(KLIPPER_DIR, 'klippy', 'chelper')
sys.path.insert(0, os.path.join(KLIPPER_DIR, 'klippy'))
sys.path.insert(0, REPO)
import dual_pa

def build():
    out = os.path.join(tempfile.mkdtemp(), 'test_dual_pa.so')
    src = [os.path.join(CHELPER, f) for f in (
        'trapq.c', 'itersolve.c', 'kin_extruder.c', 'pyhelper.c')]
    src += [os.path.join(REPO, 'dual_pa.c'), os.path.join(HERE, 'sim_helpers.c')]
    subprocess.check_call(['gcc', '-O2', '-Wall', '-shared', '-fPIC',
                           '-I' + CHELPER, '-o', out] + src + ['-lm'])
    ffi = cffi.FFI()
    ffi.cdef("""
    struct trapq *trapq_alloc(void);
    void trapq_append(struct trapq *tq, double print_time
        , double accel_t, double cruise_t, double decel_t
        , double start_pos_x, double start_pos_y, double start_pos_z
        , double axes_r_x, double axes_r_y, double axes_r_z
        , double start_v, double cruise_v, double accel);
    void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq
        , double step_dist);
    int32_t itersolve_generate_steps(struct stepper_kinematics *sk
        , struct stepcompress *sc, double flush_time);
    void itersolve_set_position(struct stepper_kinematics *sk
        , double x, double y, double z);
    struct stepper_kinematics *extruder_stepper_alloc(void);
    void extruder_set_pressure_advance(struct stepper_kinematics *sk
        , double print_time, double pressure_advance, double smooth_time);
    """ + dual_pa.LIB_CDEFS + """
    double test_position(struct stepper_kinematics *sk, struct trapq *tq
        , double t);
    struct stepcompress *test_steps_alloc(void);
    int test_steps_count(struct stepcompress *sc);
    double test_steps_time(struct stepcompress *sc, int i);
    int test_steps_dir(struct stepcompress *sc, int i);
    """)
    return ffi, ffi.dlopen(out)

FFI, LIB = build()

# 0.45 x 0.2 mm line from 1.75 mm filament
RATIO = 0.45 * 0.2 / (math.pi * 1.75 ** 2 / 4.)
ACCEL = 5000.
PUSH, PULL = 0.0578, 0.0493

class Move:
    """The parts of toolhead.Move that PrinterExtruder.process_move uses."""
    def __init__(self, dist, start_v, cruise_v, end_v, accel=ACCEL,
                 ratio=RATIO, xy=True, start_e=0.):
        self.accel = accel
        self.start_v, self.cruise_v, self.end_v = start_v, cruise_v, end_v
        self.accel_t = (cruise_v - start_v) / accel
        self.decel_t = (cruise_v - end_v) / accel
        accel_d = (start_v + cruise_v) * .5 * self.accel_t
        decel_d = (cruise_v + end_v) * .5 * self.decel_t
        self.cruise_t = max(0., (dist - accel_d - decel_d) / cruise_v)
        self.total_t = self.accel_t + self.cruise_t + self.decel_t
        self.axes_r = (1., 0., 0., ratio)
        self.axes_d = (dist if xy else 0., 0., 0., dist * ratio)
        self.ratio, self.dist = ratio, dist
        self.start_pos = (0., 0., 0., start_e)
        self.end_pos = (0., 0., 0., start_e + dist * ratio)
    def e_dist(self):
        return self.dist * self.ratio

class Extruder:
    """One extruder trapq plus stepper kinematics: 'stock' is Klipper's own
    kin_extruder.c fed like PrinterExtruder.process_move, 'dual' is
    dual_pa.c fed by dual_pa.queue_move."""
    def __init__(self, mode, push, pull=None, settle_time=.2,
                 smooth_time=.040, step_dist=1. / 141.):
        self.mode = mode
        self.push, self.pull = push, (push if pull is None else pull)
        self.settle_time = settle_time
        self.tq = LIB.trapq_alloc()
        if mode == 'stock':
            # like ExtruderStepper._set_pressure_advance: no smoothing
            # without pressure advance
            self.sk = LIB.extruder_stepper_alloc()
            LIB.extruder_set_pressure_advance(
                self.sk, 0., push, smooth_time if push else 0.)
        else:
            self.sk = LIB.dual_pa_alloc()
            LIB.dual_pa_set_smooth_time(
                self.sk, smooth_time if (self.push or self.pull) else 0.)
        LIB.itersolve_set_trapq(self.sk, self.tq, step_dist)
        self.step_dist = step_dist
        self.state = dual_pa.AdvanceState()
        self.e = 0.
        self.time = 1.
        self.stops = []
    def add(self, move, gap=0.):
        self.time += gap
        move.start_pos = (0., 0., 0., self.e)
        if self.mode == 'stock':
            r = move.axes_r[3]
            can_pa = r > 0. and (move.axes_d[0] or move.axes_d[1])
            LIB.trapq_append(self.tq, self.time, move.accel_t, move.cruise_t,
                             move.decel_t, self.e, 0., 0., 1., float(can_pa),
                             0., move.start_v * r, move.cruise_v * r,
                             move.accel * r)
        else:
            dual_pa.queue_move(self.state, LIB.trapq_append, self.tq,
                               self.time, move, 3, self.push, self.pull,
                               self.settle_time)
        self.time += move.total_t
        self.e += move.e_dist()
        if move.end_v == 0.:
            self.stops.append((self.time, self.e))
    def run(self, moves):
        for m in moves:
            self.add(m)
        return self
    def pos(self, t):
        return LIB.test_position(self.sk, self.tq, t)
    def steps(self):
        sc = LIB.test_steps_alloc()
        LIB.itersolve_set_position(self.sk, 0., 0., 0.)
        self.pos(0.)
        t, err = 0.9, 0
        while t < self.time + .2:
            t += .025
            err = err or LIB.itersolve_generate_steps(self.sk, sc, t)
        n = LIB.test_steps_count(sc)
        return err, [(LIB.test_steps_time(sc, i),
                      1 if LIB.test_steps_dir(sc, i) else -1)
                     for i in range(n)]

def times(ext, step=.0002):
    t = .95
    while t < ext.time + .1:
        yield t
        t += step

def nominal_speed(moves, t, start=1.):
    """Filament speed (mm/s) at print time t for moves queued back to back"""
    if t < start:
        return 0.
    for m in moves:
        if t < start + m.total_t:
            u = t - start
            if u < m.accel_t:
                v = m.start_v + m.accel * u
            elif u < m.accel_t + m.cruise_t:
                v = m.cruise_v
            else:
                v = m.cruise_v - m.accel * (u - m.accel_t - m.cruise_t)
            return v * m.ratio
        start += m.total_t
    return 0.

def scenarios():
    jv = 1. / RATIO          # junction speed for a 1 mm/s extruder jump
    return {
        'line': [Move(20, 0, 200, 0)],
        'corner': [Move(20, 0, 200, 50), Move(20, 50, 200, 0)],
        'long corner': [Move(60, 0, 200, 50), Move(60, 50, 200, 0)],
        'arc': ([Move(10, 0, 100, 100)] + [Move(2, 100, 100, 100)] * 30
                + [Move(10, 100, 100, 0)]),
        'extrude then travel': [Move(20, 0, 200, jv),
                                Move(30, jv, 300, 0, ratio=0.)],
        'retract and unretract': [
            Move(20, 0, 200, 0), Move(0.8, 0, 35, 0, ratio=-1., xy=False),
            Move(30, 0, 300, 0, ratio=0.),
            Move(0.8, 0, 35, 0, ratio=1., xy=False), Move(20, 0, 200, 0)],
        'short moves': [Move(1., 0, math.sqrt(ACCEL), 0)] * 6,
        'speed changes': [Move(10, 0, 60, 60), Move(30, 60, 250, 30),
                          Move(10, 30, 120, 0)],
    }


class TestDualPA(unittest.TestCase):
    def test_same_values_match_klipper(self):
        """push = pull gives exactly Klipper's own pressure advance"""
        for name, moves in scenarios().items():
            for k in (0., 0.02, PUSH):
                for settle in (0., .2):
                    a = Extruder('stock', k).run(moves)
                    b = Extruder('dual', k, k, settle).run(moves)
                    worst = max(abs(a.pos(t) - b.pos(t)) for t in times(a))
                    self.assertLess(worst, 1e-9, (name, k, settle, worst))

    def test_push_while_speeding_up_pull_while_slowing_down(self):
        """With settling done, speeding up is Klipper at push and slowing
        down is Klipper at pull"""
        moves = scenarios()['long corner']
        d = Extruder('dual', PUSH, PULL, .2).run(moves)
        sp = Extruder('stock', PUSH).run(moves)
        sl = Extruder('stock', PULL).run(moves)
        hst = .02
        t0 = 1.
        m1, m2 = moves
        # speeding up from a stop: same as Klipper at push
        for t in self._span(t0 - hst, t0 + m1.accel_t - hst):
            self.assertLess(abs(d.pos(t) - sp.pos(t)), 1e-9)
        # both slow-downs: same speed as Klipper at pull
        for start, m in ((t0, m1), (t0 + m1.total_t, m2)):
            a = start + m.accel_t + m.cruise_t + hst
            b = start + m.total_t - hst
            for t in self._span(a, b):
                ds = (d.pos(t + 1e-4) - d.pos(t - 1e-4)) / 2e-4
                ls = (sl.pos(t + 1e-4) - sl.pos(t - 1e-4)) / 2e-4
                self.assertLess(abs(ds - ls), 1e-6, (t, ds, ls))
        # the final stop ends exactly like Klipper at pull
        end = t0 + m1.total_t + m2.total_t
        for t in self._span(end - hst, end + .05):
            self.assertLess(abs(d.pos(t) - sl.pos(t)), 1e-9)

    def test_settle(self):
        """While cruising the advance moves to pull * speed in settle_time:
        the flow is |push - pull| * speed / settle_time lower, then exact"""
        moves = [Move(120, 0, 200, 0)]
        for settle in (.1, .2, .4):
            d = Extruder('dual', PUSH, PULL, settle).run(moves)
            m = moves[0]
            v = 200 * RATIO
            want = -(PUSH - PULL) * v / settle
            a = 1. + m.accel_t + .021
            for t in self._span(a, a + settle - .045):
                s = (d.pos(t + 1e-4) - d.pos(t - 1e-4)) / 2e-4 - v
                self.assertLess(abs(s - want), 1e-6, (settle, t, s, want))
            for t in self._span(a + settle + .001, 1. + m.accel_t
                                + m.cruise_t - .021):
                s = (d.pos(t + 1e-4) - d.pos(t - 1e-4)) / 2e-4 - v
                self.assertLess(abs(s), 1e-6, (settle, t, s))

    def test_filament_exact(self):
        """After every stop the extruder is where the G-code puts it"""
        for name, moves in scenarios().items():
            for push, pull in ((PUSH, PULL), (0.03, 0.08), (0.08, 0.)):
                d = Extruder('dual', push, pull, .2)
                for m in moves:
                    d.add(m, gap=.1)
                for t, e in d.stops:
                    self.assertLess(abs(d.pos(t + .03) - e), 1e-9,
                                    (name, push, pull))
                self.assertLess(abs(d.pos(d.time + .05) - d.e), 1e-9)

    def test_idle_gap(self):
        """A move after the extruder was idle starts like Klipper's"""
        a = Extruder('stock', PUSH)
        b = Extruder('dual', PUSH, PULL, .2)
        for ext in (a, b):
            ext.add(Move(20, 0, 200, 0))
            ext.add(Move(20, 0, 200, 0), gap=.5)
        t0 = 1. + Move(20, 0, 200, 0).total_t + .5
        for t in self._span(t0 - .03, t0 + .02):
            self.assertLess(abs(a.pos(t) - b.pos(t)), 1e-9)

    def test_no_advance_moves(self):
        """Retracts, unretracts and extrude only moves get no advance"""
        moves = [Move(0.8, 0, 35, 0, ratio=-1., xy=False),
                 Move(0.8, 0, 35, 0, ratio=1., xy=False),
                 Move(5, 0, 5, 0, ratio=1., xy=False)]
        a = Extruder('stock', PUSH).run(moves)
        b = Extruder('dual', PUSH, PULL, .2).run(moves)
        worst = max(abs(a.pos(t) - b.pos(t)) for t in times(a))
        self.assertLess(worst, 1e-9)

    def test_steps(self):
        """Klipper's step solver: no errors, steps in time order, never
        more than half a step from the position, no extra direction
        changes compared with Klipper at push"""
        for name, moves in scenarios().items():
            for step_dist in (1. / 141., 1. / 415.):
                d = Extruder('dual', PUSH, PULL, .2,
                             step_dist=step_dist).run(moves)
                s = Extruder('stock', PUSH, step_dist=step_dist).run(moves)
                err, steps = d.steps()
                serr, ssteps = s.steps()
                self.assertEqual((err, serr), (0, 0), name)
                st = [x[0] for x in steps]
                self.assertEqual(st, sorted(st), name)
                pos, cum = 0, []
                for _, dr in steps:
                    pos += dr
                    cum.append(pos)
                worst = 0.
                for t in times(d, .0005):
                    i = bisect.bisect_right(st, t)
                    have = cum[i - 1] if i else 0
                    worst = max(worst, abs(have - d.pos(t) / step_dist))
                self.assertLessEqual(worst, .5 + 1e-6, (name, worst))
                turns = lambda x: sum(1 for i in range(1, len(x))
                                      if x[i][1] != x[i - 1][1])
                self.assertLessEqual(turns(steps), turns(ssteps) + 2, name)

    def test_short_cruise_leftover(self):
        """What has not settled when the slow-down starts is let go at
        the start of the slow-down"""
        st = dual_pa.AdvanceState()
        v = 200 * RATIO
        cruise = .05
        out = dual_pa.plan_move(st, 1., .04, cruise, .04, 0., 0., v,
                                ACCEL * RATIO, PUSH, PULL, .2)
        settle = out[1]
        self.assertAlmostEqual(settle[2], cruise)
        left = settle[5] + settle[6] * cruise - PULL * v
        self.assertAlmostEqual(left, (PUSH - PULL) * v * (1 - cruise / .2))
        self.assertAlmostEqual(out[-1][5], PULL * v)
        self.assertAlmostEqual(st.advance, 0.)

    @staticmethod
    def _span(a, b, step=.0005):
        t = a
        while t < b:
            yield t
            t += step

if __name__ == '__main__':
    unittest.main(verbosity=2)
