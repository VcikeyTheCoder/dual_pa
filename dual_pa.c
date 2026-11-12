// Dual pressure advance: extruder position with a per move advance
//
// Copyright (C) 2026  VcikeyTheCoder
// Based on klippy/chelper/kin_extruder.c from Klipper
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Klipper's extruder kinematics compute
//     pa_position(t) = nominal_position(t) + pressure_advance * nominal_velocity(t)
// with one pressure_advance for all moves. Here the host (dual_pa.py) works
// out the advance for each move phase and stores it in the move itself:
//     advance(t) = start_pos.y + axes_r.y * t
// (start_pos.y = advance at the start of the phase, axes_r.y = its change
// per second). Moves with axes_r.x == 0 (gap fillers and sentinels) have no
// advance. The result is smoothed exactly like Klipper does it:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * (smooth_time/2 - abs(t-x)) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / ((smooth_time/2)**2))

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "list.h" // list_node
#include "trapq.h" // struct move

// Definitive integral of: position(t) = base + t * (start_v + t * half_accel)
static double
dpa_integrate(double base, double start_v, double half_accel
              , double start, double end)
{
    double half_v = .5 * start_v, sixth_a = (1. / 3.) * half_accel;
    double si = start * (base + start * (half_v + start * sixth_a));
    double ei = end * (base + end * (half_v + end * sixth_a));
    return ei - si;
}

// Definitive integral of: t * (base + t * (start_v + t * half_accel))
static double
dpa_integrate_time(double base, double start_v, double half_accel
                   , double start, double end)
{
    double half_b = .5 * base, third_v = (1. / 3.) * start_v;
    double eighth_a = .25 * half_accel;
    double si = start * start * (half_b + start * (third_v + start * eighth_a));
    double ei = end * end * (half_b + end * (third_v + end * eighth_a));
    return ei - si;
}

// Weighted integral of the extruder position (with advance) over one move
static double
dpa_move_integrate(struct move *m, double base, double start, double end
                   , double time_offset)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    double start_v = m->start_v;
    if (m->axes_r.x) {
        base += m->start_pos.y;
        start_v += m->axes_r.y;
    }
    double ha = m->half_accel;
    double iext = dpa_integrate(base, start_v, ha, start, end);
    double wgt_ext = dpa_integrate_time(base, start_v, ha, start, end);
    return wgt_ext - time_offset * iext;
}

// Weighted integral over the moves inside the smoothing window
static double
dpa_range_integrate(struct move *m, double move_time, double hst)
{
    double res = 0., start = move_time - hst, end = move_time + hst;
    double start_base = m->start_pos.x;
    res += dpa_move_integrate(m, 0., start, move_time, start);
    res -= dpa_move_integrate(m, 0., move_time, end, end);
    struct move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        double base = prev->start_pos.x - start_base;
        res += dpa_move_integrate(prev, base, start, prev->move_t, start);
    }
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        m = list_next_entry(m, node);
        double base = m->start_pos.x - start_base;
        res -= dpa_move_integrate(m, base, 0., end, end);
    }
    return res;
}

// Same as move_get_distance() in trapq.c (that symbol is not exported by
// Klipper's c_helper.so, so this library has its own copy)
static inline double
dpa_move_distance(struct move *m, double move_time)
{
    return (m->start_v + m->half_accel * move_time) * move_time;
}

struct dual_pa_stepper {
    struct stepper_kinematics sk;
    double half_smooth_time, inv_half_smooth_time2;
};

static double
dual_pa_calc_position(struct stepper_kinematics *sk, struct move *m
                      , double move_time)
{
    struct dual_pa_stepper *ds = container_of(sk, struct dual_pa_stepper, sk);
    double hst = ds->half_smooth_time;
    if (!hst)
        // Pressure advance not enabled
        return m->start_pos.x + dpa_move_distance(m, move_time);
    double area = dpa_range_integrate(m, move_time, hst);
    return m->start_pos.x + area * ds->inv_half_smooth_time2;
}

// Take over the step generation state of the kinematics being replaced, so
// the new one continues from the same position and time (and does not
// generate steps for moves that were already stepped)
void __visible
dual_pa_copy_state(struct stepper_kinematics *dst
                   , struct stepper_kinematics *src)
{
    dst->commanded_pos = src->commanded_pos;
    dst->last_flush_time = src->last_flush_time;
    dst->last_move_time = src->last_move_time;
}

void __visible
dual_pa_set_smooth_time(struct stepper_kinematics *sk, double smooth_time)
{
    struct dual_pa_stepper *ds = container_of(sk, struct dual_pa_stepper, sk);
    double hst = smooth_time * .5;
    ds->half_smooth_time = hst;
    ds->sk.gen_steps_pre_active = ds->sk.gen_steps_post_active = hst;
    ds->inv_half_smooth_time2 = hst ? 1. / (hst * hst) : 0.;
}

struct stepper_kinematics * __visible
dual_pa_alloc(void)
{
    struct dual_pa_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->sk.calc_position_cb = dual_pa_calc_position;
    ds->sk.active_flags = AF_X;
    return &ds->sk;
}

void __visible
dual_pa_free(struct stepper_kinematics *sk)
{
    struct dual_pa_stepper *ds = container_of(sk, struct dual_pa_stepper, sk);
    free(ds);
}
