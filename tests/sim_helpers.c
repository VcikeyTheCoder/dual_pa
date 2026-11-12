// Test helpers: evaluate a stepper position at a print time, and record the
// steps Klipper's step solver (itersolve.c) produces instead of compressing
// them for an MCU.
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "compiler.h"
#include "itersolve.h"
#include "list.h"
#include "trapq.h"

double __visible
test_position(struct stepper_kinematics *sk, struct trapq *tq, double t)
{
    trapq_check_sentinels(tq);
    struct move *m;
    list_for_each_entry(m, &tq->moves, node) {
        if (t >= m->print_time && t < m->print_time + m->move_t)
            return sk->calc_position_cb(sk, m, t - m->print_time);
    }
    return 0.;
}

struct stepcompress { int count, size; double *times; int *dirs; };

struct stepcompress * __visible
test_steps_alloc(void)
{
    return calloc(1, sizeof(struct stepcompress));
}

int
stepcompress_append(struct stepcompress *sc, int sdir
                    , double print_time, double step_time)
{
    if (sc->count == sc->size) {
        sc->size = sc->size ? 2 * sc->size : 4096;
        sc->times = realloc(sc->times, sc->size * sizeof(double));
        sc->dirs = realloc(sc->dirs, sc->size * sizeof(int));
    }
    sc->times[sc->count] = print_time + step_time;
    sc->dirs[sc->count] = sdir;
    sc->count++;
    return 0;
}

int stepcompress_commit(struct stepcompress *sc) { return 0; }

int
stepcompress_get_step_dir(struct stepcompress *sc)
{
    return sc->count ? sc->dirs[sc->count - 1] : 0;
}

int __visible test_steps_count(struct stepcompress *sc) { return sc->count; }
double __visible test_steps_time(struct stepcompress *sc, int i)
{ return sc->times[i]; }
int __visible test_steps_dir(struct stepcompress *sc, int i)
{ return sc->dirs[i]; }
