#!/usr/bin/env python3
# Draw the graphs in docs/images from Klipper's own C code and dual_pa.c
#   KLIPPER_DIR=~/klipper python3 scripts/make_graphs.py
#
# Copyright (C) 2026  VcikeyTheCoder
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'tests'))
import test_dual_pa as T

OUT = os.path.join(REPO, 'docs', 'images')
PUSH, PULL = T.PUSH, T.PULL
INK, MUTED, GRID = '#222222', '#6b6b6b', '#e6e6e6'
C_PUSH, C_PULL, C_DUAL, C_NONE = '#c4553d', '#7d8794', '#1f7a6d', '#b9b9b9'
plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10.5, 'axes.edgecolor': '#bbbbbb',
    'axes.labelcolor': INK, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.titlesize': 12, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left',
    'legend.frameon': False, 'figure.facecolor': 'white'})

def style(ax):
    ax.grid(color=GRID, lw=.8)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(length=0)

def series(ext, ts):
    pos = np.array([ext.pos(t) for t in ts])
    return pos, np.gradient(pos, ts)

def shade(ax, moves, t0=1.):
    t = t0
    for m in moves:
        ax.axvspan((t - t0) * 1000, (t + m.accel_t - t0) * 1000,
                   color=C_PUSH, alpha=.07, lw=0)
        d0 = t + m.accel_t + m.cruise_t
        ax.axvspan((d0 - t0) * 1000, (t + m.total_t - t0) * 1000,
                   color=C_PULL, alpha=.10, lw=0)
        t += m.total_t

def corner():
    moves = T.scenarios()['long corner']
    d = T.Extruder('dual', PUSH, PULL, .2).run(moves)
    sp = T.Extruder('stock', PUSH).run(moves)
    sl = T.Extruder('stock', PULL).run(moves)
    ts = np.arange(.96, d.time + .06, 1e-4)
    ms = (ts - 1.) * 1000
    nominal = np.array([T.nominal_speed(moves, t) for t in ts])
    e_nom = np.concatenate([[0.], np.cumsum((nominal[1:] + nominal[:-1]) * .5e-4)])
    fig, axs = plt.subplots(3, 1, figsize=(10, 9.6), sharex=True,
                            gridspec_kw={'height_ratios': [1, 1.4, 1.4]})
    ax = axs[0]
    style(ax); shade(ax, moves)
    ax.plot(ms, nominal / T.RATIO, color=INK, lw=1.6)
    ax.set_ylabel('toolhead speed (mm/s)')
    ax.set_title('Move: 0 to 200 mm/s, corner at 50 mm/s, stop')
    ax.text(5, 215, 'push', color=C_PUSH, fontsize=9.5)
    ax.text(302, 215, 'pull', color=C_PULL, fontsize=9.5)
    ax.set_ylim(-5, 240)
    ax = axs[1]
    style(ax); shade(ax, moves)
    for ext, col, ls, lab in ((sp, C_PUSH, '--', 'Klipper, pressure_advance %.4f' % PUSH),
                              (sl, C_PULL, '--', 'Klipper, pressure_advance %.4f' % PULL),
                              (d, C_DUAL, '-', 'dual_pa, push %.4f / pull %.4f' % (PUSH, PULL))):
        pos, _ = series(ext, ts)
        ax.plot(ms, (pos - e_nom) * 1000, color=col,
                ls=ls, lw=2.2 if ls == '-' else 1.4, label=lab)
    ax.set_ylabel('advance (µm of filament)')
    ax.set_title('Advance: filament pushed ahead of the nominal position')
    ax.legend(loc='lower center', fontsize=9.5)
    ax = axs[2]
    style(ax); shade(ax, moves)
    ax.plot(ms, nominal, color=C_NONE, lw=1.4, label='no pressure advance')
    for ext, col, ls, lab in ((sp, C_PUSH, '--', 'Klipper, push value'),
                              (sl, C_PULL, '--', 'Klipper, pull value'),
                              (d, C_DUAL, '-', 'dual_pa')):
        _, spd = series(ext, ts)
        ax.plot(ms, spd, color=col, ls=ls, lw=2.2 if ls == '-' else 1.4,
                label=lab)
    ax.set_ylabel('extruder speed (mm/s)')
    ax.set_xlabel('time (ms)')
    ax.set_title('Extruder speed')
    ax.legend(loc='upper right', fontsize=9.5, ncol=2)
    ax.set_xlim(ms[0], ms[-1])
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'dual_pa_corner.png'), dpi=150)

def settle():
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 4.2))
    moves = [T.Move(120, 0, 200, 0)]
    m = moves[0]
    v = 200 * T.RATIO
    style(a)
    for st, col in ((.1, C_PUSH), (.2, C_DUAL), (.4, '#3c6fb0')):
        d = T.Extruder('dual', PUSH, PULL, st).run(moves)
        ts = np.arange(1. + m.accel_t + .025, 1. + m.accel_t + m.cruise_t - .025, 5e-4)
        _, spd = series(d, ts)
        a.plot((ts - 1.) * 1000, (spd - v) / v * 100, color=col, lw=2,
               label='settle_time: %.1f' % st)
    a.axhline(0, color=C_PULL, lw=1.2, ls='--', label='Klipper')
    a.set_ylim(-10, 2)
    a.set_xlabel('time (ms)')
    a.set_ylabel('flow vs G-code (%)')
    a.set_title('Flow after speeding up to 200 mm/s')
    a.legend(loc='lower right', fontsize=9.5)
    style(b)
    tc = np.linspace(0, .5, 300)
    full = (PUSH - PULL) * v * 1000
    for st, col in ((.1, C_PUSH), (.2, C_DUAL), (.4, '#3c6fb0')):
        b.plot(tc * 1000, full * np.clip(1 - tc / st, 0, 1), color=col, lw=2,
               label='settle_time: %.1f' % st)
    b.axhline(full, color=C_PULL, lw=1.2, ls='--', label='settle_time: 0')
    b.set_xlabel('steady speed before slowing down (ms)')
    b.set_ylabel('let go at the slow-down (µm of filament)')
    b.set_title('Short lines')
    b.legend(loc='upper right', fontsize=9.5)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'dual_pa_settle.png'), dpi=150)

if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    corner()
    settle()
