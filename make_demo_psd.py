"""Generate a clearly synthetic fifth-harmonic example for the Results page.

This file only creates a static SVG. It never connects to an ODrive or writes
to the measured results directory.
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from AllTests import psd


ROOT = Path(__file__).resolve().parent
SAMPLE_RATE_HZ = 8000
SAMPLE_COUNT = 8000
RPM = 600
POLE_PAIRS = 3
FUNDAMENTAL_HZ = RPM / 60 * POLE_PAIRS
FIFTH_HZ = 5 * FUNDAMENTAL_HZ
PHASE_OFFSETS = (0, -2 * np.pi / 3, 2 * np.pi / 3)
COLORS = ('#285a70', '#c27831', '#2e746c')


def main():
    matplotlib.rcParams['svg.hashsalt'] = 'synthetic-fifth-harmonic-example'
    rng = np.random.default_rng(5)
    time_s = np.arange(SAMPLE_COUNT) / SAMPLE_RATE_HZ
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.4), sharey=True)
    for ax, (role, fundamental_amp, fifth_amp) in zip(
            axes, (('Test motor', 1.5, 0.65), ('Load motor', 1.8, 0.45))):
        for phase, (offset, color) in enumerate(zip(PHASE_OFFSETS, COLORS)):
            scale = (1, .96, 1.04)[phase]
            current = (fundamental_amp * scale * np.sin(2 * np.pi * FUNDAMENTAL_HZ * time_s + offset)
                       + fifth_amp * scale * np.sin(2 * np.pi * FIFTH_HZ * time_s + 5 * offset)
                       + .035 * rng.standard_normal(SAMPLE_COUNT))
            frequency, density, _ = psd(current, SAMPLE_RATE_HZ, RPM, POLE_PAIRS)
            mask = (frequency > 0) & (frequency <= 300)
            ax.semilogy(frequency[mask], density[mask], color=color, lw=1.15,
                        label=f'Phase {chr(65 + phase)}')
        ax.axvline(FUNDAMENTAL_HZ, color='#285a70', ls='--', lw=1.2)
        ax.axvline(FIFTH_HZ, color='#ba4b32', ls='--', lw=1.4)
        ax.text(FUNDAMENTAL_HZ + 4, 1.25, 'Fundamental\n30 Hz', color='#285a70', va='top')
        ax.text(FIFTH_HZ + 4, 1.25, '5th harmonic\n150 Hz', color='#a43e29', va='top')
        ax.set(title=role, xlabel='Frequency (Hz)', xlim=(0, 300), ylim=(1e-8, 2),
               ylabel='Current PSD (A²/Hz)')
        ax.grid(True, which='both', alpha=.2)
        ax.legend(loc='upper right', fontsize=8)
    fig.suptitle('SYNTHETIC EXAMPLE · three-phase current power spectral density',
                 fontsize=15, fontweight='bold')
    fig.text(.5, .015, '600 rpm × 3 pole pairs = 30 Hz electrical fundamental; 5 × 30 Hz = 150 Hz.  '
             '8 kHz sampling, 1 s record, periodic Hann FFT. Not motor measurements.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .06, 1, .91))
    fig.savefig(ROOT / 'demo_fifth_harmonic.svg', metadata={'Date': None})
    plt.close(fig)


if __name__ == '__main__':
    main()
