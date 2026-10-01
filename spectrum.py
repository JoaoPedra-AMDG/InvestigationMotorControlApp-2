"""Current spectra from one finite, uniformly sampled ODrive capture.

The integrated PSD has units A^2. It is current variance, not electrical
power in watts. Each board is analysed on its own clock.
"""
import math

import numpy as np


PHASES = ('ia_a', 'ib_a', 'ic_a')


def analyze_spectrum(rows, metadata):
    if len(rows) < 32:
        raise ValueError('At least 32 high-rate samples are required for a spectrum.')
    acquisition = metadata.get('acquisition') or {}
    if acquisition.get('source') != 'ODRIVE_ONBOARD':
        raise ValueError('This view requires an original onboard phase-current capture.')
    if acquisition.get('partial') or metadata.get('capture_interrupted'):
        raise ValueError('An interrupted or partial capture cannot be compared.')
    t = np.asarray([r.get('time_s') for r in rows], dtype=float)
    if not np.all(np.isfinite(t)) or not np.all(np.diff(t) > 0):
        raise ValueError('Capture timestamps are missing or unordered.')
    dt = float(np.median(np.diff(t)))
    if not np.allclose(np.diff(t), dt, rtol=0.01, atol=1e-8):
        raise ValueError('Capture sampling is not uniform enough for an FFT.')
    fs = 1 / dt
    expected_rate = acquisition.get('requested_hz')
    if expected_rate and abs(fs - float(expected_rate)) / float(expected_rate) > 0.01:
        raise ValueError('Capture timestamps disagree with the reported sample rate.')
    pole_pairs = metadata.get('pole_pairs')
    rpm = (metadata.get('plan') or {}).get('rpm')
    if not isinstance(pole_pairs, (int, float)) or not math.isfinite(pole_pairs) or pole_pairs <= 0:
        raise ValueError('The motor pole-pair count is required.')
    if not isinstance(rpm, (int, float)) or not math.isfinite(rpm) or rpm <= 0:
        raise ValueError('A positive requested test speed is required.')
    fundamental = float(rpm) * float(pole_pairs) / 60
    freq = np.fft.rfftfreq(len(t), dt)
    df = float(freq[1])
    if fundamental < 2 * df or fundamental >= fs / 2 - 2 * df:
        raise ValueError('The capture is too short or too slow to resolve the requested electrical frequency.')
    # A periodic Hann window places an on-bin sine in its centre and adjacent
    # bins. Include two bins each side to tolerate modest off-bin leakage.
    band = (freq > 0) & (np.abs(freq - fundamental) <= 2 * df)
    positive = freq > 0
    window = np.hanning(len(t) + 1)[:-1]
    phase_results = {}
    spectra = []
    for key in PHASES:
        x = np.asarray([r.get(key) for r in rows], dtype=float)
        if not np.all(np.isfinite(x)):
            raise ValueError(f'{key} has missing or invalid samples.')
        transform = np.fft.rfft((x - x.mean()) * window)
        density = np.abs(transform) ** 2 / (fs * np.sum(window ** 2))
        density[1:-1] *= 2
        spectra.append(density)
        target = float(np.sum(density[band]) * df)
        total = float(np.sum(density[positive]) * df)
        outside = max(0., total - target)
        phase_results[key] = dict(target_a2=target, outside_a2=outside,
            total_a2=total, target_fraction=target / total if total > 0 else None,
            target_to_outside_db=10 * math.log10(target / outside) if target > 0 and outside > 0 else None)
    mean_density = np.mean(spectra, axis=0)
    target = sum(v['target_a2'] for v in phase_results.values()) / 3
    outside = sum(v['outside_a2'] for v in phase_results.values()) / 3
    return dict(method='One-sided mean-detrended periodic-Hann FFT periodogram',
        meaning='Integrated current PSD is A^2, not electrical power in W.',
        role=metadata.get('role'), sample_rate_hz=fs, samples=len(t), duration_s=len(t) * dt,
        resolution_hz=df, nyquist_hz=fs / 2, target_rpm=rpm, pole_pairs=pole_pairs,
        desired_electrical_hz=fundamental,
        desired_band_hz=[max(0., fundamental - 2 * df), fundamental + 2 * df],
        band_rule='Within two FFT bins of the requested electrical fundamental; DC excluded.',
        target_fraction=target / (target + outside) if target + outside > 0 else None,
        target_to_outside_db=10 * math.log10(target / outside) if target > 0 and outside > 0 else None,
        phases=phase_results, frequency_hz=freq.tolist(),
        mean_psd_a2_per_hz=mean_density.tolist(),
        phase_psd_a2_per_hz={key:array.tolist() for key,array in zip(PHASES,spectra)},
        limitations=['One finite window is a descriptive periodogram, not an averaged PSD.',
                     'Boards are not synchronized.', 'Content above Nyquist cannot be resolved.'])
