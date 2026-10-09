#!/usr/bin/env python3
"""Measure APSK cluster centers without rescaling away the amplitude error.

Input is an SDRangel constellation capture made by apsk_probe.cpp: int16
row,column pairs. Coordinates come after symbol timing, RRC, carrier recovery
and PL-based gain control, exactly where SDRangel draws its constellation.
The one-pixel quantization is included in the uncertainty. The receiver's
cstln_amp is 75 in SDRangel 7.22.9; specify --scope-amplitude for another build.

python3 host/apsk_centroids.py points.s16 --mod 16apsk --fec 2/3 --output result
"""
import argparse
import csv
import json
import os
from pathlib import Path

import numpy as np
import dvbs2


def ideal_points(mod, fec):
    return dvbs2.apsk16_points(fec) if mod == "16apsk" else dvbs2.apsk32_points(fec)


def load_pixels(path, scope_amplitude=75.0, first=0, count=None):
    pairs = np.fromfile(path, dtype="<i2").reshape(-1, 2)
    pairs = pairs[first:] if count is None else pairs[first:first + count]
    # selectRow()/setDataColor() truncate positive pixel coordinates. Recover
    # bin centers, rather than introducing a spurious half-pixel DC offset.
    return ((pairs[:, 0] - 127.5) + 1j * (127.5 - pairs[:, 1])) / scope_amplitude


def analyze(z, ideal):
    z = np.asarray(z, complex)
    z = z[np.isfinite(z) & (np.abs(z) < 2.5)]
    if len(z) < 1000:
        raise ValueError("At least 1000 recovered symbols are required")
    # A scale is used only to initialize cluster assignments. Measurements
    # remain in the original receiver coordinates throughout.
    initial_scale = np.sqrt(np.mean(abs(z) ** 2) / np.mean(abs(ideal) ** 2))
    centers = ideal * initial_scale
    for _ in range(12):
        labels = np.argmin(abs(z[:, None] - centers[None, :]) ** 2, axis=1)
        new = np.array([z[labels == k].mean() if np.any(labels == k) else centers[k]
                        for k in range(len(ideal))])
        if np.max(abs(new - centers)) < 1e-7:
            centers = new
            break
        centers = new
    counts = np.bincount(labels, minlength=len(ideal))
    if counts.min() < 40:
        raise ValueError(f"Incomplete constellation: smallest cluster has {counts.min()} symbols")
    dc = centers.mean()
    centered = centers - dc
    rotation = np.angle(np.vdot(ideal, centered))
    aligned = centered * np.exp(-1j * rotation)
    # Fit a common gain after, rather than before, measuring the centers.
    gain = np.vdot(ideal, aligned).real / np.vdot(ideal, ideal).real
    residual = aligned - gain * ideal
    rows = []
    for k, expected in enumerate(ideal):
        members = (z[labels == k] - dc) * np.exp(-1j * rotation)
        direction = expected / abs(expected)
        error = members - aligned[k]
        radial = (error * np.conj(direction)).real
        tangential = (error * np.conj(direction)).imag
        radial_mean = (aligned[k] * np.conj(direction)).real
        rows.append(dict(symbol=k, count=int(counts[k]),
                         ideal_i=float(expected.real), ideal_q=float(expected.imag),
                         measured_i=float(centers[k].real), measured_q=float(centers[k].imag),
                         aligned_i=float(aligned[k].real), aligned_q=float(aligned[k].imag),
                         expected_radius=float(abs(expected)), measured_radius=float(abs(aligned[k])),
                         radial_error_percent=float(100 * (radial_mean / abs(expected) - 1)),
                         phase_error_deg=float(np.degrees(np.angle(aligned[k] / expected))),
                         radial_std=float(np.std(radial, ddof=1)), tangential_std=float(np.std(tangential, ddof=1)),
                         radial_mean_95ci=float(1.96 * np.std(radial, ddof=1) / np.sqrt(len(radial)))))
    rings = []
    for r in np.unique(np.round(abs(ideal), 10)):
        mask = np.isclose(abs(ideal), r)
        radii = abs(aligned[mask])
        rings.append(dict(expected_radius=float(r), measured_radius=float(radii.mean()),
                          radial_error_percent=float(100 * (radii.mean() / r - 1)),
                          symbols=np.flatnonzero(mask).tolist(), centroid_radius_spread=float(np.std(radii))))
    for ring in rings:
        ring["expected_ratio_to_inner"] = ring["expected_radius"] / rings[0]["expected_radius"]
        ring["measured_ratio_to_inner"] = ring["measured_radius"] / rings[0]["measured_radius"]
    target = ideal[labels]
    corrected = (z - dc) * np.exp(-1j * rotation)
    p = np.mean(abs(target) ** 2)
    raw_evm = np.mean(abs(corrected - target) ** 2)
    fitted_evm = np.mean(abs(corrected / gain - target) ** 2)
    result = dict(samples=len(z), scope_amplitude=75, receiver_dc_i=float(dc.real), receiver_dc_q=float(dc.imag),
                  receiver_rotation_deg=float(np.degrees(rotation)), payload_gain=float(gain),
                  centroid_error_after_common_gain_percent=float(100 * np.sqrt(np.mean(abs(residual) ** 2) / np.mean(abs(gain * ideal) ** 2))),
                  mer_in_receiver_coordinates_db=float(10 * np.log10(p / raw_evm)),
                  mer_after_common_gain_db=float(10 * np.log10(p / fitted_evm)),
                  predicted_gain_if_pl_uses_outer_ring=float(1 / max(abs(ideal))),
                  predicted_outer_radius_if_pl_uses_outer_ring=1.0,
                  predicted_inward_shift_percent=float(100 * (1 - 1 / max(abs(ideal)))),
                  rings=rings, centroids=rows,
                  limitations=["Samples are the quantized constellation pixels (1/75 unit per pixel), not raw ADC samples.",
                               "The GUI exports one payload symbol per 90-symbol slot; repeated measurements reduce sampling uncertainty.",
                               "Confidence intervals describe the selected samples and do not bound systematic receiver error.",
                               "A common gain error and a change of ring ratios are reported separately."])
    return result, corrected, labels, aligned


def reference_crosses(path, scope_amplitude=75):
    """Recover the centers of the receiver's 18-pixel white reference crosses."""
    if not Path(path).exists():
        return []
    p = np.fromfile(path, dtype="<i2").reshape(-1, 2)
    p = p[:len(p) // 18 * 18].reshape(-1, 18, 2).mean(axis=1)
    points = (p[:, 0] - 128) + 1j * (128 - p[:, 1])
    values, counts = np.unique(np.round(points, 6), return_counts=True)
    return [dict(i=float(x.real / scope_amplitude), q=float(x.imag / scope_amplitude), count=int(n))
            for x, n in zip(values, counts)]


def export(path, result, z, ideal, labels, centers):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    (path / "centroids.json").write_text(json.dumps(result, indent=2) + "\n")
    with (path / "centroids.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(result["centroids"][0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(result["centroids"])
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/esp32_datv_matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), dpi=160)
    select = np.linspace(0, len(z) - 1, min(len(z), 18000), dtype=int)
    for ax, scale, title in [(axes[0], 1, "Original receiver scale"),
                             (axes[1], result["payload_gain"], "After fitting one common gain")]:
        ax.scatter(z[select].real / scale, z[select].imag / scale, s=1, alpha=.12, color="#7457b7", rasterized=True)
        ax.scatter(ideal.real, ideal.imag, marker="+", s=90, linewidths=1.5, color="#222222", label="Ideal (E = 1)")
        ax.scatter(centers.real / scale, centers.imag / scale, s=28, color="#db491e", label="Measured centers")
        for x, y in zip(ideal, centers / scale):
            ax.plot([x.real, y.real], [x.imag, y.imag], color="#db491e", linewidth=1)
        for r in np.unique(np.round(abs(ideal), 10)):
            ax.add_patch(plt.Circle((0, 0), r, fill=False, color="#bbbbbb", linewidth=.7))
        ax.set(xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), xlabel="I", ylabel="Q", title=title, aspect="equal")
        ax.grid(alpha=.2)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(f"{result.get('modulation', 'APSK')} FEC {result.get('fec', '?')} · {result['samples']:,} recovered symbols")
    fig.tight_layout()
    fig.savefig(path / "constellation.png")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--mod", choices=("16apsk", "32apsk"), default="16apsk")
    ap.add_argument("--fec", default="2/3")
    ap.add_argument("--scope-amplitude", type=float, default=75)
    ap.add_argument("--first", type=int, default=0, help="first captured symbol")
    ap.add_argument("--count", type=int)
    a = ap.parse_args()
    ideal = ideal_points(a.mod, a.fec)
    result, z, labels, centers = analyze(load_pixels(a.input, a.scope_amplitude, a.first, a.count), ideal)
    result.update(modulation=a.mod.upper(), fec=a.fec, input_file=str(a.input), scope_amplitude=a.scope_amplitude,
                  first=a.first, requested_count=a.count,
                  receiver_reference_crosses=reference_crosses(str(a.input) + ".refs", a.scope_amplitude))
    export(a.output, result, z, ideal, labels, centers)
    print(json.dumps({k:result[k] for k in ("samples", "payload_gain", "rings", "predicted_inward_shift_percent", "centroid_error_after_common_gain_percent")}, indent=2))


if __name__ == "__main__":
    main()
