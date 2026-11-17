"""Reduce only the milling-table visual; retain its original collision mesh.

Requires trimesh and fast-simplification. Lengths follow the source STL (m).
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh


def max_surface_distance(mesh, samples):
    # Bound memory when comparing against the detailed CAD mesh.
    maximum = 0.0
    for offset in range(0, len(samples), 8):
        _, distances, _ = trimesh.proximity.closest_point_naive(mesh, samples[offset:offset + 8])
        maximum = max(maximum, float(max(distances)))
    return maximum


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--faces', type=int, default=3000)
    parser.add_argument('--max-error-m', type=float, default=0.003)
    args = parser.parse_args()
    original = trimesh.load_mesh(args.source, process=True)
    samples = original.vertices[np.linspace(0, len(original.vertices) - 1, 256).astype(int)]
    target = args.faces
    while target < len(original.faces):
        reduced = original.simplify_quadric_decimation(face_count=target, aggression=5)
        forward = max_surface_distance(reduced, samples)
        reverse_samples = reduced.vertices[np.linspace(0, len(reduced.vertices) - 1, 64).astype(int)]
        reverse = max_surface_distance(original, reverse_samples)
        bounds_error = float(abs(original.bounds - reduced.bounds).max())
        surface_error = max(forward, reverse)
        if bounds_error <= 0.001 and surface_error <= args.max_error_m:
            break
        target *= 2
    else:
        raise RuntimeError('No reduced MT mesh met the visual geometry tolerances')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    reduced.export(args.output)
    report = {
        'source': 'meshes/visual/fixtures/MT_base.stl',
        'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
        'original_triangles': len(original.faces),
        'triangles': len(reduced.faces),
        'bounds_error_m': bounds_error,
        'sampled_surface_error_m': surface_error,
        'bounds_m': reduced.bounds.tolist(),
        'collision_mesh': 'meshes/collision/fixtures/MT_base.stl (unchanged)',
    }
    args.output.with_suffix('.validation.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
