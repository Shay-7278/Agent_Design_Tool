"""Generate simulation meshes without overwriting CAD exports (trimesh/fast-simplification)."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--meshes', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = []
    for kind in ('visual', 'collision'):
        paths = [f for f in (args.meshes / kind / 'end_effectors').glob('*.stl') if 'TC_master' not in f.name]
        for source in paths:
            original = trimesh.load_mesh(source, process=True)
            count = (15000 if 'body' in source.name else 5000) if kind == 'visual' else 2000
            while True:
                reduced = original.simplify_quadric_decimation(face_count=count, aggression=5)
                bounds_error = float(abs(original.bounds-reduced.bounds).max())
                points = original.vertices[np.linspace(0,len(original.vertices)-1,100).astype(int)]
                _, errors, _ = trimesh.proximity.closest_point_naive(reduced, points)
                pad_error = 0.0
                if 'finger' in source.name:
                    x = -54.91247559 if 'left' in source.name else 54.91249084
                    pads = np.array([[x,y,z] for y in [-12,0,12] for z in [-216.35,-201.35,-186.35]])
                    _, distances, _ = trimesh.proximity.closest_point_naive(reduced,pads)
                    pad_error = float(max(distances))
                if bounds_error < .05 and max(errors) < .25 and pad_error < .01:
                    break
                count *= 2
                if count >= len(original.faces):
                    raise RuntimeError(f'Unable to reduce {source} within geometric tolerances')
            name = kind + '_' + ('body' if 'body' in source.name else 'leftfinger' if 'left' in source.name else 'rightfinger') + '.stl'
            reduced.export(args.output / name)
            report.append(dict(source=str(source.relative_to(args.meshes)), output=name,
                               original_triangles=len(original.faces), triangles=len(reduced.faces),
                               bounds_error_mm=bounds_error, sampled_surface_error_mm=float(max(errors)),
                               pad_error_mm=pad_error))
    (args.output/'mesh_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
