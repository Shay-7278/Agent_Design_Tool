"""Static Phase-A check for the Gazebo brick model."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def main() -> None:
    sdf_path = Path(__file__).parents[1] / "models" / "brick" / "model.sdf"
    root = ET.parse(sdf_path).getroot()
    box_sizes = [tuple(float(x) for x in e.text.split())
                 for e in root.findall(".//geometry/box/size")]
    assert box_sizes == [(0.190, 0.090, 0.057), (0.190, 0.090, 0.057)]
    mass = float(root.findtext(".//inertial/mass"))
    expected_mass = 700.0 * 0.190 * 0.090 * 0.057
    assert math.isclose(mass, expected_mass, rel_tol=1e-9)
    inertia = [float(root.findtext(f".//inertia/{name}"))
               for name in ("ixx", "iyy", "izz")]
    expected = [
        expected_mass / 12 * (0.090**2 + 0.057**2),
        expected_mass / 12 * (0.190**2 + 0.057**2),
        expected_mass / 12 * (0.190**2 + 0.090**2),
    ]
    assert all(math.isclose(a, b, rel_tol=1e-9) for a, b in zip(inertia, expected))
    friction = [float(root.findtext(f".//friction/ode/{name}")) for name in ("mu", "mu2")]
    assert friction == [0.84, 0.84]
    print(f"brick model OK: size={box_sizes[0]}, mass={mass:.5f} kg, mu={friction[0]}")


if __name__ == "__main__":
    main()
