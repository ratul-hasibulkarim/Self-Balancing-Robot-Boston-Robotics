"""
Planar 5-bar leg kinematics (identical maths to firmware/lib/core/five_bar.h).

Leg plane coordinates: x forward, z up, origin = hip midpoint.
  A = front hip motor (+l5/2, 0), E = rear hip motor (-l5/2, 0)
  phi1 = front thigh angle, phi4 = rear thigh angle, both measured from +x,
  counter-clockwise positive when looking from the robot's LEFT side
  (i.e. rotation about -y ... we simply work in the (x, z) plane).
  B = front knee, D = rear knee, C = foot / wheel axle.

Virtual leg: L = |OC|, theta = atan2(Cx, -Cz)  (theta > 0 -> foot ahead of hip)
"""
import math


def forward(phi1, phi4, l1, l2, l5):
    ax, az = l5 / 2, 0.0
    ex, ez = -l5 / 2, 0.0
    bx, bz = ax + l1 * math.cos(phi1), az + l1 * math.sin(phi1)
    dx, dz = ex + l1 * math.cos(phi4), ez + l1 * math.sin(phi4)
    # circle-circle intersection, both radius l2, centres B and D
    mx, mz = (bx + dx) / 2, (bz + dz) / 2
    hx, hz = dx - bx, dz - bz
    d = math.hypot(hx, hz)
    if d < 1e-9 or d > 2 * l2:
        raise ValueError("5-bar out of reach")
    h = math.sqrt(max(l2 * l2 - d * d / 4, 0.0))
    # perpendicular that points downward (foot below knees)
    px, pz = -hz / d, hx / d
    if pz > 0:
        px, pz = -px, -pz
    cx, cz = mx + h * px, mz + h * pz
    L = math.hypot(cx, cz)
    theta = math.atan2(cx, -cz)
    return dict(B=(bx, bz), D=(dx, dz), C=(cx, cz), L=L, theta=theta)


def inverse(L, theta, l1, l2, l5):
    cx, cz = L * math.sin(theta), -L * math.cos(theta)
    ax, ex = l5 / 2, -l5 / 2
    # front thigh, knee forward
    dfx, dfz = cx - ax, cz
    d1 = math.hypot(dfx, dfz)
    a1 = math.acos(max(-1, min(1, (l1 * l1 + d1 * d1 - l2 * l2) / (2 * l1 * d1))))
    phi1 = math.atan2(dfz, dfx) + a1
    # rear thigh, knee backward
    drx, drz = cx - ex, cz
    d4 = math.hypot(drx, drz)
    a4 = math.acos(max(-1, min(1, (l1 * l1 + d4 * d4 - l2 * l2) / (2 * l1 * d4))))
    phi4 = math.atan2(drz, drx) - a4
    return phi1, phi4


def jacobian(phi1, phi4, l1, l2, l5, eps=1e-6):
    """d(L, theta)/d(phi1, phi4) by central differences."""
    J = [[0.0, 0.0], [0.0, 0.0]]
    for j, (dp1, dp4) in enumerate(((eps, 0.0), (0.0, eps))):
        fp = forward(phi1 + dp1, phi4 + dp4, l1, l2, l5)
        fm = forward(phi1 - dp1, phi4 - dp4, l1, l2, l5)
        J[0][j] = (fp["L"] - fm["L"]) / (2 * eps)
        J[1][j] = (fp["theta"] - fm["theta"]) / (2 * eps)
    return J


if __name__ == "__main__":
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))
    from params import FINAL as P
    for L in (P["L_min"], P["L_nom"], P["L_max"]):
        p1, p4 = inverse(L, 0.0, P["thigh"], P["shin"], P["hip_spacing"])
        fk = forward(p1, p4, P["thigh"], P["shin"], P["hip_spacing"])
        J = jacobian(p1, p4, P["thigh"], P["shin"], P["hip_spacing"])
        print(f"L={L:.3f} phi1={math.degrees(p1):7.2f} phi4={math.degrees(p4):7.2f}"
              f"  FK L={fk['L']:.4f} th={fk['theta']:.2e}  dL/dphi={J[0]}")
