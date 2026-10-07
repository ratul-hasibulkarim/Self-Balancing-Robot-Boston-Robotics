// Planar 5-bar leg kinematics.  Same maths as design/kinematics.py.
//
// Leg-plane coordinates: x forward, z up, origin = hip midpoint.
// phi1 = front thigh angle, phi4 = rear thigh angle, measured from +x,
// counter-clockwise in the (x, z) plane (i.e. a rotation about -y).
// Virtual leg: L = |OC|, theta_b = atan2(Cx, -Cz)   (> 0 : foot ahead of hip)
#pragma once
#include <math.h>

namespace bolt {

struct FiveBarGeom {
  float l1;   // thigh
  float l2;   // shin
  float l5;   // hip spacing
};

struct LegKin {
  float L, theta;           // virtual leg length / angle (body frame)
  float Cx, Cz;             // foot position
  float alpha_rear_shin;    // rear shin angle (CCW from +x), carries the wheel stator
  float J[2][2];            // d(L, theta) / d(phi1, phi4)
  bool ok;
};

inline bool five_bar_fk(const FiveBarGeom& g, float phi1, float phi4,
                        float& cx, float& cz, float& alpha_r) {
  const float ax = 0.5f * g.l5, ex = -0.5f * g.l5;
  const float bx = ax + g.l1 * cosf(phi1), bz = g.l1 * sinf(phi1);
  const float dx = ex + g.l1 * cosf(phi4), dz = g.l1 * sinf(phi4);
  const float mx = 0.5f * (bx + dx), mz = 0.5f * (bz + dz);
  const float hx = dx - bx, hz = dz - bz;
  const float d = sqrtf(hx * hx + hz * hz);
  if (d < 1e-6f || d > 2.0f * g.l2) return false;
  const float h = sqrtf(fmaxf(g.l2 * g.l2 - 0.25f * d * d, 0.0f));
  float px = -hz / d, pz = hx / d;
  if (pz > 0) { px = -px; pz = -pz; }        // foot below the knees
  cx = mx + h * px;
  cz = mz + h * pz;
  alpha_r = atan2f(cz - dz, cx - dx);
  return true;
}

inline LegKin five_bar_solve(const FiveBarGeom& g, float phi1, float phi4) {
  LegKin k{};
  float cx, cz, ar;
  k.ok = five_bar_fk(g, phi1, phi4, cx, cz, ar);
  if (!k.ok) return k;
  k.Cx = cx; k.Cz = cz; k.alpha_rear_shin = ar;
  k.L = sqrtf(cx * cx + cz * cz);
  k.theta = atan2f(cx, -cz);
  const float eps = 1e-4f;
  for (int j = 0; j < 2; ++j) {
    const float d1 = (j == 0) ? eps : 0.f, d4 = (j == 1) ? eps : 0.f;
    float cxp, czp, cxm, czm, a;
    if (!five_bar_fk(g, phi1 + d1, phi4 + d4, cxp, czp, a) ||
        !five_bar_fk(g, phi1 - d1, phi4 - d4, cxm, czm, a)) { k.ok = false; return k; }
    const float Lp = sqrtf(cxp * cxp + czp * czp), Lm = sqrtf(cxm * cxm + czm * czm);
    k.J[0][j] = (Lp - Lm) / (2 * eps);
    k.J[1][j] = (atan2f(cxp, -czp) - atan2f(cxm, -czm)) / (2 * eps);
  }
  return k;
}

// Inverse kinematics: (L, theta_b) -> (phi1, phi4), knees outward.
inline bool five_bar_ik(const FiveBarGeom& g, float L, float theta, float& phi1, float& phi4) {
  const float cx = L * sinf(theta), cz = -L * cosf(theta);
  const float ax = 0.5f * g.l5, ex = -0.5f * g.l5;
  const float dfx = cx - ax, dfz = cz, d1 = sqrtf(dfx * dfx + dfz * dfz);
  const float drx = cx - ex, drz = cz, d4 = sqrtf(drx * drx + drz * drz);
  if (d1 > g.l1 + g.l2 || d4 > g.l1 + g.l2) return false;
  float c1 = (g.l1 * g.l1 + d1 * d1 - g.l2 * g.l2) / (2 * g.l1 * d1);
  float c4 = (g.l1 * g.l1 + d4 * d4 - g.l2 * g.l2) / (2 * g.l1 * d4);
  c1 = fminf(1.f, fmaxf(-1.f, c1));
  c4 = fminf(1.f, fmaxf(-1.f, c4));
  phi1 = atan2f(dfz, dfx) + acosf(c1);
  phi4 = atan2f(drz, drx) - acosf(c4);
  return true;
}

}  // namespace bolt
