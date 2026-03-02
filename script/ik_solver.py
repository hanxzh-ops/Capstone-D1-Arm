#!/usr/bin/env python3
import argparse
import sys
import numpy as np
from ikpy.chain import Chain

# D1-550 limits for J0..J5 (deg)
LIMITS = np.array([
    [-135.0, 135.0],  # J0
    [ -90.0,  90.0],  # J1
    [ -90.0,  90.0],  # J2
    [-135.0, 135.0],  # J3
    [ -90.0,  90.0],  # J4
    [-135.0, 135.0],  # J5
], dtype=float)

def clip_deg(q6):
    q = np.array(q6, dtype=float)
    for i in range(6):
        q[i] = np.clip(q[i], LIMITS[i, 0], LIMITS[i, 1])
    return q

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True, help="Path to D1-550 URDF")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--z", type=float, required=True)
    args = ap.parse_args()

    # Position-only IK (orientation unconstrained for simplicity)
    chain = Chain.from_urdf_file(args.urdf)
    q_full = chain.inverse_kinematics(target_position=np.array([args.x, args.y, args.z], dtype=float))

    # Typical mapping: [base, j0, j1, j2, j3, j4, j5, ...]
    if len(q_full) < 7:
        print("IK output too short for 6 joints", file=sys.stderr)
        sys.exit(2)

    q6_deg = np.rad2deg(np.array(q_full[1:7], dtype=float))
    q6_deg = clip_deg(q6_deg)

    # Print plain numbers for C++ parser
    print(" ".join(f"{v:.6f}" for v in q6_deg))

if __name__ == "__main__":
    main()
