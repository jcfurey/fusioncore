#!/usr/bin/env python3
"""
Convert Oxford Robotcar gps/ins.csv to TUM trajectory format (ground truth).

The NovAtel SPAN-CPT RTK GPS/INS solution is used as the reference trajectory.
Positions are projected to local ENU from the first fix as origin.
Orientation is set to identity quaternion (evo aligns trajectories anyway).

ins.csv columns:
  timestamp, latitude, longitude, altitude,
  northing, easting, down,
  velocity_north, velocity_east, velocity_down,
  roll, pitch, yaw

Usage:
  python3 tools/robotcar_ins_to_tum.py \
    --ins /path/to/robotcar/<seq>/gps/ins.csv \
    --out benchmarks/robotcar/<seq>/ground_truth.tum
"""

import argparse
import csv
import math


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    a  = 6378137.0
    e2 = 6.6943799901377997e-3
    N  = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    x  = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y  = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z  = (N * (1 - e2) + alt_m) * math.sin(lat)
    return x, y, z


def ecef_to_enu(px, py, pz, ref_lat_deg, ref_lon_deg, ref_x, ref_y, ref_z):
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    dx, dy, dz = px - ref_x, py - ref_y, pz - ref_z
    sl, cl = math.sin(lat), math.cos(lat)
    sn, cn = math.sin(lon), math.cos(lon)
    e =  -sn * dx + cn * dy
    n =  -sl * cn * dx - sl * sn * dy + cl * dz
    u =   cl * cn * dx + cl * sn * dy + sl * dz
    return e, n, u


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ins', required=True, help='Path to gps/ins.csv')
    parser.add_argument('--out', required=True, help='Output TUM file path')
    parser.add_argument('--subsample', type=int, default=1,
                        help='Keep every Nth row (default 1 = all rows)')
    args = parser.parse_args()

    rows = []
    with open(args.ins) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('timestamp'):
                continue
            parts = line.split(',')
            if len(parts) < 13:
                continue
            try:
                rows.append((
                    int(parts[0]),
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                ))
            except (ValueError, IndexError):
                continue

    if not rows:
        print('No valid INS rows found')
        return

    rows.sort(key=lambda r: r[0])

    ref_ts, ref_lat, ref_lon, ref_alt = rows[0]
    ref_x, ref_y, ref_z = lla_to_ecef(ref_lat, ref_lon, ref_alt)

    count = 0
    with open(args.out, 'w') as f:
        for i, (ts, lat, lon, alt) in enumerate(rows):
            if i % args.subsample != 0:
                continue
            if any(math.isnan(v) for v in [lat, lon, alt]):
                continue
            px, py, pz = lla_to_ecef(lat, lon, alt)
            e, n, u = ecef_to_enu(px, py, pz, ref_lat, ref_lon, ref_x, ref_y, ref_z)
            t = ts / 1e6
            f.write(f'{t:.6f} {e:.6f} {n:.6f} {u:.6f} 0.000000 0.000000 0.000000 1.000000\n')
            count += 1

    print(f'Written {count} poses to {args.out}')
    print(f'Origin: {ref_lat:.6f}N  {ref_lon:.6f}E  alt={ref_alt:.1f}m')


if __name__ == '__main__':
    main()
