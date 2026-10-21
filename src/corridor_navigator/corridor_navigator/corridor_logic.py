"""Pure-Python maths for corridor_navigator (no ROS imports, so it can be unit-tested).

Frames: PX4 local NED (x north, y east, z down), heading in rad, 0 = north, clockwise positive.
Body commands use ROS FLU (x forward, y left, z up, yaw rate counter-clockwise positive), the
convention of flight_controller's cmd_vel_body.
"""
import math
import re
from collections import namedtuple
from dataclasses import dataclass

EARTH_RADIUS = 6371000.0  # m, the sphere PX4 uses for its local map projection

Pose = namedtuple("Pose", "north east down heading speed")


@dataclass
class Gains:
    cross_kp: float = 0.8  # 1/s, lateral speed per metre of cross-track error
    max_lateral: float = 0.5  # m/s
    alt_kp: float = 1.0  # 1/s
    max_vertical: float = 0.5  # m/s
    yaw_kp: float = 1.0  # 1/s
    max_yaw_rate: float = 0.5  # rad/s
    hold_kp: float = 0.6  # 1/s, position hold while turning on the spot
    max_hold: float = 0.3  # m/s


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------- delivery-zone file

def parse_zone_file(path):
    """Read corner points from a text file: one 'lat,lon' (degrees) per line, '#' starts a comment."""
    points = []
    with open(path, encoding="utf-8") as f:
        for number, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [p for p in re.split(r"[,;\s]+", line) if p]
            try:
                if len(parts) != 2:
                    raise ValueError
                lat, lon = float(parts[0]), float(parts[1])
            except ValueError:
                raise ValueError(f"{path}:{number}: expected 'lat,lon', got '{line}'") from None
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                raise ValueError(f"{path}:{number}: lat/lon out of range: '{line}'")
            points.append((lat, lon))
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()  # closed ring: drop the repeated first point
    if len(points) < 3:
        raise ValueError(f"{path}: a zone needs at least 3 corner points, found {len(points)}")
    return points


def latlon_to_ne(lat, lon, ref_lat, ref_lon):
    """North/east offset (m) of a point from the reference; accurate to well under 1 cm per 100 m."""
    north = math.radians(lat - ref_lat) * EARTH_RADIUS
    east = math.radians(lon - ref_lon) * EARTH_RADIUS * math.cos(math.radians(ref_lat))
    return north, east


def point_in_polygon(north, east, polygon):
    """Ray casting; polygon is a list of (north, east)."""
    inside = False
    j = len(polygon) - 1
    for i, (ni, ei) in enumerate(polygon):
        nj, ej = polygon[j]
        if (ei > east) != (ej > east) and north < (nj - ni) * (east - ei) / (ej - ei) + ni:
            inside = not inside
        j = i
    return inside


def centroid(polygon):
    """Mean of the corner points (good enough to aim at a small zone)."""
    return (sum(p[0] for p in polygon) / len(polygon), sum(p[1] for p in polygon) / len(polygon))


# ---------------------------------------------------------------- straight-line geometry

class Line:
    """Straight line through (north, east) pointing along `heading`."""

    def __init__(self, north, east, heading):
        self.north = north
        self.east = east
        self.heading = heading
        self.along = (math.cos(heading), math.sin(heading))  # unit vector (north, east)
        self.right = (-math.sin(heading), math.cos(heading))  # unit vector to the right of travel

    def project(self, north, east):
        """(distance along the line, cross-track offset; positive = right of the line)."""
        dn, de = north - self.north, east - self.east
        return (dn * self.along[0] + de * self.along[1], dn * self.right[0] + de * self.right[1])

    def world_velocity(self, along, right):
        return (along * self.along[0] + right * self.right[0], along * self.along[1] + right * self.right[1])


def world_to_body(v_north, v_east, heading):
    """NED horizontal velocity -> (forward, left); the inverse of flight_controller's cmd_vel_cb."""
    forward = v_north * math.cos(heading) + v_east * math.sin(heading)
    left = v_north * math.sin(heading) - v_east * math.cos(heading)
    return forward, left


def speed_profile(elapsed, remaining, cruise, accel, creep=0.1):
    """Trapezoid speed: ramp up from `creep`, cruise, brake so the speed reaches ~0 at the target."""
    ramp_up = creep + accel * elapsed
    braking = max(creep, math.sqrt(2.0 * accel * max(remaining, 0.0)))
    return max(0.0, min(cruise, ramp_up, braking))


def stop_target(mode, distance_target, enter_s, margin, max_overrun):
    """Along-track distance at which to stop.

    mode: 'distance' (fixed distance), 'geofence' (zone entry + margin), 'either' (whichever first).
    enter_s: along-track distance where the drone first entered the zone, or None.
    """
    zone_target = None if enter_s is None else enter_s + margin
    if mode == "distance":
        return distance_target
    if mode == "geofence":
        return zone_target if zone_target is not None else distance_target + max_overrun
    return distance_target if zone_target is None else min(distance_target, zone_target)


# ---------------------------------------------------------------- body-frame commands

def run_command(line, pose, along_speed, z_ref, gains):
    """(forward, left, up, yaw_rate) to fly along `line` at `along_speed`, holding line, height, heading."""
    _, right = line.project(pose.north, pose.east)
    v_north, v_east = line.world_velocity(along_speed, -clamp(gains.cross_kp * right, gains.max_lateral))
    forward, left = world_to_body(v_north, v_east, pose.heading)
    up = clamp(gains.alt_kp * (pose.down - z_ref), gains.max_vertical)  # NED down above reference = too low
    yaw_rate = -clamp(gains.yaw_kp * wrap_pi(line.heading - pose.heading), gains.max_yaw_rate)
    return forward, left, up, yaw_rate


def turn_in_place_command(hold_north, hold_east, target_heading, pose, z_ref, gains):
    """(forward, left, up, yaw_rate) to turn to `target_heading` while holding position and height."""
    v_north = clamp(gains.hold_kp * (hold_north - pose.north), gains.max_hold)
    v_east = clamp(gains.hold_kp * (hold_east - pose.east), gains.max_hold)
    forward, left = world_to_body(v_north, v_east, pose.heading)
    up = clamp(gains.alt_kp * (pose.down - z_ref), gains.max_vertical)
    yaw_rate = -clamp(gains.yaw_kp * wrap_pi(target_heading - pose.heading), gains.max_yaw_rate)
    return forward, left, up, yaw_rate
