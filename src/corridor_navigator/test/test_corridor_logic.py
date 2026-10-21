"""Unit tests and a small closed-loop simulation for corridor_logic (no ROS needed)."""
import math

import pytest

from corridor_navigator import corridor_logic as cl


def test_parse_zone_file(tmp_path):
    f = tmp_path / "zone.txt"
    f.write_text("# zone\n13.0000, 80.0000\n13.0001 80.0000\n13.0001;80.0001  # corner\n13.0000,80.0000\n")
    pts = cl.parse_zone_file(str(f))
    assert pts == [(13.0, 80.0), (13.0001, 80.0), (13.0001, 80.0001)]


@pytest.mark.parametrize("text", ["1,2\n3,4\n", "1,2,3\n", "abc,2\n3,4\n5,6\n", "95,2\n3,4\n5,6\n"])
def test_parse_zone_file_rejects_bad_files(tmp_path, text):
    f = tmp_path / "zone.txt"
    f.write_text(text)
    with pytest.raises(ValueError):
        cl.parse_zone_file(str(f))


def test_latlon_to_ne():
    n, e = cl.latlon_to_ne(13.0001, 80.0, 13.0, 80.0)
    assert n == pytest.approx(11.12, abs=0.05) and e == pytest.approx(0.0, abs=1e-6)
    n, e = cl.latlon_to_ne(13.0, 80.0001, 13.0, 80.0)
    assert e == pytest.approx(10.84, abs=0.05) and n == pytest.approx(0.0, abs=1e-6)


def test_point_in_polygon():
    square = [(0, 0), (0, 4), (4, 4), (4, 0)]
    assert cl.point_in_polygon(2, 2, square)
    assert not cl.point_in_polygon(5, 2, square)
    assert not cl.point_in_polygon(2, -0.1, square)
    l_shape = [(0, 0), (0, 4), (2, 4), (2, 2), (4, 2), (4, 0)]
    assert cl.point_in_polygon(3, 1, l_shape) and not cl.point_in_polygon(3, 3, l_shape)


def test_world_to_body_inverts_flight_controller_conversion():
    # flight_controller.cmd_vel_cb: north = f*cos(h) + l*sin(h); east = f*sin(h) - l*cos(h)
    for heading in (0.0, 0.7, -2.0, 3.0):
        f, l = 0.6, -0.25
        v_n = f * math.cos(heading) + l * math.sin(heading)
        v_e = f * math.sin(heading) - l * math.cos(heading)
        assert cl.world_to_body(v_n, v_e, heading) == pytest.approx((f, l))


def test_line_projection_signs():
    line = cl.Line(0.0, 0.0, math.radians(90))  # pointing east
    s, right = line.project(0.0, 5.0)
    assert (s, right) == pytest.approx((5.0, 0.0))
    _, right = line.project(-1.0, 5.0)  # south of an eastbound line = right-hand side
    assert right == pytest.approx(1.0)


def test_run_command_corrects_toward_line_and_height():
    line = cl.Line(0.0, 0.0, 0.0)  # pointing north, drone facing north
    g = cl.Gains()
    pose = cl.Pose(1.0, 0.5, -2.8, 0.0, 0.0)  # right of line by 0.5 m, 0.2 m too low
    forward, left, up, yaw = cl.run_command(line, pose, 0.8, -3.0, g)
    assert forward == pytest.approx(0.8) and left > 0 and up > 0 and yaw == pytest.approx(0.0)
    pose = cl.Pose(1.0, 0.0, -3.0, math.radians(10), 0.0)  # nose 10 deg clockwise of the line
    assert cl.run_command(line, pose, 0.8, -3.0, g)[3] > 0  # turn counter-clockwise


def test_speed_profile_and_stop_target():
    assert cl.speed_profile(0.0, 10.0, 0.8, 0.5) == pytest.approx(0.1)
    assert cl.speed_profile(10.0, 10.0, 0.8, 0.5) == pytest.approx(0.8)
    assert cl.speed_profile(10.0, 0.0, 0.8, 0.5) == pytest.approx(0.1)  # creeps to the target
    assert cl.stop_target("distance", 13.5, 12.0, 1.0, 5.0) == 13.5
    assert cl.stop_target("geofence", 13.5, None, 1.0, 5.0) == 18.5
    assert cl.stop_target("geofence", 13.5, 12.0, 1.0, 5.0) == 13.0
    assert cl.stop_target("either", 13.5, None, 1.0, 5.0) == 13.5
    assert cl.stop_target("either", 13.5, 12.0, 1.0, 5.0) == 13.0
    assert cl.stop_target("either", 13.5, 12.8, 1.0, 5.0) == 13.5


def simulate(heading0_deg, lateral0, wind_east, tau=0.6, dt=0.05, mode="distance"):
    """Closed loop: turn on the spot, then run 13.5 m, with first-order velocity lag and a wind bias."""
    g = cl.Gains()
    target_heading = math.radians(30.0)
    n, e, down = 0.0, lateral0, -3.0
    heading = math.radians(heading0_deg)
    v_n = v_e = yaw_rate = 0.0
    z_ref = -3.0
    state = "turn"
    stable = 0
    hold = (n, e)
    line = None
    t_run = 0.0
    worst_cross = 0.0
    for step in range(int(200 / dt)):
        pose = cl.Pose(n, e, down, heading, math.hypot(v_n, v_e))
        if state == "turn":
            fwd, left, up, yaw_cmd = cl.turn_in_place_command(hold[0], hold[1], target_heading, pose, z_ref, g)
            stable = stable + 1 if abs(cl.wrap_pi(target_heading - heading)) < math.radians(3) else 0
            if stable * dt >= 1.0:
                state, line = "run", cl.Line(n, e, target_heading)
        else:
            s, right = line.project(n, e)
            worst_cross = max(worst_cross, abs(right))
            remaining = 2.5 + 10.0 + 1.0 - s
            if remaining <= 0.15:
                return s, worst_cross, step * dt
            speed = cl.speed_profile(t_run, remaining, 0.8, 0.5)
            fwd, left, up, yaw_cmd = cl.run_command(line, pose, speed, z_ref, g)
            t_run += dt
        # vehicle response, same body->world conversion as flight_controller
        cmd_n = fwd * math.cos(heading) + left * math.sin(heading)
        cmd_e = fwd * math.sin(heading) - left * math.cos(heading)
        v_n += (cmd_n - v_n) * dt / tau
        v_e += (cmd_e - v_e) * dt / tau
        yaw_rate += (-yaw_cmd - yaw_rate) * dt / tau  # FLU ccw -> NED cw
        n += v_n * dt
        e += (v_e + wind_east) * dt
        down += -up * dt
        heading = cl.wrap_pi(heading + yaw_rate * dt)
    raise AssertionError("simulation did not finish")


@pytest.mark.parametrize("heading0,lateral0,wind", [(0, 0.0, 0.0), (-40, 0.3, 0.0), (60, -0.3, 0.2), (170, 0.0, -0.3)])
def test_closed_loop_stays_in_corridor_and_stops_on_target(heading0, lateral0, wind):
    s, worst_cross, _ = simulate(heading0, lateral0, wind)
    assert 13.3 < s < 13.8  # target 13.5 m
    assert worst_cross < 0.5  # corridor half width is 1.75 m
