"""Runs every lesson's plain-Python part and checks the LaserScan direction helper.

Needs ROS 2 Humble sourced (each lesson file imports rclpy at module level).
Run: source /opt/ros/humble/setup.bash && python3 -m pytest -q tests
"""
import importlib.util
import math
from pathlib import Path

import pytest
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parents[1]
LESSONS = sorted(ROOT.glob("[0-9][0-9]_*/*.py"))
RAY_INDEX_LESSONS = [p for p in LESSONS if "def ray_index" in p.read_text()]


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_all_19_lessons_found():
    assert len(LESSONS) == 19


@pytest.mark.parametrize("path", LESSONS, ids=lambda p: p.parent.name)
def test_general_example_runs(path, monkeypatch, tmp_path):
    monkeypatch.setattr("builtins.input", lambda prompt="": "6")
    monkeypatch.chdir(tmp_path)  # 16_file_handling writes files
    load(path).general_example()


def scan(angle_min, n=360):
    """Scan where each ray's range encodes its own angle in degrees (0..359)."""
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_increment = 2 * math.pi / n
    msg.ranges = [
        float(round(math.degrees(angle_min + i * msg.angle_increment)) % 360)
        for i in range(n)
    ]
    return msg


@pytest.mark.parametrize("path", RAY_INDEX_LESSONS, ids=lambda p: p.parent.name)
@pytest.mark.parametrize("angle_min", [0.0, -math.pi], ids=["0..2pi", "-pi..pi"])
def test_ray_index_points_where_it_says(path, angle_min):
    ray_index = load(path).ray_index
    msg = scan(angle_min)
    assert msg.ranges[ray_index(msg, 0.0)] == 0.0        # front
    assert msg.ranges[ray_index(msg, math.pi / 2)] == 90.0   # left (REP 103: +y)
    assert msg.ranges[ray_index(msg, -math.pi / 2)] == 270.0  # right


def test_old_fraction_indexing_was_wrong():
    """The removed len(ranges)//2, //4 picks: on a 0..2pi scan //2 is the back."""
    msg = scan(0.0)
    n = len(msg.ranges)
    assert msg.ranges[n // 2] == 180.0
    msg = scan(-math.pi)
    assert msg.ranges[n // 4] == 270.0  # labelled 'left', is right


def scan_nodes():
    """(lesson, Node subclass) for every node class that defines scan_callback."""
    import inspect
    from rclpy.node import Node
    out = []
    for path in LESSONS:
        for name, cls in inspect.getmembers(load(path), inspect.isclass):
            if issubclass(cls, Node) and cls is not Node and "scan_callback" in vars(cls):
                out.append(pytest.param(path, name, id=f"{path.parent.name}.{name}"))
    return out


@pytest.fixture(scope="module")
def ros():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize("path,cls_name", scan_nodes())
@pytest.mark.parametrize("angle_min", [0.0, -math.pi], ids=["0..2pi", "-pi..pi"])
def test_scan_callback_runs_on_both_layouts(ros, path, cls_name, angle_min, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)  # 16_file_handling logs to a file
    node = getattr(load(path), cls_name)()
    try:
        node.scan_callback(scan(angle_min))
    finally:
        node.destroy_node()


@pytest.mark.parametrize("angle_min", [0.0, -math.pi], ids=["0..2pi", "-pi..pi"])
@pytest.mark.parametrize("obstacle_deg,sector", [(0, "front"), (90, "left"), (270, "right")])
def test_sets_lesson_names_the_right_sector(ros, angle_min, obstacle_deg, sector):
    msg = scan(angle_min)
    msg.ranges = [0.3 if d == obstacle_deg else 5.0 for d in msg.ranges]
    node = load(ROOT / "11_sets" / "sets.py").ObstacleTrackerNode()
    try:
        node.scan_callback(msg)
        assert node.previous_obstacles_ == {sector}
    finally:
        node.destroy_node()
