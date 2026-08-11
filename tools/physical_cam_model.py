from dataclasses import dataclass


@dataclass(frozen=True)
class Recipe:
    pitch: float = 63.7
    conveyor_velocity: float = 5.0
    cut_velocity: float = 3.0
    edge_length: float = 3.0
    cutter_ready: float = 17.0

    @property
    def first_cutter_velocity(self) -> float:
        return -(self.conveyor_velocity - self.cut_velocity)

    @property
    def second_cutter_velocity(self) -> float:
        return -(self.conveyor_velocity + self.cut_velocity)


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    slope: float


def derive_points(recipe: Recipe) -> list[Point]:
    return [
        Point(0.0, 0.0, 0.0),
        Point(1.0, -1.0, -0.4),
        Point(6.0, -3.0, -0.4),
        Point(10.0, -7.0, -1.6),
        Point(15.0, -15.0, -1.6),
        Point(17.0, -16.6, 0.0),
        Point(27.0, 0.0, 0.0),
        Point(recipe.pitch, 0.0, 0.0),
    ]


def object_relative_position(point: Point) -> float:
    return -(point.y + point.x)


def shortest_periodic_target(
    current: float, phase: float, pitch: float
) -> float:
    if pitch <= 0.0:
        raise ValueError("pitch must be positive")
    delta = (phase - current + pitch / 2.0) % pitch - pitch / 2.0
    return current + delta


def quintic_peak_return_dynamics(
    distance: float, seconds: float
) -> tuple[float, float, float]:
    peak_velocity = 1.875 * distance / seconds
    peak_acceleration = 5.7735026919 * distance / (seconds * seconds)
    peak_jerk = 60.0 * distance / (seconds * seconds * seconds)
    return peak_velocity, peak_acceleration, peak_jerk
