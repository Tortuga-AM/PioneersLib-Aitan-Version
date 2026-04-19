from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin
from typing import Iterable, List, Optional, Sequence, Tuple

GOAL_STRENGTH = 0.65
FIELD_LENGTH = 16.42
FIELD_WIDTH = 8.16
EPSILON = 1e-5
FORCE_TOLERANCE = 1e-9
MAX_OBSTACLE_INFLUENCE_DISTANCE_M = 4.0
GOAL_TOLERANCE_FACTOR = 1.5
DEFAULT_DT_S = 0.02
DEFAULT_STUCK_WINDOW = 20
DEFAULT_STUCK_PROGRESS_FACTOR = 0.15
DEFAULT_ESCAPE_FORCE_GAIN = 0.9


@dataclass(frozen=True)
class Vector2:
    x: float = 0.0
    y: float = 0.0

    def plus(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x + other.x, self.y + other.y)

    def minus(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x - other.x, self.y - other.y)

    def times(self, scalar: float) -> "Vector2":
        return Vector2(self.x * scalar, self.y * scalar)

    def div(self, scalar: float) -> "Vector2":
        return Vector2(self.x / scalar, self.y / scalar)

    def norm(self) -> float:
        return hypot(self.x, self.y)

    def angle(self) -> float:
        return atan2(self.y, self.x)

    def distance_to(self, other: "Vector2") -> float:
        return hypot(other.x - self.x, other.y - self.y)

    @staticmethod
    def from_polar(magnitude: float, angle_rad: float) -> "Vector2":
        return Vector2(magnitude * cos(angle_rad), magnitude * sin(angle_rad))


def rotate_vector(vector: Vector2, angle_rad: float) -> Vector2:
    return Vector2(
        vector.x * cos(angle_rad) - vector.y * sin(angle_rad),
        vector.x * sin(angle_rad) + vector.y * cos(angle_rad),
    )


class Obstacle:
    def __init__(self, strength: float, should_repel: bool) -> None:
        self.strength = strength
        self.should_repel = should_repel

    def get_force_at_position(self, current_position: Vector2, goal_position: Vector2) -> Vector2:
        raise NotImplementedError("Subclasses must implement get_force_at_position")

    def calculate_force_magnitude(self, distance: float) -> float:
        force_mag = self.strength / (EPSILON + abs(distance * distance))
        return force_mag if self.should_repel else -force_mag

    def calculate_force_magnitude_with_falloff(self, distance: float, falloff: float) -> float:
        original = self.strength / (EPSILON + abs(distance * distance))
        falloff_mag = self.strength / (EPSILON + abs(falloff * falloff))
        mag = max(original - falloff_mag, 0.0)
        return mag if self.should_repel else -mag


class PointObstacle(Obstacle):
    def __init__(self, strength: float, should_repel: bool, obstacle_radius: float, obstacle_location: Vector2) -> None:
        super().__init__(strength, should_repel)
        self.obstacle_location = obstacle_location
        self.obstacle_radius = obstacle_radius

    def get_force_at_position(self, current_position: Vector2, goal_position: Vector2) -> Vector2:
        distance = self.obstacle_location.distance_to(current_position)
        if distance > MAX_OBSTACLE_INFLUENCE_DISTANCE_M:
            return Vector2()

        outward_force_mag = self.calculate_force_magnitude(distance - self.obstacle_radius)
        initial_force = Vector2.from_polar(outward_force_mag, current_position.minus(self.obstacle_location).angle())

        theta = goal_position.minus(current_position).angle() - current_position.minus(self.obstacle_location).angle()
        mag = (outward_force_mag * _sign(sin(theta / 2.0))) / 2.0

        initial_norm = initial_force.norm()
        if initial_norm < FORCE_TOLERANCE:
            return initial_force

        tangent = rotate_vector(initial_force, pi / 2.0).div(initial_norm).times(mag)
        return tangent.plus(initial_force)


class GuidedObstacle(Obstacle):
    def __init__(self, strength: float, should_repel: bool, obstacle_radius: float, obstacle_location: Vector2) -> None:
        super().__init__(strength, should_repel)
        self.obstacle_location = obstacle_location
        self.obstacle_radius = obstacle_radius

    def get_force_at_position(self, current_position: Vector2, goal_position: Vector2) -> Vector2:
        initial_mag = self.calculate_force_magnitude(self.obstacle_location.distance_to(current_position))
        initial_force = Vector2.from_polar(initial_mag, current_position.minus(self.obstacle_location).angle())

        target_to_obstacle = self.obstacle_location.minus(goal_position)
        target_to_obstacle_angle = target_to_obstacle.angle()
        sideways_circle = Vector2.from_polar(self.obstacle_radius, target_to_obstacle.angle()).plus(self.obstacle_location)
        sideways_mag = self.calculate_force_magnitude(sideways_circle.distance_to(current_position))

        sideways_theta = goal_position.minus(current_position).angle() - current_position.minus(sideways_circle).angle()
        sideways_mag *= _sign(sin(sideways_theta))

        sideways_angle = target_to_obstacle_angle + (pi / 2.0)
        return Vector2.from_polar(sideways_mag, sideways_angle).plus(initial_force)


class HorizontalObstacle(Obstacle):
    def __init__(self, strength: float, should_repel: bool, y_position: float, falloff_radius: float) -> None:
        super().__init__(strength, should_repel)
        self.y = y_position
        self.falloff = falloff_radius

    def get_force_at_position(self, current_position: Vector2, goal_position: Vector2) -> Vector2:
        return Vector2(0.0, self.calculate_force_magnitude_with_falloff(self.y - current_position.y, self.falloff))


class VerticalObstacle(Obstacle):
    def __init__(self, strength: float, should_repel: bool, x_position: float, falloff_radius: float) -> None:
        super().__init__(strength, should_repel)
        self.x = x_position
        self.falloff = falloff_radius

    def get_force_at_position(self, current_position: Vector2, goal_position: Vector2) -> Vector2:
        return Vector2(self.calculate_force_magnitude_with_falloff(self.x - current_position.x, self.falloff), 0.0)


DEFAULT_FIELD_OBSTACLES: List[Obstacle] = [
    GuidedObstacle(1.0, True, 0.5, Vector2(4.49, 4.0)),
    GuidedObstacle(1.0, True, 0.5, Vector2(13.08, 4.0)),
]

DEFAULT_WALLS: List[Obstacle] = [
    HorizontalObstacle(0.5, True, 0.0, 1.0),
    HorizontalObstacle(0.5, True, FIELD_WIDTH, 1.0),
    VerticalObstacle(0.5, True, 0.0, 1.0),
    VerticalObstacle(0.5, True, FIELD_LENGTH, 1.0),
    VerticalObstacle(0.5, False, 7.55, 1.0),
    VerticalObstacle(0.5, False, 10.0, 1.0),
]


class RepulsorFieldPlanner:
    def __init__(
        self,
        field_obstacles: Optional[Iterable[Obstacle]] = None,
        wall_obstacles: Optional[Iterable[Obstacle]] = None,
    ) -> None:
        self.field_obstacles = list(field_obstacles) if field_obstacles is not None else list(DEFAULT_FIELD_OBSTACLES)
        self.wall_obstacles = list(wall_obstacles) if wall_obstacles is not None else list(DEFAULT_WALLS)
        self.goal: Optional[Vector2] = None
        self.path_length = 0.0

    def set_goal(self, goal: Vector2) -> None:
        self.goal = goal

    def clear_field_obstacles(self) -> None:
        self.field_obstacles.clear()

    def clear_wall_obstacles(self) -> None:
        self.wall_obstacles.clear()

    def add_field_obstacle(self, obstacle: Obstacle) -> None:
        self.field_obstacles.append(obstacle)

    def add_wall_obstacle(self, obstacle: Obstacle) -> None:
        self.wall_obstacles.append(obstacle)

    def get_goal_force(self, current_location: Vector2, goal: Vector2) -> Vector2:
        displacement = goal.minus(current_location)
        displacement_norm = displacement.norm()
        if displacement_norm == 0.0:
            return Vector2()

        magnitude = GOAL_STRENGTH * (1.0 + 1.0 / (EPSILON + displacement_norm * displacement_norm))
        return Vector2.from_polar(magnitude, displacement.angle())

    def _sum_forces(self, current_location: Vector2, target: Vector2, obstacles: Iterable[Obstacle]) -> Vector2:
        force = Vector2()
        for obstacle in obstacles:
            force = force.plus(obstacle.get_force_at_position(current_location, target))
        return force

    def get_wall_force(self, current_location: Vector2, target: Vector2) -> Vector2:
        return self._sum_forces(current_location, target, self.wall_obstacles)

    def get_obstacle_force(self, current_location: Vector2, target: Vector2) -> Vector2:
        return self._sum_forces(current_location, target, self.field_obstacles)

    def get_force(self, current_location: Vector2, target: Vector2) -> Vector2:
        return self.get_goal_force(current_location, target).plus(
            self.get_obstacle_force(current_location, target)
        ).plus(self.get_wall_force(current_location, target))

    def _escape_force(self, goal_vector: Vector2, iteration: int, base_force: float) -> Vector2:
        if goal_vector.norm() < FORCE_TOLERANCE:
            return Vector2()
        direction = goal_vector.angle() + (pi / 2.0 if iteration % 2 == 0 else -pi / 2.0)
        return Vector2.from_polar(base_force * DEFAULT_ESCAPE_FORCE_GAIN, direction)

    def get_trajectory(
        self,
        current: Vector2,
        goal: Optional[Vector2] = None,
        step_size_m: float = 0.15,
        max_iterations: int = 400,
        max_speed_mps: Optional[float] = None,
        dt_s: float = DEFAULT_DT_S,
        enable_escape: bool = True,
        stuck_window: int = DEFAULT_STUCK_WINDOW,
    ) -> List[Vector2]:
        self.path_length = 0.0
        trajectory: List[Vector2] = []
        robot = current
        target_goal = goal if goal is not None else self.goal
        if target_goal is None:
            raise ValueError("Goal must be provided or set with set_goal() before generating a trajectory")

        constrained_step_size_m = step_size_m
        if max_speed_mps is not None:
            constrained_step_size_m = min(constrained_step_size_m, max_speed_mps * dt_s)

        if constrained_step_size_m <= 0:
            raise ValueError("step_size_m and max_speed_mps*dt_s must be > 0")

        recent_goal_distances: List[float] = []

        for iteration in range(max_iterations):
            displacement_to_goal = target_goal.minus(robot)
            distance_to_goal = displacement_to_goal.norm()
            if distance_to_goal < constrained_step_size_m * GOAL_TOLERANCE_FACTOR:
                trajectory.append(target_goal)
                break

            net_force = self.get_force(robot, target_goal)
            net_force_norm = net_force.norm()
            if net_force_norm < FORCE_TOLERANCE:
                if not enable_escape:
                    break
                net_force = self._escape_force(displacement_to_goal, iteration, GOAL_STRENGTH)
            else:
                recent_goal_distances.append(distance_to_goal)
                if len(recent_goal_distances) > stuck_window:
                    recent_goal_distances.pop(0)
                if enable_escape and len(recent_goal_distances) == stuck_window:
                    progress = recent_goal_distances[0] - recent_goal_distances[-1]
                    minimum_progress = constrained_step_size_m * DEFAULT_STUCK_PROGRESS_FACTOR
                    if progress < minimum_progress:
                        net_force = net_force.plus(self._escape_force(displacement_to_goal, iteration, net_force_norm))
                        recent_goal_distances.clear()

            step = Vector2.from_polar(constrained_step_size_m, net_force.angle())
            robot = robot.plus(step)
            trajectory.append(robot)
            self.path_length += constrained_step_size_m

        return trajectory

    def sample_vector_field(
        self,
        goal: Optional[Vector2] = None,
        x_samples: int = 41,
        y_samples: int = 21,
        normalize: bool = True,
        max_vector_length: float = 0.35,
    ) -> List[Tuple[float, float, float, float]]:
        target_goal = goal if goal is not None else self.goal
        if target_goal is None:
            raise ValueError("Goal must be provided or set with set_goal() before sampling field")

        if x_samples < 2 or y_samples < 2:
            raise ValueError("x_samples and y_samples must each be >= 2")

        vectors: List[Tuple[float, float, float, float]] = []
        for x_idx in range(x_samples):
            x = (x_idx * FIELD_LENGTH) / (x_samples - 1)
            for y_idx in range(y_samples):
                y = (y_idx * FIELD_WIDTH) / (y_samples - 1)
                force = self.get_force(Vector2(x, y), target_goal)
                fx = force.x
                fy = force.y
                force_norm = hypot(fx, fy)
                if normalize and force_norm > FORCE_TOLERANCE:
                    scale = max_vector_length / force_norm
                    fx *= scale
                    fy *= scale
                vectors.append((x, y, fx, fy))
        return vectors


def _sign(value: float) -> float:
    if value > 0:
        return 1.0
    if value < 0:
        return -1.0
    return 0.0


def obstacles_as_sequences(obstacles: Sequence[Obstacle]) -> Tuple[List[Tuple[float, float, float]], List[Tuple[str, float, float, float, bool]]]:
    circles: List[Tuple[float, float, float]] = []
    walls: List[Tuple[str, float, float, float, bool]] = []
    for obstacle in obstacles:
        if isinstance(obstacle, (PointObstacle, GuidedObstacle)):
            circles.append((obstacle.obstacle_location.x, obstacle.obstacle_location.y, obstacle.obstacle_radius))
        elif isinstance(obstacle, HorizontalObstacle):
            walls.append(("h", obstacle.y, obstacle.falloff, obstacle.strength, obstacle.should_repel))
        elif isinstance(obstacle, VerticalObstacle):
            walls.append(("v", obstacle.x, obstacle.falloff, obstacle.strength, obstacle.should_repel))
    return circles, walls
