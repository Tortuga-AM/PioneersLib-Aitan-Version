from __future__ import annotations

import argparse

from repulsor_field_planner import (
    FIELD_LENGTH,
    FIELD_WIDTH,
    GuidedObstacle,
    HorizontalObstacle,
    PointObstacle,
    RepulsorFieldPlanner,
    Vector2,
    VerticalObstacle,
    obstacles_as_sequences,
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Repulsor pathfinding visualization demo")
    parser.add_argument("--start", nargs=2, type=float, default=[1.0, 1.0], metavar=("X", "Y"))
    parser.add_argument("--goal", nargs=2, type=float, default=[15.0, 7.0], metavar=("X", "Y"))
    parser.add_argument("--step-size", type=float, default=0.15)
    parser.add_argument("--iterations", type=int, default=400)
    parser.add_argument("--top-speed", type=float, default=3.0, help="Max robot speed (m/s)")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--no-escape", action="store_true", help="Disable anti-minima escape logic")
    parser.add_argument("--save-image", type=str, default="")
    parser.add_argument("--no-show", action="store_true")

    parser.add_argument(
        "--point-obstacle",
        nargs=4,
        action="append",
        metavar=("X", "Y", "RADIUS", "STRENGTH"),
        default=[],
        help="Add repelling point obstacle",
    )
    parser.add_argument(
        "--guided-obstacle",
        nargs=4,
        action="append",
        metavar=("X", "Y", "RADIUS", "STRENGTH"),
        default=[],
        help="Add repelling guided obstacle",
    )
    parser.add_argument(
        "--h-wall",
        nargs=3,
        action="append",
        metavar=("Y", "FALLOFF", "STRENGTH"),
        default=[],
        help="Add horizontal repelling wall",
    )
    parser.add_argument(
        "--v-wall",
        nargs=3,
        action="append",
        metavar=("X", "FALLOFF", "STRENGTH"),
        default=[],
        help="Add vertical repelling wall",
    )
    return parser.parse_args()


def build_planner(args: argparse.Namespace) -> tuple[RepulsorFieldPlanner, Vector2, Vector2]:
    start = Vector2(args.start[0], args.start[1])
    goal = Vector2(args.goal[0], args.goal[1])

    planner = RepulsorFieldPlanner()
    planner.set_goal(goal)

    for x, y, radius, strength in args.point_obstacle:
        planner.add_field_obstacle(PointObstacle(float(strength), True, float(radius), Vector2(float(x), float(y))))

    for x, y, radius, strength in args.guided_obstacle:
        planner.add_field_obstacle(GuidedObstacle(float(strength), True, float(radius), Vector2(float(x), float(y))))

    for y, falloff, strength in args.h_wall:
        planner.add_wall_obstacle(HorizontalObstacle(float(strength), True, float(y), float(falloff)))

    for x, falloff, strength in args.v_wall:
        planner.add_wall_obstacle(VerticalObstacle(float(strength), True, float(x), float(falloff)))

    return planner, start, goal


def visualize(planner: RepulsorFieldPlanner, start: Vector2, goal: Vector2, path: list[Vector2], args: argparse.Namespace) -> None:
    try:
        import matplotlib
        if args.no_show and args.save_image:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        if not args.save_image:
            raise RuntimeError(
                "matplotlib is required for interactive visualization. "
                "Install matplotlib or pass --save-image <file.svg> to export an SVG."
            )
        _save_svg_visualization(planner, start, goal, path, args.save_image)
        return

    fig, ax = plt.subplots(figsize=(12, 6))

    vectors = planner.sample_vector_field(goal=goal, x_samples=45, y_samples=23)
    xs = [v[0] for v in vectors]
    ys = [v[1] for v in vectors]
    us = [v[2] for v in vectors]
    vs = [v[3] for v in vectors]
    ax.quiver(xs, ys, us, vs, angles="xy", scale_units="xy", scale=1.0, alpha=0.35, color="#1f77b4")

    all_obstacles = planner.field_obstacles + planner.wall_obstacles
    circles, walls = obstacles_as_sequences(all_obstacles)

    for ox, oy, radius in circles:
        circle = plt.Circle((ox, oy), radius, color="tab:red", fill=False, linewidth=2)
        ax.add_patch(circle)

    for axis, pos, _, _, _ in walls:
        if axis == "h":
            ax.axhline(pos, color="tab:orange", linestyle="--", linewidth=1.2)
        else:
            ax.axvline(pos, color="tab:orange", linestyle="--", linewidth=1.2)

    if path:
        ax.plot([p.x for p in path], [p.y for p in path], color="tab:green", linewidth=2.5, label="robot path")

    ax.scatter([start.x], [start.y], color="tab:blue", s=90, label="start", zorder=5)
    ax.scatter([goal.x], [goal.y], color="tab:purple", s=90, label="goal", zorder=5)

    ax.set_title("Repulsor Vector Field and Robot Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xlim(0, FIELD_LENGTH)
    ax.set_ylim(0, FIELD_WIDTH)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right")

    if args.save_image:
        fig.savefig(args.save_image, dpi=160, bbox_inches="tight")

    if not args.no_show:
        plt.show()

    plt.close(fig)


def _save_svg_visualization(
    planner: RepulsorFieldPlanner,
    start: Vector2,
    goal: Vector2,
    path: list[Vector2],
    output_path: str,
) -> None:
    width_px = 1200
    height_px = 600

    def sx(x: float) -> float:
        return (x / FIELD_LENGTH) * width_px

    def sy(y: float) -> float:
        return height_px - (y / FIELD_WIDTH) * height_px

    vectors = planner.sample_vector_field(goal=goal, x_samples=40, y_samples=20)
    all_obstacles = planner.field_obstacles + planner.wall_obstacles
    circles, walls = obstacles_as_sequences(all_obstacles)

    pieces: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width_px} {height_px}">',
        '<rect x="0" y="0" width="100%" height="100%" fill="white" />',
    ]

    for axis, pos, _, _, _ in walls:
        if axis == "h":
            y = sy(pos)
            pieces.append(f'<line x1="0" y1="{y:.2f}" x2="{width_px}" y2="{y:.2f}" stroke="#ff7f0e" stroke-dasharray="8,6" stroke-width="2"/>')
        else:
            x = sx(pos)
            pieces.append(f'<line x1="{x:.2f}" y1="0" x2="{x:.2f}" y2="{height_px}" stroke="#ff7f0e" stroke-dasharray="8,6" stroke-width="2"/>')

    for x, y, radius in circles:
        pieces.append(
            f'<circle cx="{sx(x):.2f}" cy="{sy(y):.2f}" r="{(radius / FIELD_LENGTH) * width_px:.2f}" '
            'fill="none" stroke="#d62728" stroke-width="2"/>'
        )

    for x, y, u, v in vectors:
        x1 = sx(x)
        y1 = sy(y)
        x2 = sx(x + u)
        y2 = sy(y + v)
        pieces.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="#1f77b4" stroke-width="1.2" opacity="0.45"/>'
        )

    if path:
        path_points = " ".join(f"{sx(p.x):.2f},{sy(p.y):.2f}" for p in path)
        pieces.append(f'<polyline points="{path_points}" fill="none" stroke="#2ca02c" stroke-width="3"/>')

    pieces.append(f'<circle cx="{sx(start.x):.2f}" cy="{sy(start.y):.2f}" r="7" fill="#1f77b4"/>')
    pieces.append(f'<circle cx="{sx(goal.x):.2f}" cy="{sy(goal.y):.2f}" r="7" fill="#9467bd"/>')
    pieces.append("</svg>")

    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write("\n".join(pieces))


def main() -> None:
    args = parse_arguments()
    planner, start, goal = build_planner(args)

    path = planner.get_trajectory(
        start,
        goal=goal,
        step_size_m=args.step_size,
        max_iterations=args.iterations,
        max_speed_mps=args.top_speed,
        dt_s=args.dt,
        enable_escape=not args.no_escape,
    )

    print(f"Computed {len(path)} path points")
    print(f"Approximate path length: {planner.path_length:.2f} m")
    print(f"Top speed constraint: {args.top_speed:.2f} m/s")
    print("First 10 points:")
    for i, point in enumerate(path[:10], start=1):
        print(f"  {i:02d}: ({point.x:.3f}, {point.y:.3f})")

    visualize(planner, start, goal, path, args)


if __name__ == "__main__":
    main()
