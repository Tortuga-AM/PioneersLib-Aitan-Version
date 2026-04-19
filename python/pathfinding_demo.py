from repulsor_field_planner import RepulsorFieldPlanner, Vector2


def main() -> None:
    planner = RepulsorFieldPlanner()

    start = Vector2(1.0, 1.0)
    goal = Vector2(15.0, 7.0)
    planner.set_goal(goal)

    path = planner.get_trajectory(start, step_size_m=0.15)

    print(f"Computed {len(path)} path points")
    print(f"Approximate path length: {planner.path_length:.2f} m")
    print("First 10 points:")
    for i, p in enumerate(path[:10], start=1):
        print(f"  {i:02d}: ({p.x:.3f}, {p.y:.3f})")


if __name__ == "__main__":
    main()
