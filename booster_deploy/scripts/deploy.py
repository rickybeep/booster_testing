import argparse
import sys

sys.path.append(".")

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument(
    "--task", type=str, default="squat", help="Task name (default: squat)."
)
group.add_argument("-l", "--list", action="store_true", dest="list_tasks",
                   default=False, help="list available tasks")

parser.add_argument("--mujoco", action="store_true", default=False,
                    help="deploy in mujoco simulation")
parser.add_argument("--webots", action="store_true", default=False,
                    help="deploy in webots simulation")
args = parser.parse_args()


def main():
    # load task registry and dispatch
    import pkgutil
    import tasks as tasks_pkg

    # auto-import all submodules under tasks (recursive) so they can register themselves
    for mod_info in pkgutil.walk_packages(tasks_pkg.__path__, prefix="tasks."):
        full_name = mod_info.name
        try:
            __import__(full_name)
        except Exception as e:
            raise e
    from booster_deploy.utils.registry import get_task, list_tasks

    if args.list_tasks:
        print("Available tasks:")
        for task_name, cfg in list_tasks().items():
            cls = type(cfg)
            full_cls = f"{cls.__module__}.{cls.__qualname__}"
            print(f"  {task_name}\t:\t{full_cls}")
        sys.exit(0)

    try:
        task_cfg = get_task(args.task)
    except KeyError:
        print(f"Unknown task '{args.task}'. Available tasks: {list(list_tasks().keys())}")
        sys.exit(1)

    # decide how to run based on flags
    if args.mujoco:
        # run mujoco controller
        from booster_deploy.controllers.mujoco_controller import MujocoController

        MujocoController(task_cfg).run()
    else:
        # The high-level SDK changes robot modes. The firmware ROS interface
        # still supplies the low-level state and joint-command message types.
        try:
            import booster_sdk  # noqa: F401
        except ImportError:
            print(
                "Error: booster-sdk is not installed.\n"
                "Run this command through Pixi for real robot deployment.\n"
                "For MuJoCo simulation, use --mujoco flag instead."
            )
            sys.exit(1)
        try:
            import booster_interface  # noqa: F401
        except ImportError:
            print(
                "Error: the ROS 2 'booster_interface' package is not available.\n"
                "Real-robot deployment needs the firmware-provided low-level "
                "ROS interface in addition to booster-sdk.\n"
                "The Pixi deploy task sources both /opt/ros/humble/setup.bash "
                "and /opt/booster/BoosterRos2Interface/install/setup.bash; "
                "run it on the robot and verify that the overlay provides "
                "the booster_interface package.\n"
                "For simulation, run: pixi run deploy-mujoco"
            )
            sys.exit(1)

        # adjust ankle dampings for webots
        if args.webots:
            ankles = [-8, -7, -2, -1]  # indices of ankle joints
            for i in ankles:
                task_cfg.robot.joint_damping[i] = 0.5

        from booster_deploy.controllers.booster_robot_controller import BoosterRobotPortal
        with BoosterRobotPortal(task_cfg, use_sim_time=args.webots) as portal:
            portal.run()


if __name__ == "__main__":
    main()
