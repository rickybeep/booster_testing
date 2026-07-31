from booster_deploy.utils.registry import register_task

from .squat import K1SquatControllerCfg


register_task("squat", K1SquatControllerCfg())

