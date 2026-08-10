from booster_deploy.utils.registry import register_task

from .walk import K1WalkControllerCfg


register_task("walk", K1WalkControllerCfg())
