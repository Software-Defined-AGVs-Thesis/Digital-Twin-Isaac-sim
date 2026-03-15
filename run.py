__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from omegaconf import DictConfig, OmegaConf, ListConfig
from src.configurations import configFactory
from src.environments_wrappers import startSim

from typing import Dict, List
import logging
import hydra
import omni.kit.app


numba_logger = logging.getLogger("numba")
numba_logger.setLevel(logging.WARNING)
matplotlib_logger = logging.getLogger("matplotlib")
matplotlib_logger.setLevel(logging.WARNING)


def resolve_tuple(*args):
    return tuple(args)


OmegaConf.register_new_resolver("as_tuple", resolve_tuple)


def omegaconfToDict(d: DictConfig) -> Dict:
    """Converts an omegaconf DictConfig to a python Dict, respecting variable interpolation.

    Args:
        d (DictConfig): OmegaConf DictConfig.

    Returns:
        Dict: Python dict."""

    if isinstance(d, DictConfig):
        ret = {}
        for k, v in d.items():
            if isinstance(v, DictConfig):
                ret[k] = omegaconfToDict(v)
            elif isinstance(v, ListConfig):
                ret[k] = [omegaconfToDict(i) for i in v]
            else:
                ret[k] = v
    elif isinstance(d, ListConfig):
        ret = [omegaconfToDict(i) for i in d]
    else:
        ret = d

    return ret


def instantiateConfigs(cfg: dict) -> dict:
    """
    Instantiates the configurations. That is if the name of the configuration is in the instantiable_configs list,
    it will create an instance of it.
    """

    instantiable_configs = configFactory.getConfigs()

    ret = {}
    for k, v in cfg.items():
        if isinstance(v, dict):
            if k in instantiable_configs:
                ret[k] = configFactory(k, **v)
            else:
                ret[k] = instantiateConfigs(v)
        else:
            ret[k] = v
    return ret


# =============================================================================
# TurtleBot Controller Integration
# =============================================================================
def start_turtlebot_controller(robot_prim_path="/Robots/turtlebot3_burger/turtlebot3_burger/a__namespace_base_footprint"):
    """
    Start the TurtleBot controller if available.
    
    Args:
        robot_prim_path: USD path to the robot's articulation root
    """
    try:
        import sys
        sys.path.insert(0, "/opt/turtlebot")
        from turtlebot_controller import start_controller
        start_controller(robot_prim_path)
        print("[OmniLRS] TurtleBot controller started successfully")
    except ImportError:
        print("[OmniLRS] TurtleBot controller not found - skipping")
    except Exception as e:
        print(f"[OmniLRS] Failed to start TurtleBot controller: {e}")


@hydra.main(config_name="config", config_path="cfg")
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    SM, simulation_app = startSim(cfg)

    start_turtlebot_controller()
    SM.run_simulation()
    simulation_app.close()


if __name__ == "__main__":
    run()