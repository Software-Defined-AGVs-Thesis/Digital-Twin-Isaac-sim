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
# JetAuto Controller Integration
# =============================================================================
def start_jetauto_controller(robot_prim_path="/Robots/jetauto/base_link"):
    """
    Start the JetAuto mecanum wheel controller.
    Articulation root is at /Robots/jetauto/base_link
    """
    try:
        import sys
        sys.path.insert(0, "/opt/jetauto")
        from jetauto_controller import start_controller
        start_controller(robot_prim_path)
        print("[OmniLRS] JetAuto controller started successfully")
    except ImportError as e:
        print(f"[OmniLRS] JetAuto controller import failed: {e}")
    except Exception as e:
        print(f"[OmniLRS] Failed to start JetAuto controller: {e}")


@hydra.main(config_name="config", config_path="cfg")
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    SM, simulation_app = startSim(cfg)

    try:
        import omni.kit.app
        app = omni.kit.app.get_app()
        extension_manager = app.get_extension_manager()

        extension_manager.set_extension_enabled_immediate("omni.kit.xr.system.steamvr", True)
        extension_manager.set_extension_enabled_immediate("omni.kit.xr.profile.vr", True)
        print("[OmniLRS] VR extensions enabled")

        for i in range(30):
            app.update()
        print("[OmniLRS] VR extension frames pumped")

        # ── DIAGNOSTIC BLOCK ──────────────────────────────────────────
        try:
            import omni.kit.xr.core as xr_core
            print("[DIAG] XRCoreEventType members:", dir(xr_core.XRCoreEventType))
            xr_interface = xr_core.get_xr_interface()
            print("[DIAG] xr_interface:", xr_interface)
            print("[DIAG] xr_interface type:", type(xr_interface))
        except Exception as e:
            print(f"[DIAG] xr_core probe failed: {e}")
        # ── END DIAGNOSTIC ────────────────────────────────────────────

        try:
            import omni.kit.xr.core as xr_core
            xr_interface = xr_core.get_xr_interface()
            if xr_interface is not None:
                xr_interface.start_vr()
                for i in range(60):
                    app.update()
                print("[OmniLRS] VR session pre-initialized successfully")
            else:
                print("[OmniLRS] WARNING: XR interface not found, VR will need manual start")
        except Exception as e:
            print(f"[OmniLRS] VR pre-init failed: {e} — will fall back to manual Start VR")

    except Exception as e:
        print(f"[OmniLRS] Failed to enable VR extensions: {e}")

    start_jetauto_controller()
    SM.run_simulation()
    simulation_app.close()

if __name__ == "__main__":
    run()

# __author__ = "Antoine Richard"
# __copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
# __license__ = "BSD 3-Clause"
# __version__ = "2.0.0"
# __maintainer__ = "Antoine Richard"
# __email__ = "antoine.richard@uni.lu"
# __status__ = "development"

# from omegaconf import DictConfig, OmegaConf, ListConfig
# from src.configurations import configFactory
# from src.environments_wrappers import startSim

# from typing import Dict, List
# import logging
# import hydra
# import omni.kit.app

# numba_logger = logging.getLogger("numba")
# numba_logger.setLevel(logging.WARNING)
# matplotlib_logger = logging.getLogger("matplotlib")
# matplotlib_logger.setLevel(logging.WARNING)


# def resolve_tuple(*args):
#     return tuple(args)


# OmegaConf.register_new_resolver("as_tuple", resolve_tuple)


# def omegaconfToDict(d: DictConfig) -> Dict:
#     if isinstance(d, DictConfig):
#         ret = {}
#         for k, v in d.items():
#             if isinstance(v, DictConfig):
#                 ret[k] = omegaconfToDict(v)
#             elif isinstance(v, ListConfig):
#                 ret[k] = [omegaconfToDict(i) for i in v]
#             else:
#                 ret[k] = v
#     elif isinstance(d, ListConfig):
#         ret = [omegaconfToDict(i) for i in d]
#     else:
#         ret = d
#     return ret


# def instantiateConfigs(cfg: dict) -> dict:
#     instantiable_configs = configFactory.getConfigs()
#     ret = {}
#     for k, v in cfg.items():
#         if isinstance(v, dict):
#             if k in instantiable_configs:
#                 ret[k] = configFactory(k, **v)
#             else:
#                 ret[k] = instantiateConfigs(v)
#         else:
#             ret[k] = v
#     return ret


# # =============================================================================
# # JetAuto Controller Integration
# # =============================================================================
# def start_jetauto_controller(robot_prim_path="/Robots/jetauto/base_link"):
#     """
#     Start the JetAuto mecanum wheel controller.
#     Articulation root is at /Robots/jetauto/base_link
#     """
#     try:
#         import sys
#         sys.path.insert(0, "/opt/jetauto")
#         from jetauto_controller import start_controller
#         start_controller(robot_prim_path)
#         print("[OmniLRS] JetAuto controller started successfully")
#     except ImportError as e:
#         print(f"[OmniLRS] JetAuto controller import failed: {e}")
#     except Exception as e:
#         print(f"[OmniLRS] Failed to start JetAuto controller: {e}")


# @hydra.main(config_name="config", config_path="cfg")
# def run(cfg: DictConfig):
#     cfg = omegaconfToDict(cfg)
#     cfg = instantiateConfigs(cfg)
#     SM, simulation_app = startSim(cfg)

#     start_jetauto_controller()

#     # Shut down ROS2 threads before VR
#     try:
#         print("[OmniLRS] Shutting down ROS2 executor threads...")
#         SM.exec1.shutdown(timeout_sec=1)
#         SM.exec2.shutdown(timeout_sec=1)
#         SM.exec1_thread.join(timeout=2)
#         SM.exec2_thread.join(timeout=2)
#         print("[OmniLRS] ROS2 threads stopped")
#     except Exception as e:
#         print(f"[OmniLRS] Warning: {e}")

#     SM.run_simulation()
#     simulation_app.close()

# if __name__ == "__main__":
#     run()