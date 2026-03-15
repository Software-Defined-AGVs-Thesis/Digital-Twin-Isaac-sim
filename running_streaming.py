"""
Run OmniLRS with WebRTC streaming enabled.
This is a wrapper - no edits to original files needed.
"""

import sys
import os

# Change to OmniLRS directory
os.chdir("/workspace/omnilrs")
sys.path.insert(0, "/workspace/omnilrs")

# Import and configure BEFORE starting SimulationApp
from omegaconf import DictConfig, OmegaConf, ListConfig
from src.configurations import configFactory
import logging
import hydra

numba_logger = logging.getLogger("numba")
numba_logger.setLevel(logging.WARNING)
matplotlib_logger = logging.getLogger("matplotlib")
matplotlib_logger.setLevel(logging.WARNING)

def resolve_tuple(*args):
    return tuple(args)

OmegaConf.register_new_resolver("as_tuple", resolve_tuple, replace=True)

def omegaconfToDict(d):
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

def instantiateConfigs(cfg):
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

@hydra.main(config_name="config", config_path="cfg", version_base=None)
def run(cfg: DictConfig):
    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    
    # Import here after hydra setup
    from omni.isaac.kit import SimulationApp
    import omni
    from src.environments.rendering import set_lens_flares, set_chromatic_aberrations, set_motion_blur

    class SimulationApp_wait(SimulationApp):
        def __init__(self, launch_config=None, experience=""):
            super().__init__(launch_config, experience)
            self.wait_for_threads = []

        def add_wait(self, waiting_functions):
            self.wait_for_threads += waiting_functions

        def close(self, wait_for_replicator=True):
            try:
                import omni.replicator.core as rep
                if rep.orchestrator.get_status() not in [
                    rep.orchestrator.Status.STOPPED,
                    rep.orchestrator.Status.STOPPING,
                ]:
                    rep.orchestrator.stop()
                if wait_for_replicator:
                    rep.orchestrator.wait_until_complete()
                rep.orchestrator.set_capture_on_play(False)
            except Exception:
                pass

            for wait in self.wait_for_threads:
                self._app.print_and_log(f"Waiting for external thread to join: {wait}")
                wait()

            if omni.usd.get_context().can_close_stage():
                omni.usd.get_context().close_stage()

            if not self._exiting:
                self._exiting = True
                self._app.print_and_log("Simulation App Shutting Down")

                def is_stage_loading():
                    import omni.usd
                    context = omni.usd.get_context()
                    if context is None:
                        return False
                    else:
                        _, _, loading = context.get_stage_loading_status()
                        return loading > 0

                if is_stage_loading():
                    print("Waiting for USD resource operations to complete...")
                while is_stage_loading():
                    self._app.update()

                self._app.shutdown()
                self._framework.unload_all_plugins()
                print("Simulation App Shutdown Complete")

    # Start simulation
    renderer_cfg = cfg["rendering"]["renderer"]
    launch_config = renderer_cfg.__dict__.copy()
    simulation_app = SimulationApp_wait(launch_config)
    
    set_lens_flares(cfg)
    set_motion_blur(cfg)
    set_chromatic_aberrations(cfg)

    # === ENABLE WEBRTC STREAMING ===
    print("[Streaming] Enabling WebRTC extensions...")
    import carb.settings
    import omni.kit.app
    
    settings = carb.settings.get_settings()
    settings.set("/app/livestream/enabled", True)
    settings.set("/app/livestream/port", 49100)
    
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled("omni.kit.livestream.core", True)
    ext_manager.set_extension_enabled("omni.kit.livestream.webrtc", True)
    ext_manager.set_extension_enabled("omni.services.streaming.manager", True)
    ext_manager.set_extension_enabled("omni.services.streamclient.webrtc", True)
    print("[Streaming] WebRTC extensions enabled!")
    # === END STREAMING SETUP ===

    # ROS2 mode
    if cfg["mode"]["name"] == "ROS2":
        from src.environments_wrappers.ros2 import enable_ros2
        enable_ros2(simulation_app, bridge_name=cfg["mode"]["bridge_name"])
        import rclpy
        rclpy.init()
        from src.environments_wrappers.ros2.simulation_manager_ros2 import ROS2_SimulationManager
        SM = ROS2_SimulationManager(cfg, simulation_app)

    # ROS1 mode
    if cfg["mode"]["name"] == "ROS1":
        from src.environments_wrappers.ros1 import enable_ros1
        enable_ros1(simulation_app)
        import rospy
        rospy.init_node("omni_isaac_ros1")
        from src.environments_wrappers.ros1.simulation_manager_ros1 import ROS1_SimulationManager
        SM = ROS1_SimulationManager(cfg, simulation_app)

    # SDG mode
    if cfg["mode"]["name"] == "SDG":
        from src.environments_wrappers.sdg.simulation_manager_sdg import SDG_SimulationManager
        SM = SDG_SimulationManager(cfg, simulation_app)

    SM.run_simulation()
    simulation_app.close()

if __name__ == "__main__":
    run()