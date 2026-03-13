from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import define_prim
from pxr import UsdGeom, Sdf

# app/world.py 등 공용 유틸에 넣어두면 좋음
import omni.usd, omni.kit.app, omni.kit.commands as kitcmd
from pxr import Gf, Sdf

'''
def ensure_lidar_prims(lidar_cfg: dict):
    """
    RTX LiDAR 프림이 스테이지에 없으면 생성한다.
    - lidar_cfg["prim_2d"], lidar_cfg["prim_3d"] 같은 절대 경로 사용
    - 이미 있으면 그대로 둔다 (삭제하지 않음)
    """
    app = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    def _create_if_missing(path: str, config: str):
        if not path:
            return
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            print(f"[LIDAR] already exists: {path}")
            return
        print(f"[LIDAR] creating: {path} with config={config}")
        ok, created = kitcmd.execute(
            "IsaacSensorCreateRtxLidar",
            path=path,                        # 절대 경로 필수
            config=config,                    # 예: "Example_Rotary" 또는 "RPLIDAR_S2E"
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            #apply_default_config=True 
        )
        for _ in range(3): app.update()
        print(f"[LIDAR] created ok={ok}, prim_valid={stage.GetPrimAtPath(path).IsValid()}")

    # 필요하다면 2D/3D 둘 다 생성
    _create_if_missing(lidar_cfg.get("prim_2d"), lidar_cfg.get("config_2d", "Example_Rotary_2D"))
    _create_if_missing(lidar_cfg.get("prim_3d"), lidar_cfg.get("config_3d", "Example_Rotary"))

    '''

class SimWorld:
    def __init__(self, usd_path: str, spot_prim: str, ats_prim: str, imu_dummy_prim: str,
                 fixed_time_step: bool, play_every_frame: bool, target_hz: int,
                 lidar_cfg: dict | None = None):
        
        import omni.usd
        import omni.timeline
        import carb.settings


        # 스테이지 열기
        # Stage Open
        open_stage(usd_path)
        # Robot을 Physics simulation의 Articulation으로 연결
        self.stage = omni.usd.get_context().get_stage()


        # Articulation root prim 탐색기 함수
        def _find_articulation_root(stage, base_path: str) -> str | None:
            from pxr import Sdf, Usd, UsdPhysics
            base = stage.GetPrimAtPath(base_path)
            if not base.IsValid():
                return None
            

            # 경로의 하위 경로들을 순회하여 진짜 루트를 찾아낸다
            if UsdPhysics.ArticulationRootAPI.CanApply(base):
                if UsdPhysics.ArticulationRootAPI(base):
                    return base.GetPath().pathString

            it = Usd.PrimRange(base)
            for prim in it:
                if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
                    return prim.GetPath().pathString
            return None

        settings = carb.settings.acquire_settings_interface()



        # 시간과 타임라인 고정
        timeline = omni.timeline.get_timeline_interface()

        # fixed_time_step: 목표 주파수를 설정해 ROS2와 시간축 통일
        if fixed_time_step:
            settings.set("/app/player/useFixedTimeStepping", True)
        timeline.set_play_every_frame(play_every_frame)
        settings.set("/app/player/targetRunLoopFrequency", int(target_hz))
        timeline.play()
        
        #define_prim("/World/Spot/base_link", "Xform")
        #define_prim(imu_dummy_prim, "Xform")

        # 오돔과 지도좌표계 즉 map과 구분하기 위해 스테이지상 xform으로 고정점생성함
        define_prim("/World/odom", "Xform")

        #cam_path = "/World/Spot/base_link/Camera"
        #cam_path = "/World/Spot/ATS/ATS/link2/Xform/Camera"
        #if not self.stage.GetPrimAtPath(cam_path).IsValid():
            #UsdGeom.Camera.Define(self.stage, Sdf.Path(cam_path))

        spot_root = _find_articulation_root(self.stage, spot_prim)
        ats_root  = _find_articulation_root(self.stage, ats_prim)

        self.world = World()
        self.world.reset()

        if not spot_root:
            raise RuntimeError(f"[SimWorld] Could not find ArticulationRoot under '{spot_prim}'")
        
        # Articulation View를 만들어 물리엔진과 연동함
        # Articulation View는 이후 관절상태 조회, 목표치 설정의 표준 통로가 된다
        self.spot = ArticulationView(prim_paths_expr=spot_root, name="spot_view")
        self.world.scene.add(self.spot)
        self.world.play()

        self.ats = None
        if ats_root:
            self.ats = ArticulationView(prim_paths_expr=ats_root, name="ats_view")
            self.world.scene.add(self.ats)

    def step(self, render=True):
        self.world.step(render=render)
        

