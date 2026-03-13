# app/graph_builder.py
from pxr import Sdf
from omni.isaac.core.utils.extensions import enable_extension


class GraphBuilder:
    """
    Isaac Sim <-> ROS 2 브릿지용 OmniGraph를 구성합니다.
      - 카메라 RGB/CameraInfo 퍼블리시
      - Spot 제어(vel/joint), JointState 퍼블리시
      - Clock / IMU / Odometry / TF 트리 퍼블리시
      - RTX LiDAR(2D/3D) LaserScan/PointCloud 퍼블리시
    """
    def __init__(self, assets_cfg: dict, ros_cfg: dict):
        import omni.usd
        import omni.timeline
        

        self.assets = assets_cfg
        self.ros = ros_cfg

        # 타임라인 재생
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.play()

        # 스테이지 핸들
        self.stage = omni.usd.get_context().get_stage()

        # 필요한 익스텐션 활성화
        for ext in [
            "omni.anim.people",
            "omni.anim.graph.bundle",
            "omni.kit.scripting",
            "omni.anim.graph.ui",
            "omni.anim.graph.schema",
            "omni.anim.navigation.schema",
            "omni.isaac.ros2_bridge",
            "omni.isaac.sensor",           # RTX LiDAR
        ]:
            
            # 없는 플로그인이면 프로그램이 터지지 않도록 try-except와 pass로 방ㅇ어
            try:
                enable_extension(ext)
            except Exception:
                # 없는 경우도 있으므로 무시
                pass

        # 캐릭터/애니메이션용 루트 (기존 코드 호환용)
        self._define_character_graph_roots()

    # --------------------------------------------------------------------- #
    # 내부 유틸
    # --------------------------------------------------------------------- #
    def _define_character_graph_roots(self):
        from omni.isaac.core.utils.prims import define_prim
        define_prim("/World/CustomGraph", "Xform")
        define_prim("/World/CustomGraph/CharacterAnimation", "Xform")
        define_prim("/World/CustomGraph/CharacterAnimation/AnimationGraph", "AnimationGraph")

    # --------------------------------------------------------------------- #
    # 카메라 ROS 퍼블리셔 그래프 / 로봇 눈에 달리 카메라 화면을 찍어서 ROS 2로 쏘는 역할을 수행
    # --------------------------------------------------------------------- #
    def build_camera_ros_graph(self, graph_path: str = "/ActionGraph"):
        """
        - IsaacCreateRenderProduct를 사용하여 해상도(width/height)를 직접 설정
        - ROS2CameraHelper로 RGB/Info/Depth 퍼블리시
        """
        import omni.graph.core as og
        keys = og.Controller.Keys
        
        # 카메라 경로 지정
        camera_prim_path = "/World/Spot/ATS/ATS/link2/Xform/Camera"

        (graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "push",
                # ondemand: 매 프레임마다 자동으로 평가되는게 아니라 필요할 때 코드가 직접 밀어넣는 push방식으로 평가된다는 뜻
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    
                    # [변경] 해상도 조절을 위해 Viewport 노드 대신 RenderProduct 노드 사용 (Isaac Sim 5.0 표준)
                    ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],

                keys.CONNECT: [
                    # 1. 실행 순서 연결 (Tick -> RenderProduct -> Helpers)
                    ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),

                    # 2. 렌더링 된 데이터 경로 전달
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                ],

                keys.SET_VALUES: [
                    # [핵심] 해상도 및 카메라 설정
                    ("createRenderProduct.inputs:width", 640),
                    ("createRenderProduct.inputs:height", 480),
                    ("createRenderProduct.inputs:cameraPrim", Sdf.Path(camera_prim_path)),
                    ("createRenderProduct.inputs:enabled", True),

                    # RGB Helper 설정
                    ("cameraHelperRgb.inputs:frameId", "Camera"),
                    ("cameraHelperRgb.inputs:topicName", "yolo/image_raw"),
                    ("cameraHelperRgb.inputs:type", "rgb"),

                    # Camera Info Helper 설정
                    ("cameraHelperInfo.inputs:frameId", "Camera"),
                    ("cameraHelperInfo.inputs:topicName", "camera_info"),
                    
                    # Depth Helper 설정
                    ("cameraHelperDepth.inputs:frameId", "Camera"),
                    ("cameraHelperDepth.inputs:topicName", "depth"),
                    ("cameraHelperDepth.inputs:type", "depth"),
                ],
            },
        )
        og.Controller.evaluate_sync(graph)
    # --------------------------------------------------------------------- #
    # Spot/ATS + 시계/IMU/Odom/TF 그래프
    # --------------------------------------------------------------------- #
    def build_ats_graph(self, graph_path: str = "/ATSActionGraph"):
        """
        - ROS2Context
        - Spot 제어:
            * Twist 구독(vel), JointState 명령 구독
            * JointState 퍼블리시
        - Clock 퍼블리시
        - IMU 퍼블리시(시뮬 가속/각속/자세)
        - Odometry 퍼블리시(odom -> base_link)
        - TF 퍼블리시: 카메라/IMU(base_link 자식), odom 트리(odom -> base_link)
        """
        import omni.graph.core as og
        keys = og.Controller.Keys

        BODY_PRIM = Sdf.Path(self.assets["body_prim"])
        CAMERA_BASE_PRIM = Sdf.Path(self.assets["camera_base_prim"])
        SPOT_PRIM      = Sdf.Path(self.assets["spot_prim"])
        CAMERA_PRIM    = Sdf.Path(self.assets["camera_prim"])
        IMU_PRIM = Sdf.Path(self.assets["imu_prim"])
        ODOM_PRIM      = Sdf.Path("/World/odom")   # 프레임 "odom"과 연결
        ATS_PRIM = Sdf.Path("/World/Spot/ATS/ATS")

        (graph, _, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    # 초기 1회 트리거
                    ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                    # 매 프레임 트리거
                    ("Tick",           "omni.graph.action.OnPlaybackTick"),

                    # ROS2 컨텍스트
                    ("Context",        "isaacsim.ros2.bridge.ROS2Context"),

                    # Spot 제어/상태
                    ("SubscribeJointState",     "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController",  "isaacsim.core.nodes.IsaacArticulationController"),
                    ("SubscribeTwist",          "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("SpotPublishJointState",   "isaacsim.ros2.bridge.ROS2PublishJointState"),

                    # ATS 제어
                    # build_ats_graph() 안에서 keys.CREATE_NODES 리스트에 추가
                    ("SubscribeATSTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),

                    # Clock
                    ("ClockTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ClockPub",  "isaacsim.ros2.bridge.ROS2PublishClock"),

                    # IMU
                    ("OnTickIMU",       "omni.graph.action.OnTick"),
                    ("ImuComputeOdom",  "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("ImuReadSimTime",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ImuPublish",      "isaacsim.ros2.bridge.ROS2PublishImu"),

                    # Odom
                    ("OdomCompute", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("OdomPublish", "isaacsim.ros2.bridge.ROS2PublishOdometry"),

                    # TF (cam/imu)
                    ("TFReadTime_camimu", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("TFPubCam",          "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("TFPubImu",          "isaacsim.ros2.bridge.ROS2PublishTransformTree"),

                    # TF (odom)
                    ("TFReadTime_odom", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("TFPubOdom",       "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.CONNECT: [
                    # 초기화 계열(한 번)
                    ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "SpotPublishJointState.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "SubscribeTwist.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "ClockPub.inputs:execIn"),

                    # 컨텍스트 연결
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "SpotPublishJointState.inputs:context"),
                    ("Context.outputs:context", "ClockPub.inputs:context"),

                    # 조인트명 전달
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),

                    # 시계
                    ("ClockTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),

                    # IMU(매 프레임)
                    ("OnTickIMU.outputs:tick", "ImuComputeOdom.inputs:execIn"),
                    ("OnTickIMU.outputs:tick", "ImuPublish.inputs:execIn"),
                    ("Context.outputs:context", "ImuPublish.inputs:context"),
                    ("ImuComputeOdom.outputs:orientation",        "ImuPublish.inputs:orientation"),
                    ("ImuComputeOdom.outputs:angularVelocity",    "ImuPublish.inputs:angularVelocity"),
                    ("ImuComputeOdom.outputs:linearAcceleration", "ImuPublish.inputs:linearAcceleration"),
                    ("ImuReadSimTime.outputs:simulationTime",     "ImuPublish.inputs:timeStamp"),

                    # Odom(매 프레임)
                    ("Tick.outputs:tick", "OdomCompute.inputs:execIn"),
                    ("Tick.outputs:tick", "OdomPublish.inputs:execIn"),
                    ("Context.outputs:context", "OdomPublish.inputs:context"),
                    ("OdomCompute.outputs:position",        "OdomPublish.inputs:position"),
                    ("OdomCompute.outputs:orientation",     "OdomPublish.inputs:orientation"),
                    ("OdomCompute.outputs:linearVelocity",  "OdomPublish.inputs:linearVelocity"),
                    ("OdomCompute.outputs:angularVelocity", "OdomPublish.inputs:angularVelocity"),
                    ("ImuReadSimTime.outputs:simulationTime", "OdomPublish.inputs:timeStamp"),

                    # TF(cam/imu/odom)
                    ("Tick.outputs:tick", "TFPubCam.inputs:execIn"),
                    ("Tick.outputs:tick", "TFPubImu.inputs:execIn"),
                    ("Tick.outputs:tick", "TFPubOdom.inputs:execIn"),
                    ("Context.outputs:context", "TFPubCam.inputs:context"),
                    ("Context.outputs:context", "TFPubImu.inputs:context"),
                    ("Context.outputs:context", "TFPubOdom.inputs:context"),
                    ("TFReadTime_camimu.outputs:simulationTime", "TFPubCam.inputs:timeStamp"),
                    ("TFReadTime_camimu.outputs:simulationTime", "TFPubImu.inputs:timeStamp"),
                    ("TFReadTime_odom.outputs:simulationTime",   "TFPubOdom.inputs:timeStamp"),

                    # ATS 
                    # keys.CONNECT 리스트에 추가 (초기 1회 실행 + 컨텍스트 연결)
                    ("OnImpulseEvent.outputs:execOut", "SubscribeATSTwist.inputs:execIn"),
                    ("Context.outputs:context",        "SubscribeATSTwist.inputs:context"),
                    

                ],
                keys.SET_VALUES: [
                    # 제어 타깃(Spot)
                    ("ArticulationController.inputs:robotPath",   str(SPOT_PRIM)),
                    ("ArticulationController.inputs:targetPrim",  str(SPOT_PRIM)),

                    ("SubscribeJointState.inputs:topicName", self.ros["topics"]["joint_cmd"]),
                    ("SubscribeTwist.inputs:topicName",     self.ros["topics"]["cmd_twist"]),
                    ("SpotPublishJointState.inputs:topicName", self.ros["topics"]["spot_joint_state_pub"]),
                    ("SpotPublishJointState.inputs:targetPrim",   str(SPOT_PRIM)),

                    # IMU
                    ("ImuComputeOdom.inputs:chassisPrim", str(SPOT_PRIM)),
                    ("ImuPublish.inputs:frameId",   self.ros["frames"]["imu_frame"]),
                    ("ImuPublish.inputs:topicName", self.ros["topics"]["imu"]),
                    ("ImuPublish.inputs:publishOrientation",       True),
                    ("ImuPublish.inputs:publishAngularVelocity",   True),
                    ("ImuPublish.inputs:publishLinearAcceleration", True),
                    ("OnTickIMU.inputs:framePeriod", 0),
                    ("OnTickIMU.inputs:onlyPlayback", True),

                    # Odom (odom -> base_link)
                    ("OdomCompute.inputs:chassisPrim", str(SPOT_PRIM)),
                    ("OdomPublish.inputs:topicName",    self.ros["topics"]["odom"]),
                    ("OdomPublish.inputs:odomFrameId",  self.ros["frames"]["odom"]),
                    ("OdomPublish.inputs:chassisFrameId", "body"),

                    # TF - Cam / IMU (부모: base_link)
                    #("TFPubCam.inputs:parentPrim",  str(CAMERA_BASE_PRIM)),
                    #("TFPubCam.inputs:targetPrims", [str(CAMERA_PRIM)]),
                    # 1) parent를 body로 변경
                    ("TFPubCam.inputs:parentPrim",  str(BODY_PRIM)),

                    # 2) target을 camera_base, camera (+원하면 ats의 base_link/link1/link2)까지 확장
                    ("TFPubCam.inputs:targetPrims", [
                        str(CAMERA_BASE_PRIM),                           # /World/Spot/ATS/ATS/link2/Xform
                        str(CAMERA_PRIM),                                # /World/Spot/ATS/ATS/link2/Xform/Camera
                        str(ATS_PRIM.AppendPath("base_link")),           # (옵션) /World/Spot/ATS/ATS/base_link
                        str(ATS_PRIM.AppendPath("link1")),               # (옵션) /World/Spot/ATS/ATS/link1
                        str(ATS_PRIM.AppendPath("link2")),               # (옵션) /World/Spot/ATS/ATS/link2
                    ]),
                                        
                    ("TFPubImu.inputs:parentPrim",  str(BODY_PRIM)),
                    ("TFPubImu.inputs:targetPrims", [str(IMU_PRIM)]),

                    # TF - Odom 트리 (부모: /World/odom, 자식: base_link)
                    ("TFPubOdom.inputs:parentPrim",  str(ODOM_PRIM)),
                    ("TFPubOdom.inputs:targetPrims", [str(BODY_PRIM)]),

                    # ATS 용
                    # keys.SET_VALUES 리스트에 추가 (토픽명만 맞추면 됨)
                    ("SubscribeATSTwist.inputs:topicName", "/ats_twist"),        


                ],
            },
        )
        og.Controller.evaluate_sync(graph)

    # --------------------------------------------------------------------- #
    # RTX LiDAR (2D/3D) ROS 퍼블리셔 그래프
    # --------------------------------------------------------------------- #
  
    def build_lidar_ros_graph(self, full_cfg: dict, graph_path: str = "/LidarGraph"):
        """
        Isaac Sim 5.0 기준
        - 2D: ROS2RtxLidarHelper(type="laser_scan")  -> topics["scan"]
        - 3D: ROS2RtxLidarHelper(type="point_cloud")-> topics["point_cloud"]
        - 각 LiDAR prim을 RenderProduct.cameraPrim으로 직접 사용
        - TF(부모=base_link, 자식=각 LiDAR prim)
        """
        import omni.graph.core as og
        from pxr import UsdGeom, Gf, Sdf

        keys = og.Controller.Keys

        lidar   = full_cfg["sensors"]["lidar"]         # e.g., {"create_2d": True, "prim_2d": "/World/Spot/Lidar2D", ...}
        frames  = full_cfg["ros"]["frames"]            # e.g., {"base_scan": "base_scan"}
        topics  = full_cfg["ros"]["topics"]            # e.g., {"scan": "/scan", "point_cloud": "/point_cloud"}
        domain  = full_cfg.get("ros", {}).get("domain_id", None)

        create_nodes = [
            ("Tick",    "omni.graph.action.OnPlaybackTick"),                 # ①
            ("RunSim",  "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            #("RunSim",  "isaacsim.core.nodes.IsaacRunOneSimulationFrame"),   # ②
            ("Ctx",     "isaacsim.ros2.bridge.ROS2Context"),
        ]
        connect, setvals = [], []

        # --- 공통: Tick → RunSim ---
        connect += [
            ("Tick.outputs:tick", "RunSim.inputs:execIn"),
        ]

        # --- ROS2 Context 설정 ---
        # Domain ID가 있으면 넣어줌. 없으면 기본값(환경변수 사용 여부는 False로 가정)
        setvals += [
            ("Ctx.inputs:useDomainIDEnvVar", False),
        ]
        if domain is not None:
            setvals += [("Ctx.inputs:domainId", int(domain))]

        # ---------------- 2D LaserScan ----------------
        if lidar.get("create_2d", True):
            create_nodes += [
                ("RP_2D",      "isaacsim.core.nodes.IsaacCreateRenderProduct"),  # ③
                ("TFTime2D",   "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TFLidar2D",  "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("Lidar2D",    "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),       # ④
            ]

            # 흐름: RunSim.step → RP_2D.execIn → RP_2D.renderProductPath → Lidar2D.renderProductPath
            #       RunSim.step → Lidar2D.execIn (매 프레임 발행)
            connect += [
                ("RunSim.outputs:step",                 "RP_2D.inputs:execIn"),
                ("RP_2D.outputs:renderProductPath",     "Lidar2D.inputs:renderProductPath"),
                #("RunSim.outputs:step",                 "Lidar2D.inputs:execIn"),
                ("RP_2D.outputs:execOut",                 "Lidar2D.inputs:execIn"),
                ("Ctx.outputs:context",                 "Lidar2D.inputs:context"),

                # TF 갱신
                ("Tick.outputs:tick",                   "TFLidar2D.inputs:execIn"),
                ("Ctx.outputs:context",                 "TFLidar2D.inputs:context"),
                ("TFTime2D.outputs:simulationTime",     "TFLidar2D.inputs:timeStamp"),
            ]

            # 기본 RP(해상도는 필요시 조정)
            setvals += [
                ("RP_2D.inputs:enabled",    True),
                ("RP_2D.inputs:cameraPrim", Sdf.Path(lidar["prim_2d"])),
                ("RP_2D.inputs:width",      640),
                ("RP_2D.inputs:height",     480),

                # Lidar Helper 설정
                ("Lidar2D.inputs:enabled",        True),
                ("Lidar2D.inputs:type",           "laser_scan"),
                ("Lidar2D.inputs:topicName",      topics["scan"]),
                ("Lidar2D.inputs:frameId",        frames["base_scan"]),
                ("Lidar2D.inputs:frameSkipCount", 0),
                ("Lidar2D.inputs:nodeNamespace", ""),  
                # ("Lidar2D.inputs:publishFullScan", True),  # 회전형 LiDAR에서 한 바퀴 완성 스캔만 발행하려면 사용

                # TF: 부모/자식
                ("TFLidar2D.inputs:parentPrim",   str(self.assets["body_prim"])),
                ("TFLidar2D.inputs:targetPrims",  [str(lidar["prim_2d"])]),
            ]

        # ---------------- 3D PointCloud2 ----------------
        if lidar.get("create_3d", False):
            create_nodes += [
                ("RP_3D",      "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("TFTime3D",   "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TFLidar3D",  "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("Lidar3D",    "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ]

            connect += [
                ("RunSim.outputs:step",                 "RP_3D.inputs:execIn"),
                ("RP_3D.outputs:renderProductPath",     "Lidar3D.inputs:renderProductPath"),
                ("RP_3D.outputs:execOut",               "Lidar3D.inputs:execIn"),
                ("Ctx.outputs:context",                 "Lidar3D.inputs:context"),

                ("Tick.outputs:tick",                   "TFLidar3D.inputs:execIn"),
                ("Ctx.outputs:context",                 "TFLidar3D.inputs:context"),
                ("TFTime3D.outputs:simulationTime",     "TFLidar3D.inputs:timeStamp"),
            ]

            setvals += [
                ("RP_3D.inputs:enabled",    True),
                ("RP_3D.inputs:cameraPrim", Sdf.Path(lidar["prim_3d"])),
                ("RP_3D.inputs:width",      640),
                ("RP_3D.inputs:height",     480),

                ("Lidar3D.inputs:enabled",        True),
                ("Lidar3D.inputs:type",           "point_cloud"),
                ("Lidar3D.inputs:topicName",      topics["point_cloud"]),
                ("Lidar3D.inputs:frameId",        frames["base_scan_3D"]),
                ("Lidar3D.inputs:frameSkipCount", 0),

                ("TFLidar3D.inputs:parentPrim",   str(self.assets["body_prim"])),
                ("TFLidar3D.inputs:targetPrims",  [str(lidar["prim_3d"])]),
            ]

        # 그래프 생성/적용
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {keys.CREATE_NODES: create_nodes, keys.CONNECT: connect, keys.SET_VALUES: setvals},
        )
        # 즉시 계산(그래프 활성화)
        og.Controller.evaluate_sync(graph_path)


