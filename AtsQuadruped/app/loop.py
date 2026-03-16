class SimLoop:

    '''
    [제어 흐름]

    1. 시간 갱신: 가상 환경의 물리 엔진 시간을 1틱(dt) 굴림
    - 핵심 코드: self.world.step(render=True)

    2. 명령 병합 (Input): 키보드 조작 신호(teleop_vec)와 외부 ROS 통신 속도(vx_r 등)를 읽어와 최종 목표 속도(cmd_vec)로 합침
    - 핵심 코드: 
        teleop_vec, ats_cmd = self.ctrl.teleop_cmd(self.input.pressed, self.speeds)
        vx_r, vy_r, vz_r = self.ctrl.read_twist_from_graph()
        cmd_vec = np.array([teleop_vec[0] + vx_r, teleop_vec[1] + vy_r, teleop_vec[2] + vz_r], dtype=np.float32)

    3. 관측 데이터 포장 (Observation): 합쳐진 목표 속도와 현재 센서 상태를 AI 입력용 데이터 상자로 포장함
    - 핵심 코드: obs = self.obsb.build(cmd_vec)

    4. AI 뇌 추론 (Policy): 포장된 데이터를 AI 뇌에 전달하여, 넘어지지 않기 위한 12개 다리 관절의 정밀한 목표 각도로 번역함
    - 핵심 코드: action = self.policy.infer(obs)

    5. 손발 제어 (Controller): 번역된 12개 다리 명령과 목(ATS) 명령을 물리 엔진에 하달하여 실제 보행 및 회전 수행
    - 핵심 코드: self.ctrl.apply_actions(action, ats_cmd)

    6. 과거 기억 백업 (Observation): 다음 루프 궤적 계산을 위해 방금 내린 다리 명령을 백업함(차이값 즉, Delta값)
    - 핵심 코드: self.obsb.update_prev_action(action)
    '''

    def __init__(self, sim_app, world, input_dev, policy, obs_builder, controller, speeds):
        self.app = sim_app
        self.world = world
        self.input = input_dev
        self.policy = policy
        self.obsb = obs_builder
        self.ctrl = controller
        self.speeds = speeds

    def run(self):
        import numpy as np
        while self.app.raw.is_running():
            self.world.step(render=True)
            teleop_vec, ats_cmd = self.ctrl.teleop_cmd(self.input.pressed, self.speeds)
            vx_r, vy_r, vz_r = self.ctrl.read_twist_from_graph()
            cmd_vec = np.array([teleop_vec[0] + vx_r, teleop_vec[1] + vy_r, teleop_vec[2] + vz_r], dtype=np.float32)

            obs = self.obsb.build(cmd_vec)
            action = self.policy.infer(obs)
            self.ctrl.apply_actions(action, ats_cmd)
            self.obsb.update_prev_action(action)
            self.ctrl.trigger_graph()





