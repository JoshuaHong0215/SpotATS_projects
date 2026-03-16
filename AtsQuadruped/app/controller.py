import numpy as np
import omni.graph.core as og
from pathlib import Path
from datetime import datetime

# 뇌(policy.py)가 내릴 명령과 조종기(telopkey)의 명령을 받아서 가상환경 속 로봇의 관절각도를 조작하는 모터 컨트롤러같은 코드

class RobotController:
    # 로봇을 조종하기 전, 로봇의 상태와 부품 구성을 스캔하여 제어계획을 세우는 부분
    def __init__(self, spot_view, ats_view, spot_action_scale: float, ats_joint_step: float,
                 debug: bool = True, log_every_n: int = 120, dump_dir: str | None = None,
                 fallback_assume_last_two_ats: bool = True,
                 force_ats_via: str | None = None,     # "ats" | "spot" | None(auto)
                 force_ats_joint_names: list[str] | None = None,  # 예: ["joint1","joint2"]
                 ):
        """
        현재 스테이지 구조 가정(기본):
          - Spot articulation: 다리 12 DOF (fl_hx..hr_kn)
          - ATS articulation: 별도 2 DOF (joint1, joint2)  ← (실제 스테이지에 따라 다를 수 있음)
        """

        # ==============================================================================#
        # 물리엔진에 생성된 로봇 본체 즉, spot , ats의 제어 권한을 변수에 묶어둠
        self.spot = spot_view
        self.ats  = ats_view

        # AI의 명령이나 조종기의 입력값의 증폭량을 결정하는 상수
        '''
        인공지능은 보통 -1.0에서 1.0사이의 아주 작은 숫자만 뱉도록 훈련된다(정규화 및 스케일링 작업을 통한 기울기 폭발 방지)
        하지만 실제 로봇관절을 30도, 45도등 크게 움직이려면 작은 숫자를 현실적인 모터 회전값으로 키워줘야한다
        즉, AI가 뱉은 0.5라는 숫자에 스케일값 30(예시)을 곱해서 15도로 만들어주는 수학적 곱하기 과정이 증폭량을 뜻함

        기울기 폭발: AI내부에서 수백만번의 곱하는 과정이 있는데 숫자가 조금만 커져도 컴퓨터가 처리가 힘들정도로 커져서 터지는걸 뜻함
        정규화 및 스케일링: 제멋대로인 데이터를 -1~1 사이의 딱떨어지는 규격으로 꾹꾹 눌러담는 작업을 뜻함
        '''
        

        # Isaac sim simulation play시 초기의 가장 안정적인 로봇의 자세의 관절각도를 읽어와서 기본자세로 백업한다, 다리를 움직일때 기준점이 됨
        self.default_pos = self.spot.get_joint_positions()[0].copy()
        # ==============================================================================#
        
        # 디버그/로그
        self.DEBUG = bool(debug)
        self._log_every_n = int(log_every_n)
        self._apply_count = 0
        if dump_dir is None:
            dump_dir = Path(__file__).resolve().parents[1] / "logs"
        self.dump_dir = Path(dump_dir)
        self.dump_dir.mkdir(parents=True, exist_ok=True)

        # ==============================================================================#

        # Spot/ATS DOF
        # spot 다리 모터의 갯수, 상단 장비 모터가 몇개인지(DOF)를 세어 저장한다
        '''
        DOF는 Degrees of Freedom즉 자유도라고 하고 스스로 움직일 수 있는 모터를 말함
        그래서 Spot다리는 12 자유도라고하면 spot의 다리에는 독립적으로 움직일 수 있는 모터가 12개가 있다라는 말임
        '''

        try:
            self._spot_dof = int(len(self.spot.get_joint_positions()[0]))
        except Exception:
            self._spot_dof = 0
        try:
            self._ats_dof = int(len(self.ats.get_joint_positions()[0])) if self.ats else 0
        except Exception:
            self._ats_dof = 0

        # ==============================================================================#

        # ── 제어 경로 auto 선택 ──
        # 상단 장비 즉, ATS를 돌려야 하는데 이 장비가 Spot파일 안에 같이 묶여있는지("spot")아니면 별개의 파일("ats")로 존재하는지 파악
        if force_ats_via in ("ats", "spot"):
            self._control_ats_via = force_ats_via
        else:
            self._control_ats_via = "ats" if (self.ats and self._ats_dof > 0) else "spot"

        # 인덱스 초기값
        self._ats_idx_spot: list[int] = []
        self._ats_idx_ats:  list[int] = [0, 1] if self._ats_dof >= 2 else ([0] if self._ats_dof == 1 else [])

        # Spot 다리 인덱스(앞 12개)
        self._leg_idx = list(range(min(12, self._spot_dof)))

        # ── 이름/경로 로그 & 이름기반 매핑 시도 ──
        spot_names = []
        ats_names  = []
        spot_paths = getattr(self.spot, "prim_paths", None)
        ats_paths  = getattr(self.ats,  "prim_paths", None)

        try:
            spot_names = getattr(self.spot, "get_dof_names", lambda: [])()
        except Exception:
            pass
        try:
            ats_names = getattr(self.ats, "get_dof_names", lambda: [])() if self.ats else []
        except Exception:
            pass

        if self.DEBUG:
            print(f"[ATS DEBUG] control_via = {self._control_ats_via}")
            print(f"[ATS DEBUG] spot_dof={self._spot_dof}, ats_dof={self._ats_dof}")
            print(f"[ATS DEBUG] spot_prim_paths={spot_paths}")
            print(f"[ATS DEBUG] ats_prim_paths={ats_paths}")
            print(f"[ATS DEBUG] spot_dof_names(count={len(spot_names)}): {spot_names}")
            print(f"[ATS DEBUG] ats_dof_names(count={len(ats_names)}): {ats_names}")

        # 이름 기반 강제 맵핑이 주어졌으면 우선 사용
        if force_ats_joint_names and len(spot_names) > 0:
            want = [n for n in force_ats_joint_names if isinstance(n, str)]
            try:
                self._ats_idx_spot = [spot_names.index(n) for n in want]
                self._control_ats_via = "spot"
                if self.DEBUG:
                    print(f"[ATS DEBUG] forced name mapping via SPOT: {want} -> idx {self._ats_idx_spot}")
            except ValueError as e:
                if self.DEBUG:
                    print(f"[ATS DEBUG] forced name mapping failed: {e}")

        # 자동 휴리스틱: spot의 dof 이름들 중 'ats'/'tilt'/'pan'/'joint1' 등의 키워드 탐색
        if self._control_ats_via == "spot" and not self._ats_idx_spot and len(spot_names) > 0:
            keys = ["ats", "gimbal", "joint1", "joint2", "tilt", "pan", "link2"]
            cand = [i for i, n in enumerate(spot_names) if any(k in n.lower() for k in keys)]
            if len(cand) >= 2:
                self._ats_idx_spot = cand[:2]
                if self.DEBUG:
                    print(f"[ATS DEBUG] heuristic mapping via SPOT: idx={self._ats_idx_spot} names={[spot_names[i] for i in self._ats_idx_spot]}")
            elif self.DEBUG:
                print("[ATS DEBUG] heuristic mapping failed (need manual names)")

    # ─────────────────────────────────────────────────────────────────────
    # 그래프 입력 / 명령 수신부
    # ─────────────────────────────────────────────────────────────────────
    @staticmethod
    def read_twist_from_graph():
        # omnigraph에서 들어오는 linear velocity 선을 붙잡는다, 
        lin_attr = og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:linearVelocity")
        ang_attr = og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:angularVelocity")
        
        # ROS통신망이 끊기거나, 조이스틱같은 컨트롤러의 케이블이 뽑혔을때를 대비한 안전장치
        if not lin_attr or not lin_attr.is_valid() or not ang_attr or not ang_attr.is_valid():
            return 0.0, 0.0, 0.0
        
        # lin_attr에서 연결한 선을 타고 들어오는 실제 숫자 데이터 (x, y, z 속도)를 뽑아낸다
        lin = og.Controller.get(lin_attr)
        ang = og.Controller.get(ang_attr)
        if lin is None or ang is None or len(lin) < 3 or len(ang) < 3:
            return 0.0, 0.0, 0.0
        vx, vy, _ = lin
        _, _, vz = ang
        return float(vx), float(vy), float(vz)

    @staticmethod
    def teleop_from_keys(pressed, lin_speed, yaw_speed, pitch_speed):
        vx =  lin_speed if 'up'    in pressed else (-lin_speed if 'down'  in pressed else 0.0)
        vz =  lin_speed if 'left'  in pressed else (-lin_speed if 'right' in pressed else 0.0)
        qz =  yaw_speed   if 'a' in pressed else (-yaw_speed   if 'd' in pressed else 0.0)
        qy = -pitch_speed if 'w' in pressed else ( pitch_speed if 's' in pressed else 0.0)
        return np.array([vx, 0.0, vz], dtype=np.float32), np.array([qz, qy], dtype=np.float32)

    @staticmethod
    # ATS조작부 / 추후 teleop_from_keys에 반영하여 ats도 조작할 예정
    def read_ats_twist_from_graph():
        ang_attr = og.Controller.attribute("/ATSActionGraph/SubscribeATSTwist.outputs:angularVelocity")
        if not ang_attr or not ang_attr.is_valid():
            return 0.0, 0.0
        ang = og.Controller.get(ang_attr)
        if ang is None:
            return 0.0, 0.0
        try:
            if len(ang) < 3:
                return 0.0, 0.0
        except Exception:
            return 0.0, 0.0
        # yaw=z, pitch=y
        return float(ang[2]), float(ang[1])

    # ─────────────────────────────────────────────────────────────────────
    # 메인 제어 / AI뇌(policy_action)와 조종기(ats_cmd)의 명령을 받아 실제 물리엔진의 관절을 조작하는 controller.py의 심장
    # ─────────────────────────────────────────────────────────────────────
    def apply_actions(self, policy_action, ats_cmd):
        # ── Spot 다리 12축 제어 ──
        # spot의 12개의 관절각도를 읽어옴
        q_spot = self.spot.get_joint_positions()[0].copy()

        # AI가 보낸 다리제어 명령을 일렬로 정렬함
        '''
        ravel 명령으로 일렬로 정렬시키는 이유
        pytorch AI model은 결과를 뱉을 떄 2차원 행렬형태로 값을 출력함 
        즉, [1번각도, 2번각도] 이게 아닌 [[1번각도, 2번각도]]형태로 나옴 그걸 제거하는게 일렬로 정렬하는 것을 뜻함
        '''
        pa = np.asarray(policy_action, dtype=np.float32).ravel()
        if len(self._leg_idx) > 0:
            if pa.size < len(self._leg_idx):
                pa = np.pad(pa, (0, len(self._leg_idx) - pa.size), mode="constant")
            pa_leg = pa[:len(self._leg_idx)]

            # 위에서 저장한 기본자세를 기준으로 AI명령에 스케일을 곱한 값을 더해서 최종적으로 꺾어야 할 목표 각도를 계산
            '''
            AI 절대각도 45도로 꺾어라 라고 명령하지 않음
            기본 자세 즉, 위에서 선언한 self.defaul_pos를 기준점으로
            거기서 +5도만큼 꺾어라 라고 차이값 즉, Delta값으로 명령함

            그래서 아래와 같은 공식을 설정함
            목표 각도 = (기본 자세각도) + (AI가 지시하나 차이값 * 증폭량)
            '''
            q_spot[self._leg_idx] = self.default_pos[self._leg_idx] + pa_leg * self.spot_scale

        # 계산된 각도 q_spot을 물리엔진에 넣는다. 이 줄이 실행되는 순간 가상환경의 로봇다리가 움직임    
        self.spot.set_joint_position_targets(q_spot)

        # ==============================================================================#

        # ── ATS 제어 (via 선택) ──
        # 조종기에서 온 회전 명령을 일렬로 정렬시킴
        cmd = np.asarray(ats_cmd, dtype=np.float32).ravel()

        if self._control_ats_via == "ats" and self.ats and self._ats_dof > 0:
            q_ats = self.ats.get_joint_positions()[0].copy()
            need = 2 if len(self._ats_idx_ats) >= 2 else len(self._ats_idx_ats)
            if cmd.size < need:
                cmd = np.pad(cmd, (0, need - cmd.size), mode="constant")
            for i, idx in enumerate(self._ats_idx_ats[:need]):
                # 다리와 다르게 기본자세로 돌아가는게 아닌, 현재 각도(q_ats)엥서 조종기의 명령(cmd)만큼 누적(+=)해서 각도를 꺾는다
                q_ats[idx] += self.ats_step * cmd[i]
            self.ats.set_joint_position_targets(q_ats)

            if (self._apply_count % self._log_every_n == 0) and self.DEBUG:
                try:
                    get_targets = getattr(self.ats, "get_joint_position_targets", None)
                    tgt = get_targets()[0] if get_targets else None
                    cur = self.ats.get_joint_positions()[0]
                    names = getattr(self.ats, "get_dof_names", lambda: [])()
                    print("[ATS DEBUG] apply(path=ats) idx", self._ats_idx_ats,
                          "cmd", cmd[:len(self._ats_idx_ats)].tolist(),
                          "step", self.ats_step)
                    if tgt is not None:
                        print("[ATS DEBUG] targets[:8] =", np.array(tgt)[:min(8, len(tgt))])
                    print("[ATS DEBUG] current[:8] =", np.array(cur)[:min(8, len(cur))])
                    print("[ATS DEBUG] names[:8]   =", names[:min(8, len(names))])
                except Exception as e:
                    print("[ATS DEBUG] probe failed:", e)

        elif self._control_ats_via == "spot" and len(self._ats_idx_spot) > 0:
            q = self.spot.get_joint_positions()[0].copy()
            need = min(2, len(self._ats_idx_spot))
            if cmd.size < need:
                cmd = np.pad(cmd, (0, need - cmd.size), mode="constant")
            for i, idx in enumerate(self._ats_idx_spot[:need]):
                q[idx] += self.ats_step * cmd[i] 
            self.spot.set_joint_position_targets(q)
            if (self._apply_count % self._log_every_n == 0) and self.DEBUG:
                names = getattr(self.spot, "get_dof_names", lambda: [])()
                print("[ATS DEBUG] apply(path=spot) idx", self._ats_idx_spot[:need],
                      "cmd", cmd[:need].tolist(),
                      "step", self.ats_step,
                      "names", [names[i] for i in self._ats_idx_spot[:need]] if names else [])

        else:
            if (self._apply_count % self._log_every_n == 0) and self.DEBUG:
                print(f"[ATS DEBUG] apply skipped: via={self._control_ats_via}, "
                      f"ats_dof={self._ats_dof}, idx_ats={self._ats_idx_ats}, idx_spot={self._ats_idx_spot}, cmd={ats_cmd}")

        self._apply_count += 1

    @staticmethod
    def trigger_graph_impulse():
        og.Controller.set(og.Controller.attribute("/ATSActionGraph/OnImpulseEvent.state:enableImpulse"), True)