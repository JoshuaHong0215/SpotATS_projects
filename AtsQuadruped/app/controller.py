import numpy as np
import omni.graph.core as og
from pathlib import Path
from datetime import datetime

class RobotController:
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
        self.spot = spot_view
        self.ats  = ats_view
        self.spot_scale = float(spot_action_scale)
        self.ats_step   = float(ats_joint_step)
        self.default_pos = self.spot.get_joint_positions()[0].copy()
        
        # 디버그/로그
        self.DEBUG = bool(debug)
        self._log_every_n = int(log_every_n)
        self._apply_count = 0
        if dump_dir is None:
            dump_dir = Path(__file__).resolve().parents[1] / "logs"
        self.dump_dir = Path(dump_dir)
        self.dump_dir.mkdir(parents=True, exist_ok=True)

        # Spot/ATS DOF
        try:
            self._spot_dof = int(len(self.spot.get_joint_positions()[0]))
        except Exception:
            self._spot_dof = 0
        try:
            self._ats_dof = int(len(self.ats.get_joint_positions()[0])) if self.ats else 0
        except Exception:
            self._ats_dof = 0

        # ── 제어 경로 auto 선택 ──
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
    # 그래프 입력
    # ─────────────────────────────────────────────────────────────────────
    @staticmethod
    def read_twist_from_graph():
        lin_attr = og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:linearVelocity")
        ang_attr = og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:angularVelocity")
        if not lin_attr or not lin_attr.is_valid() or not ang_attr or not ang_attr.is_valid():
            return 0.0, 0.0, 0.0
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
    # 메인 제어
    # ─────────────────────────────────────────────────────────────────────
    def apply_actions(self, policy_action, ats_cmd):
        # ── Spot 다리 12축 제어 ──
        q_spot = self.spot.get_joint_positions()[0].copy()
        pa = np.asarray(policy_action, dtype=np.float32).ravel()
        if len(self._leg_idx) > 0:
            if pa.size < len(self._leg_idx):
                pa = np.pad(pa, (0, len(self._leg_idx) - pa.size), mode="constant")
            pa_leg = pa[:len(self._leg_idx)]
            q_spot[self._leg_idx] = self.default_pos[self._leg_idx] + pa_leg * self.spot_scale
        self.spot.set_joint_position_targets(q_spot)

        # ── ATS 제어 (via 선택) ──
        cmd = np.asarray(ats_cmd, dtype=np.float32).ravel()

        if self._control_ats_via == "ats" and self.ats and self._ats_dof > 0:
            q_ats = self.ats.get_joint_positions()[0].copy()
            need = 2 if len(self._ats_idx_ats) >= 2 else len(self._ats_idx_ats)
            if cmd.size < need:
                cmd = np.pad(cmd, (0, need - cmd.size), mode="constant")
            for i, idx in enumerate(self._ats_idx_ats[:need]):
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