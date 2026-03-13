# main.py (Isaac Sim 5.0) — CRASH CATCHER VERSION

from isaacsim import SimulationApp
from app.utils import load_cfg, weighted_small_yaw
import numpy as np
import traceback


def main():
    cfg = load_cfg()

    cmd_shape_debug = bool(cfg.get("controls", {}).get("cmd_shape_debug", False))

    # cfg로 조절 가능하게 (없으면 기본값)
    yaw_cfg = cfg.get("controls", {}).get("yaw_weight_small", {})
    small_abs_max = float(yaw_cfg.get("small_abs_max", 0.2))   # |vz|<=0.2 구간만
    small_weight  = float(yaw_cfg.get("weight", 3.0))           # 그 구간에서만 곱
    yaw_clip_abs  = yaw_cfg.get("clip_abs", 1.5)                # None 가능
    if yaw_clip_abs is not None:
        yaw_clip_abs = float(yaw_clip_abs)

    # 최종 합성 yaw도 안전 클립 (과도하게 커지는 것 방지)
    yaw_final_clip = float(cfg.get("controls", {}).get("yaw_final_clip", 1.5))

    print("[BOOT] Launching SimulationApp (Safe Mode)...")

    ext_list = cfg["sim"]["load_extensions"]
    safe_list = [e for e in ext_list if "omni.anim" not in e]

    sim = SimulationApp({
        "headless": cfg["sim"]["headless"],
        "load_extensions": safe_list,
    })
    sim.update()

    print("[BOOT] SimulationApp started")

    try:
        from app.world import SimWorld
        from app.graph_builder import GraphBuilder
        from app.policy import PolicyRunner
        from app.observation import ObservationBuilder
        from app.controller import RobotController
        from app.input import TeleopInput
        from app.pedestrian import PedestrianManager

        import omni.timeline

    except Exception:
        print("[FATAL] Lazy import failed:\n" + traceback.format_exc())
        sim.close()
        return

    try:
        world = SimWorld(
            usd_path=cfg["assets"]["usd_path"],
            spot_prim=cfg["assets"]["spot_prim"],
            ats_prim=cfg["assets"]["ats_prim"],
            imu_dummy_prim=cfg["assets"]["imu_prim"],
            fixed_time_step=cfg["player"]["fixed_time_step"],
            play_every_frame=cfg["player"]["play_every_frame"],
            target_hz=cfg["player"]["target_hz"],
        )
        print("[OK] Stage/World initialized")
    except Exception:
        print("[FATAL] SimWorld init failed:\n" + traceback.format_exc())
        sim.close()
        return

    gb = GraphBuilder(cfg["assets"], cfg["ros"])
    try:
        gb.build_camera_ros_graph()
        gb.build_ats_graph()
        try:
            gb.build_lidar_ros_graph(cfg, graph_path="/LidarGraph")
        except Exception as e:
            print(f"[WRN] Lidar graph build failed (Non-fatal): {e}")

        sim.update()
        print("[OK] Graphs built")
    except Exception:
        print("[FATAL] Graph build failed:\n" + traceback.format_exc())
        sim.close()
        return

    try:
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        sim.update()
        print("[OK] Timeline playing")
    except Exception:
        print("[FATAL] Timeline play failed:\n" + traceback.format_exc())
        sim.close()
        return

    try:
        policy = PolicyRunner(cfg["policy"]["path"], cfg["policy"]["device"])
        default_pos = world.spot.get_joint_positions().squeeze(0)
        obsb = ObservationBuilder(world.spot, default_pos)

        ctrl = RobotController(
            world.spot,
            world.ats,
            cfg["controls"]["spot_action_scale"],
            cfg["controls"]["ats_joint_step"],
        )
        teleop = TeleopInput()

        pedestrian = PedestrianManager(
            agent_name="Forklift_01",
            waypoints=[(-4.0, 6.0, 0.0), (-4.0, -4.0, 0.0),
                       (-7.0, -4.0, 0.0), (-7.0, 6.0, 0.0)]
        )
        print("[OK] Controllers ready")
    except Exception:
        print("[FATAL] Control stack init failed:\n" + traceback.format_exc())
        sim.close()
        return

    print("[BOOT] Entering Main Loop...")
    try:
        while sim.is_running():
            world.step(render=True)

            if timeline.is_playing():
                pedestrian.update(dt=0.0166)

            # (A) Teleop
            physical_pressed = set(teleop.pressed)

            # (B) ATS gimbal용 ROS input
            try:
                yaw_ros, pitch_ros = RobotController.read_ats_twist_from_graph()
            except Exception:
                yaw_ros, pitch_ros = 0.0, 0.0

            # (C) Mapping (gimbal 키)
            virtual_pressed = set()
            DEAD = 0.15
            if yaw_ros > DEAD:
                virtual_pressed.add('a')
            elif yaw_ros < -DEAD:
                virtual_pressed.add('d')
            if pitch_ros > DEAD:
                virtual_pressed.add('s')
            elif pitch_ros < -DEAD:
                virtual_pressed.add('w')

            merged_pressed = physical_pressed | virtual_pressed

            # (D) Teleop vector (여기 teleop_vec[2]에 키보드 yaw 있음)
            teleop_vec, ats_cmd_from_keys = ctrl.teleop_from_keys(
                merged_pressed,
                cfg["controls"]["teleop"]["lin_speed"],
                cfg["controls"]["teleop"]["ang_speed_yaw"],
                cfg["controls"]["teleop"]["ang_speed_pitch"],
            )

            # (E) Nav2 twist (body)
            vx_r, vy_r, vz_r = RobotController.read_twist_from_graph()

            #  Nav2 yaw만 "작은 구간에서만" 조건부 증폭
            vz_nav2_adj = weighted_small_yaw(
                vz_r,
                small_abs_max=small_abs_max,
                weight=small_weight,
                clip_abs=yaw_clip_abs,
            )

            #  키보드 yaw는 항상 살아있게 + Nav2 yaw를 더함
            yaw_cmd = float(teleop_vec[2] + vz_nav2_adj)

            # 최종 안전 클립
            if yaw_cmd > yaw_final_clip:
                yaw_cmd = yaw_final_clip
            elif yaw_cmd < -yaw_final_clip:
                yaw_cmd = -yaw_final_clip

            if cmd_shape_debug and (abs(vz_r) > 1e-4 or abs(teleop_vec[2]) > 1e-4):
                print(
                    f"[YAW] teleop={teleop_vec[2]:+.3f}, nav2_raw={vz_r:+.3f} -> nav2_adj={vz_nav2_adj:+.3f} -> final={yaw_cmd:+.3f} "
                    f"(small_abs_max={small_abs_max}, w={small_weight}, clip_abs={yaw_clip_abs}, final_clip={yaw_final_clip})"
                )

            cmd_vec = np.array(
                [teleop_vec[0] + vx_r, teleop_vec[1] + vy_r, yaw_cmd],
                dtype=np.float32
            )

            obs = obsb.build(cmd_vec)
            action = policy.infer(obs)
            ctrl.apply_actions(action, ats_cmd_from_keys)
            obsb.update_prev_action(action)

            RobotController.trigger_graph_impulse()

    except KeyboardInterrupt:
        print("[INFO] User stopped simulation.")
    except Exception as e:
        print("\n" + "="*50)
        print("!!! CRASH DETECTED IN MAIN LOOP !!!")
        print("Error Message:", e)
        print("Traceback:")
        traceback.print_exc()
        print("="*50 + "\n")
    finally:
        print("[INFO] Closing SimulationApp...")
        sim.close()


if __name__ == "__main__":
    main()