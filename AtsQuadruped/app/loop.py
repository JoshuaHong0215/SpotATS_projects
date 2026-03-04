class SimLoop:
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