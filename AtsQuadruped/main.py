import numpy as np
from isaacsim import SimulationApp

# 1. 시뮬레이션 앱 초기화
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.viewports import set_camera_view
import omni.appwindow
import carb.input
from my_spot import SpotFlatTerrainPolicy 

class SpotControllerMain:
    def __init__(self):
        # 2. 월드 생성
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()
        
        # 3. 로봇 초기화
        self.robot_policy = SpotFlatTerrainPolicy(
            prim_path="/World/SpotATS",
            name="SpotATS",
            position=np.array([0, 0, 0.6])
        )
        
        self.command = np.zeros(3)
        self.speed_scale = 1.0
        self.base_speed = 0.5
        
        # 4. 키보드 입력 설정 (에러 방지를 위해 가장 단순한 경로 사용)
        self._app_window = omni.appwindow.get_default_app_window()
        self._input_interface = carb.input.acquire_input_interface()
        
        # 키보드 장치를 직접 획득하는 대신 윈도우에서 바로 가져옴
        self._sub_keyboard = self._input_interface.subscribe_to_keyboard_events(
            self._app_window.get_keyboard(), 
            self._on_keyboard_event
        )

    def _on_keyboard_event(self, event, *args, **kwargs):
        ki = carb.input.KeyboardInput
        et = carb.input.KeyboardEventType

        if event.type == et.KEY_PRESS or event.type == et.KEY_REPEAT:
            if event.input == ki.UP:    self.command[0] = self.base_speed * self.speed_scale
            elif event.input == ki.DOWN:  self.command[0] = -self.base_speed * self.speed_scale
            elif event.input == ki.LEFT:  self.command[2] = self.base_speed * self.speed_scale
            elif event.input == ki.RIGHT: self.command[2] = -self.base_speed * self.speed_scale
            elif event.input == ki.Q:     self.command[1] = self.base_speed * self.speed_scale
            elif event.input == ki.E:     self.command[1] = -self.base_speed * self.speed_scale
            elif event.input == ki.RIGHT_BRACKET:
                self.speed_scale += 0.1
                print(f"Speed Scale: {self.speed_scale:.1f}")
            elif event.input == ki.LEFT_BRACKET:
                self.speed_scale = max(0.1, self.speed_scale - 0.1)
                print(f"Speed Scale: {self.speed_scale:.1f}")

        elif event.type == et.KEY_RELEASE:
            if event.input in [ki.UP, ki.DOWN]: self.command[0] = 0
            if event.input in [ki.LEFT, ki.RIGHT]: self.command[2] = 0
            if event.input in [ki.Q, ki.E]: self.command[1] = 0
        return True

    def run(self):
        self._world.reset()
        set_camera_view(eye=[2.5, 2.5, 2.5], target=[0, 0, 0])
        
        print("\n[START] 로봇 조종을 시작합니다.")
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_playing():
                self.robot_policy.forward(dt=1/60.0, command=self.command)

        simulation_app.close()

if __name__ == "__main__":
    main = SpotControllerMain()
    main.run()