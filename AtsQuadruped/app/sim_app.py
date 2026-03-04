from isaacsim import SimulationApp
from typing import List, Dict, Any

class SimApp:
    def __init__(self, headless: bool, extensions: List[str]):
        self._app = SimulationApp({"headless": headless, "load_extensions": extensions})

    @property
    def raw(self):
        return self._app

    def update(self):
        self._app.update()

    def is_running(self) -> bool:
        return self._app.is_running()

    def close(self):
        self._app.close()