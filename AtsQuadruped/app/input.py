from pynput import keyboard

class TeleopInput:
    def __init__(self):
        self.pressed = set()
        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.daemon = True
        self._listener.start()

    def _on_press(self, key):
        try:
            if key.char in ['a', 's', 'd', 'w']:
                self.pressed.add(key.char)
        except AttributeError:
            if key in [keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right]:
                self.pressed.add(key.name)

    def _on_release(self, key):
        try:
            if key.char in ['a', 's', 'd', 'w']:
                self.pressed.discard(key.char)
        except AttributeError:
            if key.name in ["up", "down", "left", "right"]:
                self.pressed.discard(key.name)