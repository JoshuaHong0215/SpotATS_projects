from pynput import keyboard

class TeleopInput:
    def __init__(self):
        '''
        self.pressed = set()은 현재 누르고 있는 키들의 이름을 담아둘 바구니를 뜻함
        여기서 중점은 리스트가 아닌 set() 즉, 집합을 사용한 것이 핵심임
        키보드로 꾹 누르고 있으면 신호가 연속으로 들어오는 set을 쓰면 중복은 알아서 걸러주어 
        바구니에 w한개만 깔끔하게 유지됨
        '''
        self.pressed = set()
        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.daemon = True
        self._listener.start()


    
    # 분기 처리 담당
    '''
        a,s,d,w가 아닌 방향키를 눌렀을 때도 같은 명령을 수행하기 위한 장치를 마련함
        방향키로 인해 AttributerError가 났다면 그 키의 고유이름(name)을 
        self.pressed에 add하여 같은 명령을 수행함

        추가적으로 NumberPad를 사용하여 ats 조작 기능 추가 할 예정
    '''

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