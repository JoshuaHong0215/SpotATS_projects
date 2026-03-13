import torch

class PolicyRunner:
    def __init__(self, path: str, device: str = "auto"):
        if device == "auto":
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.model = torch.jit.load(path, map_location=self.device).eval()

    def infer(self, obs_np):
        with torch.no_grad():
            # 일반적인 파이썬박스 즉 Numpy배열을 Pytorch는 읽지 못한다.
            # 그래서 AI전용 포장지인 Tensor로 데이터를 감싼다.
            t = torch.tensor(obs_np, dtype=torch.float32, device=self.device).unsqueeze(0)
            out = self.model(t).squeeze(0).cpu().numpy()
        return out