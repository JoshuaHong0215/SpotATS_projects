# app/utils.py
import os
from pathlib import Path
import yaml


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _resolve_from_root(path_str: str) -> str:
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    root = _project_root()
    return str((root / p).resolve())


def weighted_small_yaw(
    vz: float,
    *,
    small_abs_max: float = 0.2,     # |vz|가 이 값 이하일 때만 가중
    weight: float = 3.0,             # 그 구간에서만 곱할 가중치
    clip_abs: float | None = 1.5     # 안전 클립(원치 않으면 None)
) -> float:
    """
    Nav2 angular.z(vz)가 너무 작아서 로봇이 회전을 시작 못 하는 구간에서만
    '조건부 증폭'을 적용한다.

    규칙:
      - |vz| <= small_abs_max  => vz *= weight
      - |vz| >  small_abs_max  => vz 그대로
      - clip_abs가 있으면 최종적으로 [-clip_abs, +clip_abs]로 제한
    """
    try:
        v = float(vz)
    except Exception:
        return 0.0

    if abs(v) <= float(small_abs_max):
        v = v * float(weight)

    if clip_abs is not None:
        m = float(clip_abs)
        if v > m:
            v = m
        elif v < -m:
            v = -m

    return float(v)


def load_cfg(path: str | None = None):
    if path is None:
        path = os.environ.get("ATS_CONFIG")
    if path is None:
        path = _project_root() / "configs" / "default.yaml"
    else:
        path = Path(path)

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    try:
        cfg["assets"]["usd_path"] = _resolve_from_root(cfg["assets"]["usd_path"])
    except KeyError:
        pass

    try:
        cfg["policy"]["path"] = _resolve_from_root(cfg["policy"]["path"])
    except KeyError:
        pass

    return cfg