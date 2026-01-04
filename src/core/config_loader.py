import os
from pathlib import Path
from typing import Any, Dict

import yaml


CONFIG_DIR = Path(__file__).resolve().parent.parent.parent / "config"


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merged[key] = merge_dict(base[key], value)
        else:
            merged[key] = value
    return merged


def load_config() -> Dict[str, Any]:
    settings = load_yaml(CONFIG_DIR / "settings.yaml")
    simulation = load_yaml(CONFIG_DIR / "simulation.yaml")
    models = load_yaml(CONFIG_DIR / "models.yaml")

    config = {
        "settings": settings,
        "simulation": simulation,
        "models": models,
    }

    env_mode = os.getenv("ROBOT_MODE")
    if env_mode:
        config["settings"]["mode"] = env_mode

    return config
