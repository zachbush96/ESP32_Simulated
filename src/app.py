from src.core.config_loader import load_config
from src.simulation.world import build_simulation_from_config


def main() -> None:
    config = load_config()
    mode = config.get("settings", {}).get("mode", "simulation")
    if mode == "simulation":
        world = build_simulation_from_config()
        world.run()
    else:
        # Placeholder: hook real robot loop here when hardware is available
        world = build_simulation_from_config()
        world.run()


if __name__ == "__main__":
    main()
