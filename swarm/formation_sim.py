"""Four-drone virtual-structure simulation for umbrella coverage."""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Dict, Optional, Tuple


@dataclass
class DronePoint:
    """A drone target position in the virtual umbrella structure."""

    x: float
    y: float
    z: float


TargetPoint = Tuple[float, float, float]
FormationPoints = Dict[str, DronePoint]


class FormationSimulator:
    """Virtual-structure simulator for four-drone umbrella coverage."""

    def __init__(self, target: TargetPoint = (0.0, 0.0, 0.0), d: float = 1.2, h: float = 1.5) -> None:
        self.target = target
        self.d = d
        self.h = h
        self.points: FormationPoints = {}

    def compute_umbrella_formation(
        self,
        target: Optional[TargetPoint] = None,
        d: Optional[float] = None,
        h: Optional[float] = None,
    ) -> FormationPoints:
        """Compute four drone positions using the virtual-structure method."""
        if target is not None:
            self.target = target
        if d is not None:
            self.d = d
        if h is not None:
            self.h = h

        x, y, z = self.target
        d_value = self.d
        h_value = self.h
        flight_z = z + h_value
        self.points = {
            "drone_1": DronePoint(x - d_value, y + d_value, flight_z),
            "drone_2": DronePoint(x + d_value, y + d_value, flight_z),
            "drone_3": DronePoint(x - d_value, y - d_value, flight_z),
            "drone_4": DronePoint(x + d_value, y - d_value, flight_z),
        }
        return self.points

    def save_2d_plot(self, output_path: Path) -> Path:
        """Save a 2D matplotlib visualization of the four-drone umbrella rectangle."""
        if not self.points:
            self.compute_umbrella_formation()

        output_path.parent.mkdir(parents=True, exist_ok=True)
        cache_dir = output_path.parent / ".matplotlib_cache"
        cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", str(cache_dir))

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        target_x, target_y, _target_z = self.target
        p1 = self.points["drone_1"]
        p2 = self.points["drone_2"]
        p3 = self.points["drone_3"]
        p4 = self.points["drone_4"]

        figure, axis = plt.subplots(figsize=(7, 7))
        rectangle_x = [p1.x, p2.x, p4.x, p3.x, p1.x]
        rectangle_y = [p1.y, p2.y, p4.y, p3.y, p1.y]
        axis.plot(rectangle_x, rectangle_y, "b-", linewidth=2, label="umbrella rectangle")

        drone_x = [point.x for point in self.points.values()]
        drone_y = [point.y for point in self.points.values()]
        axis.scatter(drone_x, drone_y, c="tab:blue", s=90, label="drones")
        for name, point in self.points.items():
            axis.text(point.x + 0.04, point.y + 0.04, name, fontsize=10)

        axis.scatter([target_x], [target_y], c="tab:red", s=120, marker="o", label="pedestrian target")
        axis.scatter([target_x], [target_y], c="tab:green", s=70, marker="x", label="umbrella center")
        axis.text(target_x + 0.04, target_y - 0.12, "target / center", fontsize=10)

        axis.set_title("DroneUmbrella Four-Drone Virtual Structure")
        axis.set_xlabel("x / m")
        axis.set_ylabel("y / m")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linestyle="--", alpha=0.4)
        axis.legend(loc="upper right")
        margin = self.d + 0.8
        axis.set_xlim(target_x - margin, target_x + margin)
        axis.set_ylim(target_y - margin, target_y + margin)

        figure.tight_layout()
        figure.savefig(output_path, dpi=160)
        plt.close(figure)
        return output_path
