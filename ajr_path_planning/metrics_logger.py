import csv
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple


RESULTS_FILE = (
    Path(__file__).resolve().parent.parent
    / "results"
    / "planning_results.csv"
)


def write_result(
    algorithm: str,
    start: Tuple[float, float],
    goal: Tuple[float, float],
    success: bool,
    path_length_m: Optional[float],
    elapsed_time_ms: float,
    point_count: int,
    tree_node_count: int,
) -> None:
    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)

    file_exists = RESULTS_FILE.exists()

    with RESULTS_FILE.open(
        "a",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.writer(csv_file)

        if not file_exists:
            writer.writerow([
                "ido",
                "algoritmus",
                "start_x_m",
                "start_y_m",
                "cel_x_m",
                "cel_y_m",
                "sikeres",
                "utvonal_hossz_m",
                "futasido_ms",
                "utvonal_pontok_szama",
                "fa_csomopontok_szama",
            ])

        writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            algorithm,
            f"{start[0]:.3f}",
            f"{start[1]:.3f}",
            f"{goal[0]:.3f}",
            f"{goal[1]:.3f}",
            "igen" if success else "nem",
            "" if path_length_m is None else f"{path_length_m:.3f}",
            f"{elapsed_time_ms:.3f}",
            point_count,
            tree_node_count,
        ])