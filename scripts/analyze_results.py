#!/usr/bin/env python3

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, stdev

ROOT = Path(__file__).resolve().parents[1]
INPUT_FILE = ROOT / "results" / "planning_results_12cel.csv"
RESULTS_DIR = ROOT / "results"

NUMERIC_COLUMNS = [
    "utvonal_hossz_m",
    "futasido_ms",
    "utvonal_pontok_szama",
    "fa_csomopontok_szama",
]

METRIC_LABELS = {
    "utvonal_hossz_m": "Útvonalhossz (m)",
    "futasido_ms": "Futásidő (ms)",
    "utvonal_pontok_szama": "Útvonalpontok száma",
    "fa_csomopontok_szama": "Fa csomópontok száma",
}


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def format_value(value, digits=3):
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


def calculate_stats(values):
    values = [value for value in values if math.isfinite(value)]

    if not values:
        return {
            "atlag": math.nan,
            "median": math.nan,
            "szoras": math.nan,
            "minimum": math.nan,
            "maximum": math.nan,
        }

    return {
        "atlag": mean(values),
        "median": median(values),
        "szoras": stdev(values) if len(values) > 1 else 0.0,
        "minimum": min(values),
        "maximum": max(values),
    }


def load_results():
    if not INPUT_FILE.exists():
        sys.exit(f"Hiba: nem található a bemeneti fájl: {INPUT_FILE}")

    results = defaultdict(list)

    with INPUT_FILE.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)

        required_columns = {
            "algoritmus",
            "sikeres",
            *NUMERIC_COLUMNS,
        }

        missing_columns = required_columns - set(reader.fieldnames or [])

        if missing_columns:
            sys.exit(
                "Hiba: hiányzó CSV-oszlop(ok): "
                + ", ".join(sorted(missing_columns))
            )

        for row in reader:
            algorithm = row["algoritmus"].strip()

            if not algorithm:
                continue

            for column in NUMERIC_COLUMNS:
                row[column] = to_float(row[column])

            results[algorithm].append(row)

    return results


def write_summary_csv(results):
    output_file = RESULTS_DIR / "summary_statistics.csv"

    fieldnames = [
        "algoritmus",
        "meresek_szama",
        "sikeres_meresek_szama",
        "sikeressegi_arany_szazalek",
        "mutato",
        "atlag",
        "median",
        "szoras",
        "minimum",
        "maximum",
    ]

    with output_file.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        for algorithm, rows in sorted(results.items()):
            successful_rows = [
                row for row in rows
                if row["sikeres"].strip().lower() == "igen"
            ]

            success_rate = (
                100.0 * len(successful_rows) / len(rows)
                if rows else 0.0
            )

            for column in NUMERIC_COLUMNS:
                values = [row[column] for row in successful_rows]
                stat = calculate_stats(values)

                writer.writerow({
                    "algoritmus": algorithm,
                    "meresek_szama": len(rows),
                    "sikeres_meresek_szama": len(successful_rows),
                    "sikeressegi_arany_szazalek": format_value(success_rate, 1),
                    "mutato": METRIC_LABELS[column],
                    "atlag": format_value(stat["atlag"]),
                    "median": format_value(stat["median"]),
                    "szoras": format_value(stat["szoras"]),
                    "minimum": format_value(stat["minimum"]),
                    "maximum": format_value(stat["maximum"]),
                })

    return output_file


def write_markdown_report(results):
    output_file = RESULTS_DIR / "evaluation_report.md"
    algorithms = sorted(results.keys())

    with output_file.open("w", encoding="utf-8") as file:
        file.write("# A* és RRT mérési kiértékelés\n\n")
        file.write(
            "A kiértékelés a `planning_results_12cel.csv` "
            "fájl mérési adataiból készült.\n\n"
        )

        file.write("| Mutató | " + " | ".join(algorithms) + " |\n")
        file.write("|---|" + "|".join("---:" for _ in algorithms) + "|\n")

        file.write(
            "| Mérések száma | "
            + " | ".join(str(len(results[algorithm])) for algorithm in algorithms)
            + " |\n"
        )

        success_rates = []

        for algorithm in algorithms:
            rows = results[algorithm]
            successful = sum(
                row["sikeres"].strip().lower() == "igen"
                for row in rows
            )
            rate = 100.0 * successful / len(rows) if rows else 0.0
            success_rates.append(f"{rate:.1f}%")

        file.write("| Sikerességi arány | " + " | ".join(success_rates) + " |\n")

        for column in NUMERIC_COLUMNS:
            values = []

            for algorithm in algorithms:
                successful_rows = [
                    row for row in results[algorithm]
                    if row["sikeres"].strip().lower() == "igen"
                ]
                average = calculate_stats(
                    [row[column] for row in successful_rows]
                )["atlag"]

                values.append(format_value(average))

            file.write(
                f"| {METRIC_LABELS[column]} | "
                + " | ".join(values)
                + " |\n"
            )

        file.write(
            "\nA futásidő ezredmásodpercben, az útvonalhossz méterben értendő. "
            "Az A* fa-csomópontszáma definíció szerint 0; az RRT értéke "
            "a generált keresőfa méretét jelzi.\n"
        )

    return output_file


def create_plots(results):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "A diagramok nem készültek el, mert hiányzik a matplotlib.\n"
            "Telepítés: sudo apt install python3-matplotlib"
        )
        return []

    algorithms = sorted(results.keys())

    plot_definitions = [
        ("utvonal_hossz_m", "Útvonalhossz összehasonlítása", "m",
         "path_length_comparison.png"),
        ("futasido_ms", "Futásidő összehasonlítása", "ms",
         "runtime_comparison.png"),
        ("utvonal_pontok_szama", "Útvonalpontok számának összehasonlítása", "db",
         "path_points_comparison.png"),
        ("fa_csomopontok_szama", "RRT-fa csomópontszáma", "db",
         "rrt_tree_size.png"),
    ]

    created_files = []

    for column, title, unit, filename in plot_definitions:
        data = []

        for algorithm in algorithms:
            successful_rows = [
                row for row in results[algorithm]
                if row["sikeres"].strip().lower() == "igen"
            ]
            data.append([row[column] for row in successful_rows])

        figure, axis = plt.subplots(figsize=(7, 4.5))
        axis.boxplot(data, labels=algorithms, showmeans=True)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.3)

        figure.tight_layout()

        output_file = RESULTS_DIR / filename
        figure.savefig(output_file, dpi=180)
        plt.close(figure)

        created_files.append(output_file)

    return created_files


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    results = load_results()

    print("Beolvasott mérések:")

    for algorithm, rows in sorted(results.items()):
        successful = sum(
            row["sikeres"].strip().lower() == "igen"
            for row in rows
        )
        print(f"  {algorithm}: {len(rows)} mérés, {successful} sikeres")

    summary_file = write_summary_csv(results)
    report_file = write_markdown_report(results)
    plot_files = create_plots(results)

    print(f"\nLétrehozva: {summary_file}")
    print(f"Létrehozva: {report_file}")

    for plot_file in plot_files:
        print(f"Létrehozva: {plot_file}")


if __name__ == "__main__":
    main()
