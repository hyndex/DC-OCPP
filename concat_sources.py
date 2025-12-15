#!/usr/bin/env python3
"""Concatenate all files from ./src and ./include into a single output file."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List


def list_files(directories: Iterable[Path], exclude: Path) -> List[Path]:
    """Return a sorted list of files under the provided directories, skipping the output file."""
    collected: List[Path] = []
    for directory in directories:
        if not directory.exists():
            continue
        for path in directory.rglob("*"):
            if not path.is_file():
                continue
            if path.resolve() == exclude:
                continue
            collected.append(path)
    return sorted(collected)


def write_bundle(files: Iterable[Path], root: Path, destination: Path) -> None:
    """Write file contents to the destination with headers for each path."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as out_file:
        for path in files:
            relative = path.relative_to(root)
            out_file.write(f"=== {relative.as_posix()} ===\n")
            with path.open("r", encoding="utf-8", errors="replace") as source:
                out_file.write(source.read())
            out_file.write("\n\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Concatenate all files under ./src and ./include into a single file."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("Controller.txt"),
        help="Path to the output file (default: Controller.txt)",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Repository root containing src/ and include/ (default: script directory)",
    )

    args = parser.parse_args()
    root = args.root.resolve()
    output_path = args.output.resolve()

    src_dir = root / "src"
    include_dir = root / "include"
    files = list_files([src_dir, include_dir], exclude=output_path)
    write_bundle(files, root=root, destination=output_path)


if __name__ == "__main__":
    main()
