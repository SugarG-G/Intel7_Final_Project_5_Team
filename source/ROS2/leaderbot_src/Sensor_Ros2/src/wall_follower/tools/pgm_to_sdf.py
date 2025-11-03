#!/usr/bin/env python3
"""
Utility script to convert a 2D occupancy grid map (map.yaml + map.pgm) into a
Gazebo-compatible SDF world file composed of static box obstacles.

Example:
    python3 pgm_to_sdf.py --map /home/ubuntu/map.yaml --output /tmp/map.world
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import yaml


@dataclass
class MapMeta:
    resolution: float
    origin: Tuple[float, float, float]
    negate: bool
    occupied_thresh: float
    free_thresh: float
    image_path: Path
    width: int
    height: int
    max_value: int


@dataclass
class CellRect:
    x_start: int
    x_end: int
    row_start: int
    row_end: int

    @property
    def width_cells(self) -> int:
        return self.x_end - self.x_start + 1

    @property
    def height_cells(self) -> int:
        return self.row_end - self.row_start + 1


def _load_map_metadata(yaml_path: Path) -> Dict:
    data = yaml.safe_load(yaml_path.read_text())
    if 'image' not in data:
        raise ValueError(f'"image" key is missing in {yaml_path}')
    return data


def _load_pgm_image(pgm_path: Path) -> Tuple[np.ndarray, int]:
    with pgm_path.open('rb') as f:
        header = f.readline().strip()
        if header != b'P5':
            raise ValueError(f'Unsupported PGM format {header!r} (expected binary P5)')

        def _read_non_comment_line() -> bytes:
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line

        dims_line = _read_non_comment_line()
        width_str, height_str = dims_line.strip().split()
        width = int(width_str)
        height = int(height_str)

        maxval_line = _read_non_comment_line()
        max_value = int(maxval_line.strip())
        if max_value <= 0 or max_value > 255:
            raise ValueError(f'Unsupported max value {max_value} in {pgm_path}')

        img_data = f.read(width * height)
        if len(img_data) != width * height:
            raise ValueError(f'Unexpected image size in {pgm_path}: '
                             f'expected {width * height} bytes, got {len(img_data)}')
        image = np.frombuffer(img_data, dtype=np.uint8).reshape((height, width))
        return image, max_value


def _compute_occupancy(image: np.ndarray, max_value: int, meta: MapMeta) -> np.ndarray:
    normalized = image.astype(np.float32) / float(max_value)
    if meta.negate:
        occupancy = normalized
    else:
        occupancy = 1.0 - normalized
    occupied = occupancy >= meta.occupied_thresh
    return occupied


def _inflate_grid(grid: np.ndarray, iterations: int) -> np.ndarray:
    if iterations <= 0:
        return grid
    inflated = grid.copy()
    height, width = inflated.shape
    for _ in range(iterations):
        expanded = inflated.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                src_y = slice(max(0, -dy), min(height, height - dy))
                dst_y = slice(max(0, dy), min(height, height + dy))
                src_x = slice(max(0, -dx), min(width, width - dx))
                dst_x = slice(max(0, dx), min(width, width + dx))
                expanded[dst_y, dst_x] |= inflated[src_y, src_x]
        inflated = expanded
    return inflated


def _extract_rectangles(grid: np.ndarray) -> List[CellRect]:
    height, width = grid.shape
    rectangles: List[CellRect] = []
    active: Dict[Tuple[int, int], Tuple[int, int]] = {}

    for row in range(height):
        segments: List[Tuple[int, int]] = []
        col = 0
        while col < width:
            if grid[row, col]:
                start = col
                while col + 1 < width and grid[row, col + 1]:
                    col += 1
                end = col
                segments.append((start, end))
            col += 1

        next_active: Dict[Tuple[int, int], Tuple[int, int]] = {}
        for seg in segments:
            if seg in active:
                next_active[seg] = (active[seg][0], row)
            else:
                next_active[seg] = (row, row)

        for seg, (row_start, row_end) in active.items():
            if seg not in next_active:
                rectangles.append(CellRect(seg[0], seg[1], row_start, row_end))

        active = next_active

    for seg, (row_start, row_end) in active.items():
        rectangles.append(CellRect(seg[0], seg[1], row_start, row_end))

    return rectangles


def _rectangle_to_sdf(rect: CellRect, meta: MapMeta, wall_height: float,
                      min_cell_span_x: int, min_cell_span_y: int,
                      thickness_padding: float, color_rgba: Sequence[float]) -> str | None:
    if rect.width_cells < min_cell_span_x or rect.height_cells < min_cell_span_y:
        return None

    resolution = meta.resolution
    origin_x, origin_y, origin_yaw = meta.origin
    map_height = meta.height

    size_x = rect.width_cells * resolution
    size_y = rect.height_cells * resolution
    size_y = max(size_y, thickness_padding)

    center_x = origin_x + ((rect.x_start + rect.x_end + 1) / 2.0) * resolution
    top_center = origin_y + (map_height - rect.row_start - 0.5) * resolution
    bottom_center = origin_y + (map_height - rect.row_end - 0.5) * resolution
    center_y = (top_center + bottom_center) / 2.0

    pose = f"{center_x:.6f} {center_y:.6f} {wall_height / 2.0:.6f} 0 0 {origin_yaw:.6f}"

    visual_rgba = " ".join(f"{c:.3f}" for c in color_rgba)

    return (
        "    <model name=\"wall_{name}\">\n"
        "      <static>true</static>\n"
        f"      <pose>{pose}</pose>\n"
        "      <link name=\"link\">\n"
        "        <collision name=\"collision\">\n"
        "          <geometry>\n"
        f"            <box><size>{size_x:.6f} {size_y:.6f} {wall_height:.6f}</size></box>\n"
        "          </geometry>\n"
        "        </collision>\n"
        "        <visual name=\"visual\">\n"
        "          <geometry>\n"
        f"            <box><size>{size_x:.6f} {size_y:.6f} {wall_height:.6f}</size></box>\n"
        "          </geometry>\n"
        "          <material>\n"
        f"            <ambient>{visual_rgba}</ambient>\n"
        f"            <diffuse>{visual_rgba}</diffuse>\n"
        "          </material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
    )


def _generate_sdf(rectangles: List[CellRect], meta: MapMeta, args) -> str:
    color_rgba = (*args.visual_color, args.visual_alpha)
    models = []
    for index, rect in enumerate(rectangles):
        sdf_block = _rectangle_to_sdf(
            rect=rect,
            meta=meta,
            wall_height=args.wall_height,
            min_cell_span_x=args.min_cells_x,
            min_cell_span_y=args.min_cells_y,
            thickness_padding=args.min_wall_thickness,
            color_rgba=color_rgba,
        )
        if sdf_block is not None:
            models.append(sdf_block.replace("wall_{name}", f"wall_{index:04d}"))

    models_str = "".join(models)
    if not models_str:
        raise RuntimeError('No wall segments detected; check thresholds or inflation settings.')

    return (
        '<?xml version="1.0" ?>\n'
        '<sdf version="1.7">\n'
        f'  <world name="{args.world_name}">\n'
        '    <gravity>0 0 -9.8</gravity>\n'
        '    <magnetic_field>0 0 0</magnetic_field>\n'
        '    <include>\n'
        '      <uri>model://ground_plane</uri>\n'
        '    </include>\n'
        '    <include>\n'
        '      <uri>model://sun</uri>\n'
        '    </include>\n'
        f'{models_str}'
        '  </world>\n'
        '</sdf>\n'
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Convert a ROS occupancy grid (map.yaml + map.pgm) into an SDF world.')
    parser.add_argument('--map', required=True, type=Path,
                        help='Path to the occupancy grid YAML (map.yaml).')
    parser.add_argument('--output', required=True, type=Path,
                        help='Destination path for the generated SDF world.')
    parser.add_argument('--world-name', default='converted_map',
                        help='Name to assign to the SDF world.')
    parser.add_argument('--wall-height', type=float, default=1.0,
                        help='Height of the generated wall boxes in meters (default: 1.0).')
    parser.add_argument('--inflate', type=int, default=0,
                        help='Number of dilation iterations to enlarge occupied cells (default: 0).')
    parser.add_argument('--min-cells-x', type=int, default=1,
                        help='Minimum contiguous occupied cells along X to keep a wall segment.')
    parser.add_argument('--min-cells-y', type=int, default=1,
                        help='Minimum contiguous occupied cells along Y to keep a wall segment.')
    parser.add_argument('--min-wall-thickness', type=float, default=0.05,
                        help='Minimum wall thickness (Y dimension) in meters (default: 0.05).')
    parser.add_argument('--visual-color', type=float, nargs=3, metavar=('R', 'G', 'B'),
                        default=(0.7, 0.2, 0.2),
                        help='RGB color for wall visuals (default: 0.7 0.2 0.2).')
    parser.add_argument('--visual-alpha', type=float, default=1.0,
                        help='Alpha transparency for wall visuals (default: 1.0).')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    yaml_path = args.map.expanduser().resolve()
    output_path = args.output.expanduser().resolve()

    yaml_data = _load_map_metadata(yaml_path)
    image_path = (yaml_path.parent / yaml_data['image']).resolve()

    image, max_value = _load_pgm_image(image_path)
    meta = MapMeta(
        resolution=float(yaml_data['resolution']),
        origin=tuple(float(v) for v in yaml_data.get('origin', [0.0, 0.0, 0.0])),
        negate=bool(yaml_data.get('negate', 0)),
        occupied_thresh=float(yaml_data.get('occupied_thresh', 0.65)),
        free_thresh=float(yaml_data.get('free_thresh', 0.25)),
        image_path=image_path,
        width=image.shape[1],
        height=image.shape[0],
        max_value=max_value,
    )

    occupancy = _compute_occupancy(image, max_value, meta)
    if args.inflate > 0:
        occupancy = _inflate_grid(occupancy, args.inflate)

    rectangles = _extract_rectangles(occupancy)

    sdf_content = _generate_sdf(rectangles, meta, args)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(sdf_content)

    total_cells = sum(rect.width_cells * rect.height_cells for rect in rectangles)
    print(f'Generated {len(rectangles)} wall segments '
          f'covering {total_cells} occupied cells.')
    print(f'SDF world written to: {output_path}')


if __name__ == '__main__':
    main()
