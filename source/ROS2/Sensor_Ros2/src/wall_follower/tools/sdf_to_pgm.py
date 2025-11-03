#!/usr/bin/env python3
"""
Convert a Gazebo SDF world made of box-shaped obstacles into a 2D occupancy grid
map (PGM + YAML) compatible with ROS2 nav2/map_server.

Usage example:
    python3 sdf_to_pgm.py \
        --world /home/ubuntu/map.world \
        --output-dir /home/ubuntu/Intel_study/Intel7_Final_Project_5_Team/RoS2/map_gazebo \
        --map-name cleaned_map \
        --resolution 0.05 \
        --margin 0.5
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple
import xml.etree.ElementTree as ET

import imageio.v2 as imageio
import numpy as np
import yaml


def _parse_pose(element: ET.Element | None) -> Tuple[float, float, float]:
    if element is None or element.text is None:
        return 0.0, 0.0, 0.0
    values = [float(v) for v in element.text.strip().split()]
    while len(values) < 6:
        values.append(0.0)
    x, y, _, roll, pitch, yaw = values
    if abs(roll) > 1e-5 or abs(pitch) > 1e-5:
        raise ValueError(
            'Only planar (roll=0,pitch=0) poses are supported; '
            f'got roll={roll}, pitch={pitch}.'
        )
    return x, y, yaw


def _compose_pose(parent: Tuple[float, float, float],
                  child: Tuple[float, float, float]) -> Tuple[float, float, float]:
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    x = px + cos_yaw * cx - sin_yaw * cy
    y = py + sin_yaw * cx + cos_yaw * cy
    yaw = pyaw + cyaw
    return x, y, yaw


def _extract_boxes(world_path: Path) -> List[Tuple[float, float, float, float, float]]:
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        raise ValueError(f'Could not find <world> element in {world_path}')

    boxes: List[Tuple[float, float, float, float, float]] = []

    for model in world.findall('model'):
        name = model.get('name', '')
        if name in {'ground_plane', 'sun'}:
            continue
        model_pose = _parse_pose(model.find('pose'))

        for link in model.findall('link'):
            link_pose = _parse_pose(link.find('pose'))
            model_link_pose = _compose_pose(model_pose, link_pose)

            for collision in link.findall('collision'):
                collision_pose = _parse_pose(collision.find('pose'))
                pose_total = _compose_pose(model_link_pose, collision_pose)
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                box = geometry.find('box')
                if box is None:
                    continue
                size_elem = box.find('size')
                if size_elem is None or size_elem.text is None:
                    continue
                sx, sy, sz = (float(v) for v in size_elem.text.strip().split())
                boxes.append((pose_total[0], pose_total[1], pose_total[2], sx, sy))

    if not boxes:
        raise ValueError(
            f'No box geometries found in {world_path}. '
            'Ensure the world contains <collision><geometry><box> elements.'
        )

    return boxes


def _box_corners(box: Tuple[float, float, float, float, float]) -> Iterable[Tuple[float, float]]:
    cx, cy, yaw, sx, sy = box
    hx = sx / 2.0
    hy = sy / 2.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            dx = sign_x * hx
            dy = sign_y * hy
            x = cx + cos_yaw * dx - sin_yaw * dy
            y = cy + sin_yaw * dx + cos_yaw * dy
            yield x, y


def _compute_bounds(boxes: Sequence[Tuple[float, float, float, float, float]],
                    margin: float) -> Tuple[float, float, float, float]:
    xs: List[float] = []
    ys: List[float] = []
    for box in boxes:
        for x, y in _box_corners(box):
            xs.append(x)
            ys.append(y)
    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin
    return min_x, max_x, min_y, max_y


def _generate_grid(boxes: Sequence[Tuple[float, float, float, float, float]],
                   bounds: Tuple[float, float, float, float],
                   resolution: float,
                   occupied_value: int,
                   free_value: int) -> np.ndarray:
    min_x, max_x, min_y, max_y = bounds
    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)
    grid = np.full((height, width), free_value, dtype=np.uint8)

    for row in range(height):
        y = max_y - (row + 0.5) * resolution
        for col in range(width):
            x = min_x + (col + 0.5) * resolution
            if _point_inside_any_box(x, y, boxes):
                grid[row, col] = occupied_value
    return grid


def _point_inside_any_box(x: float, y: float,
                          boxes: Sequence[Tuple[float, float, float, float, float]]) -> bool:
    for cx, cy, yaw, sx, sy in boxes:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx = x - cx
        dy = y - cy
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        if abs(local_x) <= sx / 2.0 and abs(local_y) <= sy / 2.0:
            return True
    return False


def _write_pgm(path: Path, grid: np.ndarray) -> None:
    imageio.imwrite(path, grid)


def _write_yaml(path: Path, pgm_filename: str, resolution: float,
                origin: Tuple[float, float, float]) -> None:
    data = {
        'image': pgm_filename,
        'resolution': float(resolution),
        'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    path.write_text(yaml.dump(data, default_flow_style=False, sort_keys=False))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Convert an SDF world (with box obstacles) into a ROS occupancy grid.')
    parser.add_argument('--world', required=True, type=Path,
                        help='Path to the input SDF world file.')
    parser.add_argument('--output-dir', required=True, type=Path,
                        help='Directory to store the generated map.yaml and map.pgm.')
    parser.add_argument('--map-name', default='generated_map',
                        help='Base filename for output map (default: generated_map).')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Resolution (meters per pixel) for the occupancy grid.')
    parser.add_argument('--margin', type=float, default=0.4,
                        help='Extra free-space margin (meters) added around occupied cells.')
    parser.add_argument('--occupied-value', type=int, default=0,
                        help='Pixel value for occupied cells (default: 0 = black).')
    parser.add_argument('--free-value', type=int, default=254,
                        help='Pixel value for free cells (default: 254 ~ white).')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    world_path = args.world.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    boxes = _extract_boxes(world_path)
    bounds = _compute_bounds(boxes, margin=args.margin)
    grid = _generate_grid(
        boxes=boxes,
        bounds=bounds,
        resolution=args.resolution,
        occupied_value=args.occupied_value,
        free_value=args.free_value,
    )

    pgm_path = output_dir / f'{args.map_name}.pgm'
    yaml_path = output_dir / f'{args.map_name}.yaml'
    _write_pgm(pgm_path, grid)

    origin = (bounds[0], bounds[2], 0.0)  # min_x, min_y, yaw=0
    _write_yaml(yaml_path, pgm_path.name, args.resolution, origin)

    print(f'Generated occupancy grid: {pgm_path}')
    print(f'Generated map metadata:   {yaml_path}')
    print(f'Resolution: {args.resolution} m/px, size: {grid.shape[1]} x {grid.shape[0]} px')
    print(f'Origin (map frame): x={origin[0]:.3f}, y={origin[1]:.3f}')


if __name__ == '__main__':
    main()
