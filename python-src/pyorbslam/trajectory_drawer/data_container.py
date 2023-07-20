from typing import Union, Tuple

from dataclasses import dataclass
import trimesh
import numpy as np

@dataclass
class LineContainer:
    pos: np.ndarray
    color: Union[Tuple[float, float, float, float], np.ndarray] = (1.0, 1.0, 1.0, 1.0)
    width: int = 1

@dataclass
class MeshContainer:
    mesh: trimesh.Trimesh
    color: Union[Tuple[float, float, float,float], np.ndarray] = (1.0, 1.0, 1.0, 1.0)
    edgeColor: Union[Tuple[float, float, float, float], np.ndarray] = (1.0, 1.0, 1.0, 1.0)
    drawEdges: bool = False
    drawFaces: bool = True

@dataclass
class PointCloudContainer:
    pts: np.ndarray
    colors: Union[np.ndarray, Tuple[float, float, float, float]] = (1.0, 1.0, 1.0, 1.0)
    size: float = 5
