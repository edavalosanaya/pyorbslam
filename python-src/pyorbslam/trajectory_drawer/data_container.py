from typing import Union

from dataclasses import dataclass
import trimesh
import numpy as np

@dataclass
class MeshContainer:
    mesh: trimesh.Trimesh
    color: Union[tuple, np.ndarray] = (1,1,1,1)
    drawEdges: bool = False
    drawFaces: bool = True

