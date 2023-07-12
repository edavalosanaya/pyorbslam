from typing import Literal, Any

from dataclasses import dataclass

@dataclass
class DataChunk:
    name: str
    vtype: Literal['line', 'image', 'mesh', 'point cloud'] # visual type
    data: Any

