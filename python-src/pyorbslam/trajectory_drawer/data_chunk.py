from typing import Literal, Any

from dataclasses import dataclass

@dataclass
class DataChunk:
    name: str
    vtype: Literal['line', 'image', 'mesh'] # visual type
    data: Any

