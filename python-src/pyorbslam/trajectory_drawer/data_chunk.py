from typing import Literal, Any

from dataclasses import dataclass

@dataclass
class DataChunk:
    name: str
    vtype: Literal['line', 'image'] # visual type
    data: Any

