import logging
from typing import Dict, Any, Literal

import numpy as np
import pyqtgraph.opengl as gl
from PyQt5.QtGui import QColor
from dataclasses import asdict

logger = logging.getLogger('pyorbslam')

from ..data_chunk import DataChunk
from ..data_container import MeshContainer

class Display3D(gl.GLViewWidget):

    visuals: Dict[str, Any] # Dict[str, Item]

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        # Container
        self.visuals = {}

        # Map for creating empty elements
        self.item_create_map = {
            'line': self.create_line,
            'mesh': self.create_mesh
        }

        self.item_update_map = {
            'line': self.update_line,
            'mesh': self.update_mesh
        }
        
        # Changing the defaults
        # self.setBackgroundColor(QColor(255,255,255))
        
        # Create an AxisItem for the bottom axis
        bottom_axis = gl.GLAxisItem()
        self.addItem(bottom_axis)
        self.visuals['axis'] = bottom_axis

        # Adding a grid
        grid = gl.GLGridItem()
        grid.scale(2,2,1)
        self.addItem(grid)
       
        # Other configuration
        self.setCameraPosition(distance=1)

    def __str__(self):
        return "<Display3D>"

    def __repr__(self):
        return str(self)

    def create_visual(self, name: str, vtype: Literal['line', 'mesh']):
        
        # Create the item
        item = self.item_create_map[vtype]()
        self.addItem(item)

        # Then storing the data
        self.visuals[name] = item

        logger.debug(f"{self}: Created visual: {name}")

    def update_visual(self, data_chunk: DataChunk):
        # logger.debug(f"{self}::setData: {data_chunk.name} - {data_chunk.vtype}")
        update_fun = self.item_update_map[data_chunk.vtype]
        update_fun(self.visuals[data_chunk.name], data_chunk)

    def delete_visual(self, name: str):

        # Obtain and delete
        item = self.visuals[name]
        self.removeItem(item)
        
        logger.debug(f"{self}: Deleted visual: {name}")

    ####################################################################################
    ## Creating visual elements
    ####################################################################################

    def create_line(self):
        return gl.GLLinePlotItem(pos=np.empty((0,3)))

    def update_line(self, line, data_chunk: DataChunk):
        line.setData(pos=data_chunk.data)

    def create_mesh(self):
        return gl.GLMeshItem(smooth=False)

    def update_mesh(self, mesh, data_chunk: DataChunk):
        mesh_cont: MeshContainer = data_chunk.data

        # Define the vertices and faces of the mesh
        mesh_data = gl.MeshData(vertexes=mesh_cont.mesh.vertices, faces=mesh_cont.mesh.faces)
        mesh.setMeshData(
            meshdata=mesh_data,
            edgeColor=mesh_cont.color,
            drawFaces=mesh_cont.drawFaces, 
            drawEdges=mesh_cont.drawEdges
        )
        mesh.meshDataChanged()
