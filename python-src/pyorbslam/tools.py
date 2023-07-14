import os
import pathlib
import numpy as np
from typing import List

import pandas as pd
from plyfile import PlyData, PlyElement

from .state import State
from .slam import ASLAM

def record_slam_data(step_id: int, slam: ASLAM, dir: pathlib.Path, items: List = ['pose', 'point cloud']):

    # If the directory doesn't exists, create it
    if not dir.exists():
        os.mkdir(dir)

    # First, if slam isn't working, continue
    if slam.get_state() == State.OK:

        # For each of the requested items store
        for item in items:
            if item == 'pose':
                with open(dir/f"pose_{step_id}.npy", 'wb') as f:
                    np.save(f, slam.get_pose_to_target())

            elif item == 'point cloud':
                # import pdb; pdb.set_trace()
                vertex = slam.get_point_cloud().astype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
                ply_data = PlyData([PlyElement.describe(vertex, 'vertex').flatten()])
                ply_data.write(str(dir/f'pc_{step_id}.ply'))
    
    # Update the meta record if present
    meta_df = pd.Series({'step_id': step_id, 'data': slam.get_state() == State.OK}).to_frame().T
    meta_df.to_csv(
        str(dir/'meta.csv'),
        mode='a',
        header=not (dir/'meta.csv').exists(),
        index=False
    )

def load_slam_data(dir: pathlib.Path):
    ...
