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
                vertex = np.array([(x,y,z) for x,y,z in slam.get_point_cloud()], dtype=("f4,f4,f4"))
                ply_data = PlyData([PlyElement.describe(vertex, 'vertex')])
                ply_data.write(str(dir/f'pc_{step_id}.ply'))
    
    # Update the meta record if present
    meta_file = dir / 'meta.csv'
    if meta_file.exists():
        meta_df = pd.read_csv(meta_file)
    else:
        meta_df = pd.DataFrame()

    # Find the row matching the step id
    row_index = meta_df.index[meta_df['step_id'] == step_id]

    # Update the row with the new incoming data
    new_data = slam.get_state() == State.OK  # Replace with your new incoming data
    if len(row_index) > 0:
        meta_df.loc[row_index[0]] = {'step_id': step_id, 'data': new_data}
    else:
        meta_df = meta_df.append({'step_id': step_id, 'data': new_data}, ignore_index=True)

    # Save the modified DataFrame back to the CSV file
    meta_df.to_csv(meta_file, index=False)

def load_slam_data(step_id: int, dir: pathlib.Path, items: List = ['pose', 'point cloud']):
    
    # First check the directory
    if not dir.exists():
        raise FileNotFoundError

    # Then load the meta data
    meta_df = pd.read_csv(dir/'meta.csv')

    # Get the step data
    row = meta_df[meta_df['step_id'] == step_id]

    # Check if found row
    if not row.empty:
    
        # Data Container
        data = {}

        for item in items:

            # Load point cloud
            if item == 'pose':
                with open(dir/f"pose_{step_id}.npy", 'rb') as f:
                    data[item] = np.load(f)

            # Load the pose
            elif item == 'point cloud':
                with open(dir/f"pc_{step_id}.ply", "rb") as f:
                    ply_data = PlyData.read(f)
                    vertex = ply_data['vertex'].data
                    data[item] = np.array(vertex).view(np.float32).reshape((vertex.shape[0],3))
        
        return data

    return None
