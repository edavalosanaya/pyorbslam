import os
import pathlib
import numpy as np
from typing import List

import cv2
import pandas as pd
from plyfile import PlyData, PlyElement

from .state import State
from .slam import ASLAM

def record_slam_data(step_id: int, slam: ASLAM, dir: pathlib.Path, items: List = ['pose', 'image', 'point cloud']):

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

            elif item == 'image':
                cv2.imwrite(str(dir/f'image_{step_id}.jpg'), slam.get_current_frame())
    
    # Update the meta record if present
    meta_file = dir / 'meta.csv'
    if meta_file.exists():
        meta_df = pd.read_csv(meta_file)
    else:
        meta_df = pd.DataFrame()

    # Find the row matching the step id
    try:
        row_index = meta_df.index[meta_df['step_id'] == step_id]
    except KeyError:
        row_index = []

    # Update the row with the new incoming data
    new_data = slam.get_state() == State.OK  # Replace with your new incoming data
    if len(row_index) > 0:
        meta_df.loc[row_index[0]] = {'step_id': step_id, 'data': new_data}
    else:
        meta_df = meta_df.append({'step_id': step_id, 'data': new_data}, ignore_index=True)

    # Save the modified DataFrame back to the CSV file
    meta_df.to_csv(meta_file, index=False)

def load_slam_data(step_id: int, dir: pathlib.Path, items: List = ['pose', 'image', 'point cloud']):
    
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

            # Load image
            elif item == 'image':
                data[item] = cv2.imread(str(dir/f"image_{step_id}.jpg"), 0) 

            # Load the pose
            elif item == 'point cloud':
                with open(dir/f"pc_{step_id}.ply", "rb") as f:
                    ply_data = PlyData.read(f)
                    vertex = ply_data['vertex'].data
                    data[item] = np.array(vertex).view(np.float32).reshape((vertex.shape[0],3))
        
        return data

    return None


def string_to_numpy(string):
    string = string.replace('[', '').replace(']', '').replace('\n', '')
    array = np.fromstring(string, sep=' ')
    array = array.reshape((4, 4))
    return array


def apply_rt_to_pts(pts, rt):
    homo_pts = np.hstack((pts, np.ones((pts.shape[0],1))))
    t_homo_pts = np.dot(rt, homo_pts.T).T
    return t_homo_pts[:, :3]
