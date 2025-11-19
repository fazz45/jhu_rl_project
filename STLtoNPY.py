import numpy as np
from stl import mesh
import os

# Define the full absolute path to the STL file
# Replace 'yourusername' with your actual Linux/macOS username
# or use os.path.expanduser to handle the ~ automatically
stl_path = os.path.expanduser('~/jhu_rl_project/MarsTerrain.stl')

# 1. Load the STL file
if os.path.exists(stl_path):
    your_mesh = mesh.Mesh.from_file(stl_path)
    
    # Access the data as a numpy array
    mesh_data_array = your_mesh.data
    points_array = your_mesh.points 

    # 2. Save the NumPy arrays (they will save in the directory where you run the script from)
    np.save('MarsTerrain.npy', mesh_data_array)
    np.save('MarsTerrainPointsArray.npy', points_array)
    
    print(f"STL file at '{stl_path}' converted and saved successfully.")
else:
    print(f"Error: The file '{stl_path}' was not found.")


