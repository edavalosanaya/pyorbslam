import numpy as np
import pyvista as pv

# Creating random data
N = 100
data = np.zeros((1, N, N))
data[:] = np.random.randint(0, 10000, data.shape)

# Creating a mesh from our data
g = pv.UniformGrid()
g.dimensions = np.array(data.shape) + 1
g.spacing = (10, 10, 10)
g.cell_data['data'] = data.flatten()
#Careful with threshold as it will turn your data into UnstructuredGrid
#g = g.threshold([0.0001, int(data.max())])

# Creating scene and loading the mesh
p = pv.Plotter()
p.add_mesh(g, opacity=0.5, name='data', cmap='gist_ncar')
p.show(interactive_update=True)

# Animation
for i in range(5, 1000):
    # Updating our data
    data = np.full((i, N, N),0)
    data[:] = np.random.randint(0,1000000, data.shape)
    
    # Recreating the mesh
    g = pv.UniformGrid()
    g.dimensions = np.array(data.shape) + 1
    g.spacing = (10, 10, 10)
    g.cell_data['data'] = data.flatten()

    # Reloading the mesh to the scene
    p.clear()
    p.add_mesh(g, opacity=0.5, name='data')

    # Redrawing
    p.update()
