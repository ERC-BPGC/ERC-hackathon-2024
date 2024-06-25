import numpy as np
import matplotlib.pyplot as plt
from stl import mesh
from matplotlib.collections import PolyCollection

def plot_stl_projection_y_axis_filled(stl_file_path, output_png_path):
    # Load the STL file
    your_mesh = mesh.Mesh.from_file(stl_file_path)
    
    # Extract the vertices
    vectors = your_mesh.vectors
    
    #save vectors as npy file
    np.save('roads.npy', vectors)
    
    # Project the vertices onto the Y-axis
    projected_vectors = vectors[:, :, [0, 2]]  # Keep only X and Z components
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(20, 20), dpi=600)
    
    # Create a PolyCollection from the projected vectors
    poly_collection = PolyCollection(projected_vectors, edgecolors='k', facecolors='grey')
    ax.add_collection(poly_collection)
    
    ax.autoscale()
    ax.set_aspect('equal')
    ax.set_title('STL Projection on Y-Axis')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Z Axis')
    
    # Save the plot as a PNG image
    plt.savefig(output_png_path, dpi=600)
    plt.show()

# Example usage
stl_file_path = 'roads.stl'
output_png_path = 'output_projection_high_res_filled.png'
# plot_stl_projection_y_axis_filled(stl_file_path, output_png_path)

def visualize_map_np(roads_path = 'roads.npy'):
    
     #save vectors as npy file
    vectors = np.load('roads.npy')
    
    # Project the vertices onto the Y-axis
    projected_vectors = vectors[:, :, [0, 2]]  # Keep only X and Z components
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(20, 20), dpi=600)
    
    # Create a PolyCollection from the projected vectors
    poly_collection = PolyCollection(projected_vectors, edgecolors='k', facecolors='grey')
    ax.add_collection(poly_collection)
    
    ax.autoscale()
    ax.set_aspect('equal')
    ax.set_title('STL Projection on Y-Axis')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Z Axis')
    
    # Save the plot as a PNG image
    # plt.savefig(output_png_path, dpi=600)
    plt.show()
    
visualize_map_np()