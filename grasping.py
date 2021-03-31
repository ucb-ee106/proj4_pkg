#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter code for EE106B grasp planning project.
Author: Amay Saxena, Tiffany Cappellari
"""
# may need more imports
import numpy as np
from utils import *
import math
import trimesh
import vtkplotter

MAX_GRIPPER_DIST = 0.075
MIN_GRIPPER_DIST = 0.03
GRIPPER_LENGTH = 0.105


def compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Compute the force closure of some object at contacts, with normal vectors 
    stored in normals. Since this is two contact grasp, we are using a basic algorithm
    wherein a grasp is in force closure as ling as the line connecting the two contact
    points lies in both friction cones.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    normal0 = -1.0 * normals[0] / (1.0 * np.linalg.norm(normals[0]))
    normal1 = -1.0 * normals[1] / (1.0 * np.linalg.norm(normals[1]))

    alpha = np.arctan(mu)
    line = vertices[0] - vertices[1]
    line = line / (1.0 * np.linalg.norm(line))
    angle1 = np.arccos(normal1.dot(line))

    line = -1 * line
    angle2 = np.arccos(normal0.dot(line))

    if angle1 > alpha or angle2 > alpha:
        return 0
    if gamma == 0:
        return 0
    return 1


def get_grasp_map(vertices, normals, num_facets, mu, gamma):
    """ 
    defined in the book on page 219.  Compute the grasp map given the contact
    points and their surface normals

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    :obj:`numpy.ndarray` grasp map
    """
    # YOUR CODE HERE
    raise NotImplementedError


def contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench):
    """
    Compute the minimum norm input force required for the given grasp 
    (at contacts with surface normals) to produce the desired_wrench. 
    This is a utility function that you may choose to implement to 
    use in other functions.

    If G is the grasp map and w the given desired_wrench, this function would compute

    argmin ||f||^2
    subject to f in friction cone and Gf = w

    You can then use this function in the gravity resistence metric and also to
    implement LQ in the ferrari canny metric. What you return from this function
    is up to you. You may choose to return the final objective value, the solution f,
    and a boolean indicating whether the problem is feasible.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors 
        will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : :obj:`numpy.ndarray`
        potential wrench to be produced

    Returns
    -------
    up to you.
    """
    raise NotImplementedError


def compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Gravity produces some wrench on your object. Computes whether the grasp can 
    produce an equal and opposite wrench.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will 
        be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    raise NotImplementedError


def compute_robust_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Should return a score for the grasp according to the robust force closure metric.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will 
        be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    raise NotImplementedError


def compute_ferrari_canny(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Should return a score for the grasp according to the Ferrari Canny metric.
    Use your favourite python convex optimization package. We suggest cvxpy or casadi.

    Parameters
    ----------
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : 2x3 :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will 
        be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    raise NotImplementedError


def custom_grasp_planner(object_mesh, vertices):
    """
    Write your own grasp planning algorithm! You will take as input the mesh
    of an object, and a pair of contact points from the surface of the mesh.
    You should return a 4x4 ridig transform specifying the desired pose of the
    end-effector (the gripper tip) that you would like the gripper to be at
    before closing in order to execute your grasp.

    You should be prepared to handle malformed grasps. Return None if no
    good grasp is possible with the provided pair of contact points.
    Keep in mind the constraints of the gripper (length, minimum and maximum
    distance between fingers, etc) when picking a good pose, and also keep in
    mind limitations of the robot (can the robot approach a grasp from the inside
    of the mesh? How about from below?). You should also make sure that the robot
    can successfully make contact with the given contact points without colliding
    with the mesh.

    The trimesh package has several useful functions that allow you to check for
    collisions between meshes and rays, between meshes and other meshes, etc, which
    you may want to use to make sure your grasp is not in collision with the mesh.

    Take a look at the functions find_intersections, find_grasp_vertices, 
    normal_at_point in utils.py for examples of how you might use these trimesh 
    utilities. Be wary of using these functions directly. While they will probably 
    work, they don't do excessive edge-case handling. You should spend some time
    reading the documentation of these packages to find other useful utilities.
    You may also find the collision, proximity, and intersections modules of trimesh
    useful.

    Feel free to change the signature of this function to add more arguments
    if you believe they will be useful to your planner.

    Parameters
    ----------
    object_mesh : A triangular mesh of the object, as loaded in with trimesh.
    vertices : 2x3 :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed

    Returns
    -------
    numpy.ndarray of shape (4, 4). Should be the rigid transform for the desired
    pose of the gripper, in the object's reference frame.
    """
    raise NotImplementedError


def visualize_grasp(mesh, vertices, pose):
    """Visualizes a grasp on an object. Object specified by a mesh, as
    loaded by trimesh. vertices is a pair of (x, y, z) contact points.
    pose is the pose of the gripper tip.
    """
    p1, p2 = vertices
    center = (p1 + p2) / 2
    approach = pose[:3, 2]
    tail = center - GRIPPER_LENGTH * approach

    contact_points = []
    for v in vertices:
        contact_points.append(vtkplotter.shapes.Point(pos=v, r=30))

    vec = (p1 - p2) / np.linalg.norm(p1 - p2)
    line = vtkplotter.shapes.Tube([center + 0.5 * MAX_GRIPPER_DIST * vec,
                                   center - 0.5 * MAX_GRIPPER_DIST * vec], r=0.001, c='g')
    approach = vtkplotter.shapes.Tube([center, tail], r=0.001, c='g')
    vtkplotter.show([mesh, line, approach] + contact_points, newPlotter=True)


def randomly_sample_from_mesh(mesh, n):
    """Example of sampling points from the surface of a mesh.
    Returns n (x, y, z) points sampled from the surface of the input mesh
    uniformly at random. Also returns the corresponding surface normals.
    """
    vertices, face_ind = trimesh.sample.sample_surface_even(mesh, self.n_vert)
    normals = mesh.face_normals[face_ind]
    return vertices, normals


def load_grasp_data(object_name):
    """Loads grasp data from the provided NPZ files. It returns three arrays:

    grasp_vertices: numpy.ndarray of shape (5, 2, 3). For each of the 5 grasps,
            this stores a pair of (x, y, z) locations that are the contact points
            of the grasp.
    normals: numpy.ndarray of shape (5, 2, 3). For each grasp, stores the normal
            vector to the mesh at the two contact points. Remember that the normal
            vectors to a closed mesh always point OUTWARD from the mesh.
    tip_poses: numpy.ndarray of shape (5, 4, 4). For each of the five grasps, this
            stores the 4x4 rigid transform of the reference frame of the gripper
            tip before the gripper is closed in order to grasp the object.
    results: numpy.ndarray of shape (5, 5). Stores the result of five trials for
            each of the five grasps. Entry (i, j) is a 1 if the jth trial of the
            ith grasp was successful, 0 otherwise.
    """
    data = np.load('grasp_data/{}.npz'.format(object_name))
    return data['grasp_vertices'], data['normals'], data['tip_poses'], data['results']


def load_mesh(object_name):
    mesh = trimesh.load_mesh('objects/{}.obj'.format(object_name))
    mesh.fix_normals()
    return mesh


def main():
    """ Example for interacting with the codebase. Loads data and
    visualizes each grasp against the corresponding mesh of the given
    object.
    """
    obj = 'pawn' # should be 'pawn' or 'nozzle'.
    vertices, normals, poses, results = load_grasp_data(obj)
    mesh = load_mesh(obj)
    for v, p in zip(vertices, poses):
        visualize_grasp(mesh, v, p)


if __name__ == '__main__':
    main()
