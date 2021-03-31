#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Utils for EE106B grasp planning project.
Author: Chris Correa.
Adapted for Spring 2020 by Amay Saxena
"""
import numpy as np
from math import sin, cos, atan2, sqrt
from trimesh import proximity

def find_intersections(mesh, p1, p2):
    """
    Finds the points of intersection between an input mesh and the
    line segment connecting p1 and p2.

    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    p1 (3x np.ndarray): line segment point
    p2 (3x np.ndarray): line segment point

    Returns
    -------
    on_segment (2x3 np.ndarray): coordinates of the 2 intersection points
    faces (2x np.ndarray): mesh face numbers of the 2 intersection points
    """
    ray_origin = (p1 + p2) / 2
    ray_length = np.linalg.norm(p1 - p2)
    ray_dir = (p2 - p1) / ray_length
    locations, index_ray, index_tri = mesh.ray.intersects_location(
        ray_origins=[ray_origin, ray_origin],
        ray_directions=[ray_dir, -ray_dir],
        multiple_hits=True)
    dist_to_center = np.linalg.norm(locations - ray_origin, axis=1)
    dist_mask = dist_to_center <= (ray_length / 2) # only keep intersections on the segment.
    on_segment = locations[dist_mask]
    faces = index_tri[dist_mask]
    return on_segment, faces

def find_grasp_vertices(mesh, p1, p2):
    """
    If the tips of an ideal two fingered gripper start off at
    p1 and p2 and then close, where will they make contact with the object?
    
    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    p1 (3x np.ndarray): starting gripper point
    p2 (3x np.ndarray): starting gripper point

    Returns
    -------
    locations (nx3 np.ndarray): coordinates of the closed gripper's n contact points
    face_ind (nx np.ndarray): mesh face numbers of the closed gripper's n contact points
    """
    ray_dir = p2 - p1
    locations, index_ray, face_ind = mesh.ray.intersects_location(
        ray_origins=[p1, p2],
        ray_directions=[p2 - p1, p1 - p2],
        multiple_hits=False)
    return locations, face_ind

def normal_at_point(mesh, p):
    """
    Returns the normal vector to the mesh at a point p.
    Requires that p is a point on the surface of the mesh (or at least
    that it is very close to a point on the surface).
    
    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    p (3x np.ndarray): point to get normal at

    Returns
    -------
    (3x np.ndarray): surface normal at p
    """
    point, dist, face = proximity.closest_point(mesh, [p])
    if dist > 0.001:
        print("Input point is not on the surface of the mesh!")
        return None
    return mesh.face_normals[face[0]]

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec (nx np.ndarray): vector to normalize

    Returns
    -------
    (nx np.ndarray): normalized vector
    """
    return vec / np.linalg.norm(vec)

def hat(v):
    """
    See https://en.wikipedia.org/wiki/Hat_operator or the MLS book

    Parameters
    ----------
    v (3x, 3x1, 6x, or 6x1 np.ndarray): vector to create hat matrix for

    Returns
    -------
    (3x3 or 6x6 np.ndarray): the hat version of the v
    """
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def adj(g):
    """
    Adjoint of a rotation matrix. See the MLS book.

    Parameters
    ----------
    g (4x4 np.ndarray): homogenous transform matrix

    Returns
    -------
    (6x6 np.ndarray): adjoint matrix
    """
    if g.shape != (4, 4):
        raise ValueError

    R = g[0:3,0:3]
    p = g[0:3,3]
    result = np.zeros((6, 6))
    result[0:3,0:3] = R
    result[0:3,3:6] = np.matmul(hat(p), R)
    result[3:6,3:6] = R
    return result

def look_at_general(origin, direction):
    """
    Creates a homogenous transformation matrix at the origin such that the 
    z axis is the same as the direction specified. There are infinitely 
    many of such matrices, but we choose the one where the y axis is as 
    vertical as possible.  

    Parameters
    ----------
    origin (3x np.ndarray): origin coordinates
    direction (3x np.ndarray): direction vector

    Returns
    -------
    (4x4 np.ndarray): homogenous transform matrix
    """
    up = np.array([0, 0, 1])
    z = normalize(direction) # create a z vector in the given direction
    x = normalize(np.cross(up, z)) # create a x vector perpendicular to z and up
    y = np.cross(z, x) # create a y vector perpendicular to z and x

    result = np.eye(4)

    # set rotation part of matrix
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z

    # set translation part of matrix to origin
    result[0:3,3] = origin

    return result
