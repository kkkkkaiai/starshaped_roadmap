import numpy as np
import warnings
from typing import Optional
import numpy.typing as npt
from numpy import linalg as LA
from functools import lru_cache
from copy import copy

def compute_weight(Gamma, dist_limit=1.0, weight_pow=1.0):
    distances = np.array(Gamma)
    
    # choose the index of biggest value and set it to 1
    max_index = np.argmax(distances)
    w = np.zeros(len(distances))
    w[max_index] = 1.0
    
    return w

def compute_diagonal_matrix(
    Gamma,
    dim,
    rho=1,
    repulsion_coeff=1.0,
    tangent_eigenvalue_isometric=True,
    tangent_power=5,
    treat_obstacle_special=False,
    self_priority=1,
):
    if Gamma <= 1.2:
        delta_eigenvalue = 1.0 / abs(Gamma) ** (self_priority / rho)
        eigenvalue_reference = 1 - delta_eigenvalue
        eigenvalue_tangent = 1
    else:
        eigenvalue_reference = 1
        eigenvalue_tangent = 1

    # no need to change
    return np.diag(
        np.hstack((eigenvalue_reference, np.ones(dim - 1) * eigenvalue_tangent))
    )


# @lru_cache(maxsize=10)
# TODO: expand cache for this [numpy-arrays]
# TODO: OR make cython
def get_orthogonal_basis(vector: np.ndarray, normalize: bool = True) -> np.ndarray:
    """Get Orthonormal basis matrxi for an dimensional input vector."""
    # warnings.warn("Basis implementation is not continuous.") (?! problem?)
    if not (vector_norm := np.linalg.norm(vector)):
        warnings.warn("Zero norm-vector.")
        return np.eye(vector.shape[0])

    vector = vector / vector_norm

    dim = vector.shape[0]
    if dim <= 1:
        return vector.reshape((dim, dim))

    basis_matrix = np.zeros((dim, dim))

    if dim == 2:
        basis_matrix[:, 0] = vector
        basis_matrix[:, 1] = np.array([-basis_matrix[1, 0], basis_matrix[0, 0]])
    else:
        # exit
        warnings.warn("Basis dim>2 implementation is not continuous.")
        exit()

    return basis_matrix


class UnitDirectionError(Exception):
    def __init__(self, message="Error with Unit Direction Handling"):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return f"{self.message}"


def get_directional_weighted_sum(
    null_direction: np.ndarray,
    weights: npt.ArrayLike,
    directions: np.ndarray,
    unit_directions: list = None,
    total_weight: float = 1,
    normalize: bool = True,
    normalize_reference: bool = True,
) -> np.ndarray:
    """Weighted directional mean for inputs vector ]-pi, pi[ with respect to the null_direction

    Parameters
    ----------
    null_direction: basis direction for the angle-frame
    directions: the directions which the weighted sum is taken from
    unit_direction: list of unit direction
    weights: used for weighted sum
    total_weight: [<=1]
    normalize: variable of type Bool to decide if variables should be normalized

    Return
    ------
    summed_velocity: The weighted sum transformed back to the initial space
    """
    # TODO: this can be vastly speed up by removing the 'unit directions'
    weights = np.array(weights)

    ind_nonzero = np.logical_and(
        weights > 0, LA.norm(directions, axis=0)
    )  # non-negative

    null_direction = np.copy(null_direction)
    directions = directions[:, ind_nonzero]
    weights = weights[ind_nonzero]

    if total_weight > 1:
        weights = weights / np.sum(weights) * total_weight

    n_directions = weights.shape[0]
    if (n_directions == 1) and np.sum(weights) >= 1:
        return directions[:, 0] / LA.norm(directions[:, 0])

    dim = np.array(null_direction).shape[0]

    base = get_orthogonal_basis(vector=null_direction)
    if unit_directions is None:
        unit_directions = [
            UnitDirection(base).from_vector(directions[:, ii])
            for ii in range(directions.shape[1])
        ]
    else:
        for u_dir in unit_directions:
            u_dir.transform_to_base(base)

    summed_dir = UnitDirection(base).from_angle(np.zeros(dim - 1))
    for ii, u_dir in enumerate(unit_directions):
        summed_dir = summed_dir + u_dir * weights[ii]

    if True:
        return summed_dir.as_vector()

def compute_decomposition_matrix(obs, x_t):
    """Compute decomposition matrix and orthogonal matrix to basis"""
    normal_vector = obs._star_rep.get_normal_direction(x_t)
    reference_direction = obs._star_rep.get_reference_direction(x_t)

    E_orth = get_orthogonal_basis(normal_vector, normalize=True)
    E = np.copy((E_orth))
    E[:, 0] = -reference_direction

    return E, E_orth


def get_relative_obstacle_velocity(
    position: np.ndarray,
    obstacle_list,
    E_orth: np.ndarray,
    weights: list,
    ind_obstacles: Optional[int] = None,
    gamma_list: Optional[list] = None,
    cut_off_gamma: float = 1e4,
    velocity_only_in_positive_normal_direction: bool = True,
    normal_weight_factor: float = 1.3,
) -> np.ndarray:
    """Get the relative obstacle velocity

    Parameters
    ----------
    E_orth: array which contains orthogonal matrix with repsect to the normal
    direction at <position>
    array of (dimension, dimensions, n_obstacles
    obstacle_list: list or <obstacle-conainter> with obstacles
    ind_obstacles: Inidicates which obstaces will be considered (array-like of int)
    gamma_list: Precalculated gamma-values (list of float) -
                It is adviced to use 'proportional' gamma values, rather
                than relative ones

    Return
    ------
    relative_velocity: array-like of float
    """
    n_obstacles = len(obstacle_list)

    if gamma_list is None:
        gamma_list = np.zeros(n_obstacles)
        for n in range(n_obstacles):
            gamma_list[n] = obs[n].get_gamma(position, in_global_frame=True)

    if ind_obstacles is None:
        ind_obstacles = gamma_list < cut_off_gamma
        gamma_list = gamma_list[ind_obstacles]

    obs = obstacle_list
    ind_obs = ind_obstacles
    dim = position.shape[0]

    xd_obs = np.zeros((dim))
    for ii, it_obs in zip(range(np.sum(ind_obs)), np.arange(n_obstacles)[ind_obs]):
        xd_obs_n = np.zeros(dim)

        # The Exponential term is very helpful as it help to avoid
        # the crazy rotation of the robot due to the rotation of the object
        if obs[it_obs].is_deforming:
            weight_deform = np.exp(-1 / 1 * (np.max([gamma_list[ii], 1]) - 1))
            vel_deformation = obs[it_obs].get_deformation_velocity(pos_relative[:, ii])

            if velocity_only_in_positive_normal_direction:
                vel_deformation_local = E_orth[:, :, ii].T.dot(vel_deformation)
                if (vel_deformation_local[0] > 0 and not obs[it_obs].is_boundary) or (
                    vel_deformation_local[0] < 0 and obs[it_obs].is_boundary
                ):
                    vel_deformation = np.zeros(vel_deformation.shape[0])

                else:
                    vel_deformation = E_orth[:, 0, ii].dot(vel_deformation_local[0])

            xd_obs_n += weight_deform * vel_deformation
        xd_obs = xd_obs + xd_obs_n * weights[ii]
    return xd_obs


def compute_modulation_matrix(D, E, weight, initial_velocity):
    # M(x) = E(x)*D(x)*E^(-1)
    # M = np.zeros((2, 2, len(weight)))
    vels = np.zeros((2, len(weight)))

    for i in range(len(weight)):
        # M[:, :, i] = E_orth[:, :, i].dot(D[:, :, i]).dot(np.linalg.inv(E_orth[:, :, i]))
        # if the first term of D is 1
        if D[0, 0, i] == 1:
            relative_vel = initial_velocity
        else:
            # print('inv matrix', np.linalg.inv(E[:,:,i]))
            velocity_temp = np.linalg.pinv(E[:,:,i]).dot(initial_velocity)
            velocity_temp = D[:,:,i].dot(velocity_temp)
            relative_vel = E[:,:,i].dot(velocity_temp)

        vels[:, i] = relative_vel

    vel = np.zeros(2)
    for i in range(len(weight)):
        vel += np.array(vels[:, i]).T * weight[i]

    return vel.T


def modulation_velocity(position, goal_position, graph_manager, safety_margin=0.0, project=True):
    attractive_dir = goal_position - position
    initial_velocity = attractive_dir / np.linalg.norm(attractive_dir)
    # print('init velocity', initial_velocity)

    obs_number = len(graph_manager._star_id_list)

    position = np.array(position)
    Gamma = np.zeros(len(graph_manager._star_id_list))

    for i in range(len(graph_manager._star_id_list)):
        id = graph_manager._star_id_list[i]
        Gamma[i] = graph_manager._nodes[id].get_gamma(position)

        # if position has been outside the obstacle, project the postion to the boundary

    max_Gamma = np.max(Gamma)
    if max_Gamma < 1.5 and project:
        max_Gamma_id = np.argmax(Gamma)
        position = graph_manager._nodes[graph_manager._star_id_list[max_Gamma_id]].project_to_boundary(position)
        Gamma[max_Gamma_id] = graph_manager._nodes[graph_manager._star_id_list[max_Gamma_id]].get_gamma(position)
    weight = compute_weight(Gamma)

    E = np.zeros((2, 2, obs_number))
    D = np.zeros((2, 2, obs_number))
    E_orth = np.zeros((2, 2, obs_number))

    for n in range(obs_number):
        D[:, :, n] = compute_diagonal_matrix(Gamma[n], 2)
        E[:, :, n], E_orth[:, :, n] = compute_decomposition_matrix(graph_manager._nodes[graph_manager._star_id_list[n]], position)

    star_obs_list = []
    for i in range(len(graph_manager._star_id_list)):
        star_obs_list.append(graph_manager._nodes[graph_manager._star_id_list[i]]._star_rep)

    new_velocity = compute_modulation_matrix(D, E, weight, initial_velocity)


    return new_velocity, max_Gamma

def modulation_frontier_points(frontier_points, reference_position, star_rep, safety_margin):
    for i in range(len(frontier_points)):
        gamma = star_rep.get_gamma(frontier_points[i])
        attractive_dir = frontier_points[i] - reference_position
        frontier_points[i] = frontier_points[i] + attractive_dir / np.linalg.norm(attractive_dir) * safety_margin

        
