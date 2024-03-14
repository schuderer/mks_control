from __future__ import annotations

import time
from math import sin, cos, atan2, sqrt, pi
from typing import Optional

import matplotlib.pyplot as plt

# Constants
CLASSIC = 'classic'
MODIFIED = 'modified'
OFFSET = 'offset'  # modified but using alpha and a from the previous link

class Matrix:
    """An applied 4x4 Denavit-Hartenberg matrix that stores the transformations for a particular joint angle.
    A series of multiplied matrices is used to calculate the position and orientation of the end effector of
    a robotic arm.

    :param matrix: A 4x4 matrix (list of lists) representing the transformation matrix
    """
    def __init__(self, matrix=None):
        """Initialize the matrix.
        :param matrix: A 4x4 matrix (list of lists) representing the transformation matrix (default: identity matrix)
        """
        if matrix is None:
                matrix = [[1 if i == j else 0 for j in range(4)] for i in range(4)]
        elif len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise ValueError(f"Matrix must be a 4x4 list of lists. Got {len(matrix)}x{len(matrix[0])}")
        elif any(not isinstance(cell, (int, float)) for row in matrix for cell in row):
            raise ValueError(f"Matrix must contain only numbers: {matrix}")
        # elif matrix[3] != [0, 0, 0, 1]:
        #     raise ValueError(f"The bottom row of the matrix must be [0, 0, 0, 1]. It is {matrix[3]}")
        self._matrix = matrix

    def __mul__(self, other: Matrix):
        """Multiply two 4x4 matrices"""
        result = [[0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    result[i][j] += self._matrix[i][k] * other._matrix[k][j]
        return Matrix(result)

    @property
    def translation_vector(self):
        """Return a copy of the translation vector"""
        return [self._matrix[0][3], self._matrix[1][3], self._matrix[2][3]]

    @property
    def x(self):
        return self._matrix[0][3]

    @property
    def y(self):
        return self._matrix[1][3]

    @property
    def z(self):
        return self._matrix[2][3]

    @property
    def rotation_matrix(self):
        """Return a copy of the 3x3 rotation matrix"""
        return [[self._matrix[i][j] for j in range(3)] for i in range(3)]

    @property
    def alpha(self):
        """Return the rotation angle around the x-axis in radians"""
        return atan2(self._matrix[2][1], self._matrix[2][2])

    @property
    def beta(self):
        """Return the rotation angle around the y-axis in radians"""
        return atan2(-self._matrix[2][0], sqrt(self._matrix[2][1]**2 + self._matrix[2][2]**2))

    @property
    def gamma(self):
        """Return the rotation angle around the z-axis in radians"""
        return atan2(self._matrix[1][0], self._matrix[0][0])

    @property
    def euler_angles(self):
        """Return the rotation angles around the x, y, and z axes in radians"""
        return self.alpha, self.beta, self.gamma

    @property
    def alpha_deg(self):
        """Return the rotation angle around the x-axis in degrees"""
        return self.alpha * 180 / pi

    @property
    def beta_deg(self):
        """Return the rotation angle around the y-axis in degrees"""
        return self.beta * 180 / pi

    @property
    def gamma_deg(self):
        """Return the rotation angle around the z-axis in degrees"""
        return self.gamma * 180 / pi

    @property
    def euler_angles_deg(self):
        """Return the rotation angles around the x, y, and z axes in degrees"""
        return [self.alpha_deg, self.beta_deg, self.gamma_deg]

    def T(self):
        """Return the transpose of the matrix"""
        return Matrix([[self._matrix[j][i] for j in range(4)] for i in range(4)])

    def __getitem__(self, item):
        return self._matrix[item]

    def __str__(self):
        # pretty print the 4x4 matrix
        return "\n".join(" ".join(f"{cell:.2f}" for cell in row) for row in self._matrix)

    def __repr__(self):
        return f"Matrix({str(self._matrix)})"


class Link:
    def __init__(self, alpha, a, theta_offset, d, lower_bound=None, upper_bound=None, use_degrees=False, convention=MODIFIED, fixed=False):
        """Prepare a 4x4 Denavit-Hartenberg matrix for a single link
        :param a: The distance from the z-axis of the previous link to the common normal
        :param d: The distance from the x-axis of the previous link to the common normal
        :param alpha: The angle from the z-axis of the previous link to the common normal
        :param theta_offset: The angle between the x-axes of the previous and current link
        :param lower_bound: The lower bound of the link angle
        :param upper_bound: The upper bound of the link angle
        :param use_degrees: If True, the link angles are expected in degrees on initialization, otherwise in radians (default: False)
        :param convention: Use classic (distal) or modified (proximal) DH parameters (default: kinematics.MODIFIED)
        """
        self.alpha = alpha if not use_degrees else alpha * pi / 180
        self.a = a
        self.theta_offset = theta_offset if not use_degrees else theta_offset * pi / 180
        self.d = d
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.convention = convention
        self.fixed = fixed
        sin_alpha = sin(self.alpha)
        cos_alpha = cos(self.alpha)
        if abs(sin_alpha) < 0.000000001:
            sin_alpha = 0
        if abs(cos_alpha) < 0.000000001:
            cos_alpha = 0

        def rot_z_classic(theta: float = 0):
            theta += self.theta_offset
            sin_theta = sin(theta)
            cos_theta = cos(theta)
            # Classic (distal) DH parameters
            # https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters, https://www.youtube.com/watch?v=8FdBSGSRCmY
            return [
                [cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, a * cos_theta],
                [sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
                [        0,              sin_alpha,              cos_alpha,             d],
                [        0,                      0,                      0,             1]
            ]

        def rot_z_modified(theta: float = 0):
            theta += self.theta_offset
            sin_theta = sin(theta)
            cos_theta = cos(theta)
            # Modified (proximal) Denavit-Hartenberg parameters
            # https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters
            return [
                [            cos_theta,             -sin_theta,          0,                a],
                [sin_theta * cos_alpha,  cos_theta * cos_alpha, -sin_alpha,   -sin_alpha * d],
                [sin_theta * sin_alpha,  cos_theta * sin_alpha,  cos_alpha,    cos_alpha * d],
                [                    0,                      0,          0,                1]
            ]

        if self.convention == CLASSIC:
            self._rot_z = rot_z_classic
        elif self.convention == MODIFIED:
            self._rot_z = rot_z_modified
        else:
            raise ValueError(f"Unknown DH convention: {self.convention}. Use kinematics.CLASSIC or kinematics.MODIFIED")

        if self.fixed:
            self.fixed_matrix = Matrix(self._rot_z())

    def __call__(self, theta: float = 0, fix_bounds=True, use_degrees=False):
        """Calculate and return the 4x4 Denavit-Hartenberg matrix for a given link angle"""
        if self.fixed:
            return self.fixed_matrix
        if use_degrees:
            theta = theta * pi / 180
        if self.lower_bound is not None and theta < self.lower_bound:
            if fix_bounds:
                theta = self.lower_bound
            else:
                raise ValueError(f"Link angle {theta} is below the lower bound {self.lower_bound}")
        if self.upper_bound is not None and theta > self.upper_bound:
            if fix_bounds:
                theta = self.upper_bound
            else:
                raise ValueError(f"Link angle {theta} is above the upper bound {self.upper_bound}")
        return Matrix(self._rot_z(theta))

    # def __str__(self):
    #     sa = sin(self.alpha)
    #     if abs(sa) < 0.000000001:
    #         sa = 0
    #     if round(sa) == sa:
    #         sa = int(sa)
    #     ca = cos(self.alpha)
    #     if abs(ca) < 0.000000001:
    #         ca = 0
    #     if round(ca) == ca:
    #         ca = int(ca)
    #     a = self.a
    #     d = self.d
    #     theta_offset_deg = self.theta_offset * 180 / pi
    #     theta = f"θi+{theta_offset_deg:.0f}" if self.theta_offset != 0 else "θi"
    #     matrix_str = f"""
    #     [
    #         [cos({theta}), {f'-sin({theta})' if ca==1 else (0 if ca==0 else f'-sin({theta})*'+str(ca))},  {f'sin({theta})' if sa==1 else (0 if sa==0 else f'sin({theta})*'+str(sa))}, {f'cos({theta})' if a==1 else (0 if a==0 else str(a)+f'*cos({theta})')}],
    #         [sin({theta}),  {f'cos({theta})' if ca==1 else (0 if ca==0 else f'cos({theta})*'+str(ca))}, {f'-cos({theta})' if sa==1 else (0 if sa==0 else f'-cos({theta})*'+str(sa))}, {f'sin({theta})' if a==1 else (0 if a==0 else str(a)+f'*sin({theta})')}],
    #         [      0,       {sa},     {ca},       {d}],
    #         [      0,       0,     0,         1]
    #     ]"""
    #     return matrix_str

    def __repr__(self):
        return (f"Link(alpha={self.alpha}, a={self.a}, theta_offset={self.theta_offset}, d={self.d}, "
                f"lower_bound={self.lower_bound}, upper_bound={self.upper_bound}, "
                f"convention={self.convention}, fixed={self.fixed})")


class KinematicChain:
    def __init__(self, links: list[Link]):
        """Initialize a kinematic chain with a list of links"""
        self.links = links
        self._current_visualization: Optional[plt.Axes] = None

    @classmethod
    def from_configuration(cls, arm) -> KinematicChain:
        links = []
        alpha_i_minus_1, a_i_minus_1 = 0, 0
        for i, params in enumerate(arm.KINEMATIC_CHAIN):
            alpha_i, a_i, theta_i, d_i = params
            fixed = i in arm.FIXED_LINKS
            print(f"Link {i + 1} params: {params}")
            if arm.KINEMATIC_CONVENTION == OFFSET:
                link = Link(alpha_i_minus_1, a_i_minus_1, theta_i, d_i, use_degrees=True, convention=MODIFIED,
                            fixed=fixed)
            else:
                link = Link(alpha_i, a_i, theta_i, d_i, use_degrees=True,
                            convention=arm.KINEMATIC_CONVENTION,
                            fixed=fixed)
            print(f"Link {i + 1}: {link}")
            links.append(link)
            alpha_i_minus_1 = alpha_i
            a_i_minus_1 = a_i
        return cls(links)

    def forward_kinematics(self, thetas: list[float], link_indices=None, fix_bounds=True) -> Matrix:
        """Calculate the forward kinematics for a given set of link angles"""
        links = link_indices if link_indices is not None else self.links
        full_thetas = self._get_thetas_with_fixed_links(thetas, links)
        result = Matrix()
        for i, (link, theta) in enumerate(zip(links, full_thetas)):
            result *= link(theta, fix_bounds)
            print(f"Result after link {i + 1}:\n{result}")
        return result

    def visualize_link_angles(self, thetas: list[float], link_indices=None, fix_bounds=False, interactive=False, use_degrees=False) -> plt.Axes:
        """Visualize the link angles by showing small rgb coordinate system vectors (3 lines) for each link in a 3d plot,
        with the rgb lines' origin being translated correctly according to the link's transformation matrix,
        and the rgb lines' orientation being rotated according to the link's transformation matrix.
        Taking into consideration whether the DH parameters are in classic or modified form.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        self.update_visualization(thetas, link_indices=link_indices, fix_bounds=fix_bounds, use_degrees=use_degrees, axis_obj=ax)
        if interactive:
            plt.ion()
        plt.show()
        self._current_visualization = ax
        return ax

    def update_visualization(self, thetas: list[float], link_indices=None, fix_bounds=False, use_degrees=False, axis_obj: Optional[plt.Axes] = None):
        """Update the data in an existing plot.
        """
        ax = axis_obj or self._current_visualization
        ax.clear()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-500, 500)  # Todo: make these limits dynamic
        ax.set_ylim(-500, 500)
        ax.set_zlim(0, 1000)
        ax.set_title('Visualizing Link Angles')
        links = link_indices if link_indices is not None else self.links
        full_thetas = self._get_thetas_with_fixed_links(thetas, links)
        s = 80  # arrow scale factor
        result = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for i, (link, theta) in enumerate(zip(self.links, full_thetas)):
            if use_degrees:
                theta = theta * pi / 180
            x, y, z = result.translation_vector
            r = result.rotation_matrix
            # First, add a label with the link number to the new origin
            ax.text(x, y, z, str(i+1), color='gray')
            # then plot the coordinate system vectors (in x = red, y = green, z = blue) at the new origin:
            for xyz in range(3):
                ax.quiver(x, y, z, r[0][xyz]*s, r[1][xyz]*s, r[2][xyz]*s, color=['r', 'g', 'b'][xyz])
            result *= link(theta, fix_bounds)
        result *= Matrix()  # end effector (identity matrix)
        x, y, z = result.translation_vector
        r = result.rotation_matrix
        ax.text(x, y, z, "EE", color='gray')
        for xyz in range(3):
            ax.quiver(x, y, z, r[0][xyz] * s, r[1][xyz] * s, r[2][xyz] * s, color=['r', 'g', 'b'][xyz])
        # print("3D plot updated")

    @staticmethod
    def sleep(seconds: float):
        """Sleep for a given number of seconds"""
        plt.pause(seconds)

    @staticmethod
    def close_visualization():
        plt.close()

    @staticmethod
    def _get_thetas_with_fixed_links(thetas: list[float], links: list[Link]) -> list[float]:
        len_including_fixed_links = len(links)
        len_without_fixed_links = len([j for j in links if not j.fixed])
        if len(thetas) == len_without_fixed_links and len_including_fixed_links != len_without_fixed_links:
            for i, j in enumerate(links):
                if j.fixed:
                    thetas.insert(i, 0)
        if len(thetas) != len_including_fixed_links:
            expectation = f"{len_without_fixed_links} or {len_including_fixed_links}" if len_including_fixed_links != len_without_fixed_links else len_without_fixed_links
            raise ValueError(f"Expected {expectation} link angles, got {len(thetas)}")
        return thetas

    def __str__(self):
        result = "KinematicChain([\n  " + ",\n  ".join(str(link) for link in self.links) + "\n])"
        return result

    def __repr__(self):
        return str(self)

    def __getitem__(self, item):
        return self.links[item]

    def __len__(self):
        return len(self.links)


if __name__ == "__main__":
    # Example usage
    # Define the kinematic chain
    import arctos_arm
    # reload if already imported
    from importlib import reload
    reload(arctos_arm)
    chain = KinematicChain.from_configuration(arctos_arm)
    print(chain)
    # Calculate the forward kinematics
    result = chain.forward_kinematics([0]*len(chain), fix_bounds=False)
    # Visualize the link angles
    animate_link_indices = [2]
    theta_angle = [0.0] * len(chain)
    increment = 5 * pi / 180
    max_angle = pi / 4
    min_angle = -pi / 4
    chain.visualize_link_angles([0] * len(chain), fix_bounds=False, interactive=True)
    now = time.time()
    while time.time() - now < 10:
        plt.pause(0.1)
        for i in animate_link_indices:
            theta_angle[i] += increment
            if theta_angle[i] > max_angle:
                increment *= -1
                theta_angle[i] = max_angle
            elif theta_angle[i] < min_angle:
                increment *= -1
                theta_angle[i] = min_angle
        chain.update_visualization(theta_angle, fix_bounds=False)
    plt.close()

    # fig = plt.figure()
    #         ax = fig.add_subplot(111, projection='3d')
    #         ax.set_xlabel('X')
    #         ax.set_ylabel('Y')
    #         ax.set_zlabel('Z')
    #         ax.set_xlim(-100, 100)
    #         ax.set_ylim(-100, 100)
    #         ax.set_zlim(-100, 100)
    #         ax.set_title('Visualizing Link Angles')
    #         result = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    #         for link, theta in zip(self.links, thetas):
    #             result *= link(theta, fix_bounds)
    #             x, y, z = result.translation_vector
    #             ax.quiver(0, 0, 0, x, y, z, arrow_length_ratio=0.1)
    #         plt.show()
    #         return "3D plot displayed"


#        fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')
#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_zlabel('Z')
#         ax.set_xlim(-500, 500)
#         ax.set_ylim(-500, 500)
#         ax.set_zlim(-500, 500)
#         ax.set_box_aspect([1, 1, 1])
#         ax.set_title("Visualization of link angles")
#
#         result = Matrix([[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]])
#         for link, theta in zip(self.links, thetas):
#             result *= link(theta, fix_bounds)
#             x, y, z = result.translation_vector
#             r = result.rotation_matrix
#             for i in range(3):
#                 ax.plot([x, x + r[i][0]], [y, y + r[i][1]], [z, z + r[i][2]])
#         plt.show()
#         return "3D plot displayed"
