from __future__ import annotations

import time
from copy import deepcopy
from math import sin, cos, atan2, sqrt, pi
from typing import Optional, Union, Tuple

import matplotlib.pyplot as plt

# Constants
CLASSIC = 'classic'
MODIFIED = 'modified'
OFFSET = 'offset'  # modified but using alpha and a from the previous link


class Matrix:
    """A general matrix class that can be used for any size of matrix"""

    def __init__(self, matrix: Optional[Matrix, list[list[float]], list[float]] = None, shape: Optional[Tuple[int, int]] = None, fill: Union[Matrix, list[float], list[list[float]], float] = 0):
        """Initialize the matrix.
        Initializing with another matrix will create a deep copy of the matrix.
        Initializing with a shape and fill will create a matrix of the given shape filled with the given value.
        Initializing with a 1D list will create a matrix with a single column. If you need a row vector, use shape=(1, n) with fill=[your vector].

        :param matrix: A list of lists representing the matrix (optional if shape is given)
        :param shape: A tuple of (rows, columns) representing the shape of the matrix (optional)
        :param fill: A list or a single value to fill the matrix with (optional)
        """
        if matrix is not None:
            if isinstance(matrix, Matrix):
                matrix = deepcopy(matrix._matrix)
            if not isinstance(matrix[0], list):
                # If the input is a 1D list, convert it to a column vector in a 2D list
                matrix = [[cell] for cell in matrix]
            self._matrix = deepcopy(matrix)
            self.shape = (len(matrix), len(matrix[0]))
        if shape is not None:
            if matrix is not None:
                raise ValueError(f"Cannot initialize matrix with both a shape {shape} and a matrix {matrix}. If you need to reshape the data, assign it to the fill parameter instead.")
            if isinstance(fill, (int, float)):
                fill = [float(fill)]
            elif isinstance(fill, Matrix):
                fill = fill._matrix
            flattened_fill = self._flatten(fill)
            full_length_fill = self._recycle_vector_to_length(flattened_fill, shape[0] * shape[1])
            new_matrix = self._reshape(full_length_fill, shape)
            self._matrix = new_matrix
            self.shape = shape

    @classmethod
    def new(cls, *args, **kwargs):
        return cls(*args, **kwargs)

    def reshape(self, shape: Tuple[int, int]):
        """Reshape the matrix"""
        _matrix = self._reshape(self._matrix, shape)
        return self.new(_matrix)

    def _reshape(self, matrix_or_vector: Union[list[list[float]], list[float]], shape: Tuple[int, int]) -> list[list[float]]:
        """Reshape a matrix"""
        flattened = self._flatten(matrix_or_vector)
        if shape[0] * shape[1] != len(flattened):
            raise ValueError(f"New shape {shape} must have the same number of elements as the current shape of {matrix_or_vector}")
        new_matrix = [[flattened[i*shape[1] + j] for j in range(shape[1])] for i in range(shape[0])]
        return new_matrix

    def flatten(self):
        """Flatten the matrix"""
        return self._flatten(self._matrix)

    @staticmethod
    def _flatten(matrix_or_vector: Union[list[list[float]], list[float]]) -> list[float]:
        """Flatten a matrix"""
        if isinstance(matrix_or_vector[0], list):
            if len(matrix_or_vector[0]) > 0 and isinstance(matrix_or_vector[0][0], list):
                raise ValueError(f"Can only flatten a 1D or 2D list. Got a 3D list: {matrix_or_vector}")
            return [item for sublist in matrix_or_vector for item in sublist]
        else:
            return matrix_or_vector

    @staticmethod
    def _recycle_vector_to_length(vector: list[float], length: int):
        """Recycle a vector to a given length by repeating it as many times as necessary"""
        if len(vector) == length:
            return vector
        if len(vector) > length:
            raise ValueError(f"Vector length {len(vector)} is greater than the desired length {length}")
        if length % len(vector) != 0:
            raise ValueError(f"Vector length {len(vector)} is not a multiple of the desired length {length}")
        return vector * (length // len(vector))

    @classmethod
    def identity(cls, size: int):
        """Return an identity matrix of a given size"""
        return cls([[1 if i == j else 0 for j in range(size)] for i in range(size)])

    def transpose(self):
        """Return the transpose of the matrix"""
        return self.new([[self._matrix[j][i] for j in range(self.shape[0])] for i in range(self.shape[1])])

    @property
    def T(self):
        """Return the transpose of the matrix"""
        return self.transpose()

    def extend_right(self, other):
        other = self.new(other)
        if self.shape[0] != other.shape[0]:
            raise ValueError(f"Cannot combine matrices of shapes {self.shape} and {other.shape} horizontally")
        new_matrix = deepcopy(self._matrix)
        for i in range(self.shape[0]):
            new_matrix[i].extend(other._matrix[i])
        return self.new(new_matrix)

    def extend_bottom(self, other):
        other = self.new(other)
        if self.shape[1] != other.shape[1]:
            raise ValueError(f"Cannot combine matrices of shapes {self.shape} and {other.shape} vertically")
        new_matrix = deepcopy(self._matrix)
        new_matrix.extend(other._matrix)
        return self.new(new_matrix)

    def swap_rows(self, i: int, j: int):
        """Swap two rows of the matrix"""
        self._matrix[i], self._matrix[j] = self._matrix[j], self._matrix[i]
        return self.new(deepcopy(self._matrix))

    def swap_cols(self, i: int, j: int):
        """Swap two columns of the matrix"""
        for row in self._matrix:
            row[i], row[j] = row[j], row[i]
        return self.new(deepcopy(self._matrix))

    def rows(self, from_index: int, to_index: Optional[int] = None):
        """Return a new matrix with a subset of the rows"""
        if to_index is None:
            to_index = from_index + 1
        return self.new(self._matrix[from_index:to_index])

    def cols(self, from_index: int, to_index: Optional[int] = None):
        """Return a new matrix with a subset of the columns"""
        if to_index is None:
            to_index = from_index + 1
        return self.new([[row[i] for i in range(from_index, to_index)] for row in self._matrix])

    # def gauss_jordan_elimination(self, augmentation: Optional[Union[list[list[float]], list[float], Matrix]] = None):
    #     """Perform Gauss-Jordan elimination on the matrix"""
    #     matrix = self.new(self)
    #     if augmentation is not None:
    #         augmentation = augmentation if isinstance(augmentation, self.new) else self.new(augmentation)
    #         if augmentation.shape[0] != matrix.shape[0]:
    #             raise ValueError(f"Cannot perform Gauss-Jordan elimination between coefficient matrix of shape {self.shape} with an augmentation matrix/vector of shape {augmentation.shape}")
    #         matrix = matrix.extend_right(augmentation)
    #         # print(matrix)
    #     for i in range(matrix.shape[0]):
    #         # If the pivot element is 0, swap the row with a lower row
    #         if matrix[i][i] == 0:
    #             for j in range(i+1, matrix.shape[0]):
    #                 if matrix[j][i] != 0:
    #                     matrix = matrix.swap_rows(i, j)
    #                     break
    #         # If the pivot element is still 0, skip this row as it's already eliminated
    #         if matrix[i][i] == 0:
    #             continue
    #         # If the pivot element is not 0, divide its own row by the diagonal element to make it 1
    #         matrix[i] = [cell / matrix[i][i] for cell in matrix[i]]
    #         # Eliminate the other rows
    #         for j in range(matrix.shape[0]):
    #             # Skip the pivot row
    #             if j != i:
    #                 # Subtract the diagonal row multiplied by the element in the current row
    #                 matrix[j] = [matrix[j][k] - matrix[i][k] * matrix[j][i] for k in range(matrix.shape[1])]
    #     # Note: if the first column is all 0s, the matrix is singular, which means it has no inverse
    #     # print(f"{self.shape=}, {augmentation.shape=}, {matrix.shape=}")
    #     # print(f"result:\n{matrix}")
    #     coefficients = matrix.cols(0, self.shape[1])
    #     aug_result = matrix.cols(self.shape[1], matrix.shape[1])
    #     # print(f"{coefficients=}\n{aug_result=}")
    #     return coefficients, aug_result

    def gauss_elimination(self):
        """Perform Gauss elimination on the matrix and return the reduced row echelon form, as well as the pivots."""
        matrix = self.new(self)  # creates a copy
        curr_row = 0
        curr_col = 0
        pivots = []
        while curr_row < self.shape[0] and curr_col < self.shape[1]:
            # https://de.wikipedia.org/wiki/Gau%C3%9F-Jordan-Algorithmus
            # Man wählt die erste Spalte von links, in der mindestens ein von Null verschiedener Wert steht.
            if all(cell == 0 for cell in matrix.cols(curr_col, curr_col + 1)):
                curr_col += 1
                continue
            # We're now at a pivot element
            # Ist die oberste Zahl der gewählten Spalte eine Null, so vertauscht man die erste Zeile mit einer anderen Zeile, in der in dieser Spalte keine Null steht.
            if matrix[curr_row][curr_col] == 0:
                found_nonzero_row = False
                for i in range(curr_row + 1, self.shape[0]):
                    if matrix[i][curr_col] != 0:
                        matrix = matrix.swap_rows(curr_row, i)
                        found_nonzero_row = True
                        break
                if not found_nonzero_row:
                    curr_col += 1
                    continue
            pivots.append(curr_col)
            # print(f"curr_row={curr_row}, curr_col={curr_col}, {matrix[curr_row][curr_col]=}")
            # Man dividiert die erste Zeile durch das nun oberste Element der gewählten Spalte.
            matrix[curr_row] = [cell / matrix[curr_row][curr_col] for cell in matrix[curr_row]]
            # Man subtrahiert entsprechende Vielfache der ersten Zeile von den darunterliegenden Zeilen mit dem Ziel, dass das erste Element jeder Zeile (außer der ersten) Null wird.
            for i in range(curr_row + 1, self.shape[0]):
                matrix[i] = [matrix[i][j] - matrix[curr_row][j] * matrix[i][curr_col] for j in range(self.shape[1])]
            # Durch Streichen der ersten Zeile und Spalte erhält man eine Restmatrix, auf die man diese Schritte wieder anwendet. Das führt man solange durch, bis die Matrix in Zeilenstufenform ist.
            curr_row += 1
            curr_col += 1
        reduced_row_echelon_form = matrix
        return reduced_row_echelon_form, pivots

    def gauss_jordan_elimination(self, augmentation: Optional[Union[list[list[float]], list[float], Matrix]] = None):
        """Perform Gauss-Jordan elimination on the matrix"""
        matrix = self.new(self)
        if augmentation is not None:
            augmentation = augmentation if isinstance(augmentation, Matrix) else self.new(augmentation)
            if augmentation.shape[0] != matrix.shape[0]:
                raise ValueError(f"Cannot perform Gauss-Jordan elimination between coefficient matrix of shape {self.shape} with an augmentation matrix/vector of shape {augmentation.shape}. The number of rows must be equal.")
            matrix = matrix.extend_right(augmentation)
            # print(matrix)
        rref, pivots = matrix.gauss_elimination()
        # Man zieht danach von den darüberliegenden Zeilen entsprechende Vielfache ab, sodass über einer führenden 1 nur Nullen stehen.
        for pivot_row in range(len(pivots) - 1, 0, -1):
            pivot_col = pivots[pivot_row]
            for row_above in range(pivot_row - 1, -1, -1):
                val_above_pivot = rref[row_above][pivot_col]
                new_row_above = []
                for row in range(rref.shape[1]):
                    val_to_subtract = rref[pivot_row][row] * val_above_pivot
                    new_row_above.append(rref[row_above][row] - val_to_subtract)
                rref[row_above] = new_row_above
        coeff = rref.cols(0, self.shape[1])
        aug_res = rref.cols(self.shape[1], rref.shape[1])
        # print(f"{coefficients=}\n{aug_result=}")
        return coeff, aug_res

    def minor(self, i: int, j: int):
        """Return the minor of the matrix with the ith row and jth column removed"""
        return self.new([row[:j] + row[j+1:] for row in (self._matrix[:i] + self._matrix[i+1:])])

    def determinant(self, mul=1):
        """Calculate the determinant of the matrix"""
        # Base case of recursion: matrix is 1x1
        if self.shape[0] == 1 and self.shape[1] == 1:
            return mul * self[0][0]
        # Recursive case: expand along the first row
        return sum(
            # Note that minor gets us the submatrix of self without the first row and with the i-th column removed
            mul * self[0][i] * self.minor(0, i).determinant(-1 if i % 2 == 1 else 1)
            for i in range(self.shape[1])
        )

    def solve(self, b: Union[Matrix, list[float], list[list[float]]]):
        """Solve the system of linear equations using gauss-jordan elimination and return the result."""
        b = self.new(b)
        if self.shape[0] != self.shape[1]:
            raise ValueError(f"Cannot solve a non-square matrix of shape {self.shape}")
        if self.determinant() == 0:
            raise ValueError(f"Cannot solve a singular matrix")
        return self.gauss_jordan_elimination(b)[1]

    def inverse(self):
        """Return the inverse of the matrix"""
        return self.solve(self.identity(self.shape[0]))

    @staticmethod
    def safe_div(a: float, b: float, replace_with=0) -> float:
        """Safely divide two numbers, returning val (default 0) if the denominator is 0"""
        # Todo maybe use float('inf') or float('nan') instead
        return a / b if b != 0 else replace_with

    def __mul__(self, other: Union[Matrix, float, int, list]):
        """Multiply two matrices of compatible shapes.
        The number of columns in the first matrix must be equal to the number of rows in the second matrix.

        If the matrices are 1D, the dot product is calculated.
        """
        if isinstance(other, (int, float)):
            result = [[self._matrix[i][j] * other for j in range(self.shape[1])] for i in range(self.shape[0])]
            return self.new(result)
        if isinstance(other, list):
            other = self.new(other)
        if self.shape[1] != other.shape[0]:
            raise ValueError(f"Cannot multiply matrices of shapes {self.shape} and {other.shape}")
        result = [[0 for _ in range(other.shape[1])] for _ in range(self.shape[0])]
        for i in range(self.shape[0]):
            for j in range(other.shape[1]):
                for k in range(self.shape[1]):
                    result[i][j] += self._matrix[i][k] * other._matrix[k][j]
        return self.new(result)

    def __rmul__(self, other: Union[Matrix, float, int, list]):
        """Multiply two matrices of compatible shapes"""
        if isinstance(other, (int, float)):
            return self.__mul__(other)
        elif isinstance(other, list):
            other = self.new(other)
        return other.__mul__(self)

    def __truediv__(self, other: Union[Matrix, float, int, list]):
        """Divide two matrices of same shapes"""
        if isinstance(other, (int, float)):
            result = [[self.safe_div(self._matrix[i][j], other) for j in range(self.shape[1])] for i in range(self.shape[0])]
            return self.new(result)
        if isinstance(other, list):
            other = self.new(other)
        if self.shape != other.shape:
            raise ValueError(f"Cannot divide matrices of shapes {self.shape} and {other.shape}")
        result = [[self._matrix[i][j] / other._matrix[i][j] for j in range(self.shape[1])] for i in range(self.shape[0])]
        return self.new(result)

    def __rtruediv__(self, other: Union[Matrix, float, int, list]):
        """Divide two matrices of same shapes"""
        if isinstance(other, (int, float)):
            result = [[self.safe_div(other, self._matrix[i][j]) for j in range(self.shape[1])] for i in range(self.shape[0])]
            return self.new(result)
        if isinstance(other, list):
            other = self.new(other)
        return other.__truediv__(self)

    def __add__(self, other: Union[Matrix, float, int, list]):
        """Add two matrices of the same shape"""
        if isinstance(other, (int, float)):
            result = [[self._matrix[i][j] + other for j in range(self.shape[1])] for i in range(self.shape[0])]
            return self.new(result)
        if isinstance(other, list):
            other = self.new(other)
        if self.shape != other.shape:
            raise ValueError(f"Cannot add matrices of shapes {self.shape} and {other.shape}")
        result = [[self._matrix[i][j] + other._matrix[i][j] for j in range(self.shape[1])] for i in range(self.shape[0])]
        return self.new(result)

    def __radd__(self, other):
        return self.__add__(other)

    def __neg__(self):
        """Negate the matrix"""
        return self.__mul__(-1)

    def __sub__(self, other: Union[Matrix, float, int, list]):
        """Subtract two matrices of the same shape"""
        if isinstance(other, list):
            other = self.new(other)
        return self.__add__(-other)

    def __rsub__(self, other: Union[Matrix, float, int, list]):
        """Subtract two matrices of the same shape"""
        return (-self).__add__(other)

    def __getitem__(self, idx: int):
        return self._matrix[idx]

    def __setitem__(self, idx: int, value: list[float]):
        self._matrix[idx] = value

    def __iter__(self):
        flat = self._flatten(self._matrix)
        return iter(flat)

    def __len__(self):
        return len(self._flatten(self._matrix))

    def __str__(self):
        # pretty print the 4x4 matrix
        return "\n".join(" ".join(f"{cell:.2f}" for cell in row) for row in self._matrix)

    def __repr__(self):
        return f"Matrix({str(self._matrix)})"

    def __eq__(self, other: Union[Matrix, list[list[float]], list[float], float, int]):
        """Check if two matrices are equal"""
        if isinstance(other, Matrix):
            return self._matrix == other._matrix
        elif isinstance(other, list):
            other_matrix = self.new(other)
            return self._matrix == other_matrix._matrix
        elif isinstance(other, (float, int)) and self.shape == (1, 1):
            return self._matrix[0][0] == other
        else:
            raise ValueError(f"Cannot compare matrix of shape {self.shape} with {other} of type {type(other)}")

def test_matrix_class():
    # Test initialization variants
    init_m1 = Matrix([[1, 2], [3, 4]])
    print(f"{init_m1=}, should be [[1, 2], [3, 4]]")
    assert init_m1 == [[1, 2], [3, 4]]

    init_m2 = Matrix(shape=(2, 2), fill=0)
    print(f"{init_m2=}, should be [[0, 0], [0, 0]]")
    assert init_m2 == [[0, 0], [0, 0]]

    init_m3 = Matrix(shape=(2, 3), fill=[1, 2, 3, 4, 5, 6])
    print(f"{init_m3=}, should be [[1, 2, 3], [4, 5, 6]]")
    assert init_m3 == [[1, 2, 3], [4, 5, 6]]

    print(f"Initialization should refuse filling (2, 3) shape with 6 elements")
    try:
        init_m3 = Matrix(shape=(2, 3), fill=[1, 2, 3, 4, 5])
        raise ValueError(f"Expected ValueError for mismatched shape and fill length, got {init_m3}")
    except ValueError as e:
        print(f"  Caught expected exception: {e}")

    init_m4 = Matrix(shape=(2, 3), fill=[[1, 2, 3], [4, 5, 6]])
    print(f"{init_m4=}, should be [[1, 2, 3], [4, 5, 6]]")
    assert init_m4 == [[1, 2, 3], [4, 5, 6]]

    init_m5 = Matrix(shape=(2, 1), fill=1)
    print(f"{init_m5=}, should be [[1], [1]]")
    assert init_m5 == [[1], [1]]

    init_m6 = Matrix([1, 2, 3])  # Should become a column vector
    print(f"{init_m6=}, should be [[1], [2], [3]]")
    assert init_m6 == [[1], [2], [3]]

    # Test reshape
    reshape_m1 = Matrix([[1, 2], [3, 4]])
    print(f"{reshape_m1.reshape((1, 4))=}, should be [[1, 2, 3, 4]]")
    assert reshape_m1.reshape((1, 4)) == [[1, 2, 3, 4]]

    # Test scalar operations
    scalar_m1 = Matrix([[1, 2], [3, 4]])
    print(f"{scalar_m1 * 2=}, should be [[2, 4], [6, 8]]")
    assert scalar_m1 * 2 == [[2, 4], [6, 8]]

    print(f"{2 * scalar_m1=}, should be [[2, 4], [6, 8]]")
    assert 2 * scalar_m1 == [[2, 4], [6, 8]]

    print(f"{scalar_m1 + 2=}, should be [[3, 4], [5, 6]]")
    assert scalar_m1 + 2 == [[3, 4], [5, 6]]

    print(f"{scalar_m1 - 2=}, should be [[-1, 0], [1, 2]]")
    assert scalar_m1 - 2 == [[-1, 0], [1, 2]]

    print(f"{2 - scalar_m1=}, should be [[1, 0], [-1, -2]]")
    assert 2 - scalar_m1 == [[1, 0], [-1, -2]]

    # Test transpose
    transpose_m1 = Matrix([[1, 2], [3, 4]])
    print(f"{transpose_m1.T=}, should be [[1, 3], [2, 4]]")
    assert transpose_m1.T == [[1, 3], [2, 4]]

    # Test identity matrix
    identity_m1 = Matrix.identity(3)
    print(f"{identity_m1=}, should be [[1, 0, 0], [0, 1, 0], [0, 0, 1]]")
    assert identity_m1 == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    # Test matrix multiplication
    matmul_m1 = Matrix([[1, 2], [3, 4]])
    matmul_m2 = Matrix([[5, 6], [7, 8]])
    print(f"{matmul_m1 * matmul_m2=}, should be [[19, 22], [43, 50]]")
    assert matmul_m1 * matmul_m2 == [[19, 22], [43, 50]]

    print(f"{matmul_m1 * Matrix.identity(2)=}, should be [[1, 2], [3, 4]]")
    assert matmul_m1 * Matrix.identity(2) == [[1, 2], [3, 4]]

    # Incompatible shapes
    print(f"matmul_m1 * [1, 2, 3], should raise ValueError")
    try:
        print(f"{matmul_m1 * [[1, 2, 3], [4, 5, 6]]=}, should raise ValueError")
        raise ValueError(f"Expected ValueError for incompatible shapes")
    except ValueError as e:
        print(f"  Caught expected exception: {e}")

    # Non-square matrix multiplication
    matmul_m3 = Matrix([[1, 2, 3], [4, 5, 6]])
    matmul_m4 = [[7, 8], [9, 10], [11, 12]]
    print(f"{matmul_m3 * matmul_m4=}, should be [[58, 64], [139, 154]]")
    assert matmul_m3 * matmul_m4 == [[58, 64], [139, 154]]

    # Order of multiplication matters even for lists
    print(f"{matmul_m4 * matmul_m3=}, should be [[39, 54, 69], [49, 68, 87], [59, 82, 105]]")
    assert matmul_m4 * matmul_m3 == [[39, 54, 69], [49, 68, 87], [59, 82, 105]]

    # Dot product of two vectors
    dot_m1 = Matrix([[1, 2, 3]])
    dot_m2 = Matrix([[4], [5], [6]])
    print(f"{dot_m1 * dot_m2=}, should be [[32]]")
    assert dot_m1 * dot_m2 == [[32]]

    print(f"Testing index access")
    assert (dot_m1 * dot_m2)[0][0] == 32

    # Test addition
    add_m1 = Matrix([[1, 2], [3, 4]])
    add_m2 = Matrix([[5, 6], [7, 8]])
    print(f"{add_m1 + add_m2=}, should be [[6, 8], [10, 12]]")
    assert add_m1 + add_m2 == [[6, 8], [10, 12]]

    # Test subtraction
    sub_m1 = Matrix([[1, 2], [3, 4]])
    sub_m2 = [[5, 6], [7, 8]]
    print(f"{sub_m1 - sub_m2=}, should be [[-4, -4], [-4, -4]]")
    assert sub_m1 - sub_m2 == [[-4, -4], [-4, -4]]

    print(f"{sub_m2 - sub_m1=}, should be [[4, 4], [4, 4]]")
    assert sub_m2 - sub_m1 == [[4, 4], [4, 4]]

    # Test division
    div_m1 = Matrix([[1, 2], [3, 4]])
    div_m2 = Matrix([[2, 4], [6, 8]])
    print(f"{div_m1 / div_m2=}, should be [[0.5, 0.5], [0.5, 0.5]]")
    assert div_m1 / div_m2 == [[0.5, 0.5], [0.5, 0.5]]

    # Test Gauss-Jordan elimination
    AXES_ANGLE_RATIO = [
        [13.6, 0, 0, 0, 0, 0],
        [0, -144, 0, 0, 0, 0],
        [0, 0, 114, 119.65, 0, 0],
        [0, 0, 0, 5, 0, 0],
        [0, 0, 0, 0, 9.3, -11.625],
        [0, 0, 0, 0, -9.3, -11.625]
    ]
    AXES_ANGLE_RATIO_INV = [
       [ 0.07352941,  0.        ,  0.        ,  0.        ,  0.        ,0.        ],
       [-0.        , -0.00694444, -0.        , -0.        , -0.        ,-0.        ],
       [ 0.        ,  0.        ,  0.00877193, -0.20991228,  0.        ,0.        ],
       [ 0.        ,  0.        ,  0.        ,  0.2       ,  0.        ,0.        ],
       [ 0.        ,  0.        ,  0.        ,  0.        ,  0.05376344,-0.05376344],
       [-0.        , -0.        , -0.        , -0.        , -0.04301075,-0.04301075]]
    elimination_m1 = Matrix(AXES_ANGLE_RATIO)
    elimination_m2 = Matrix(AXES_ANGLE_RATIO_INV)
    augmentation = Matrix.identity(6)
    coeff, res = elimination_m1.gauss_jordan_elimination(augmentation=augmentation)
    # print(f"{coeff=}, {res=}")
    # print(f"{res=}, should be {elimination_m2}")
    error = sum((res - elimination_m2)*(res - elimination_m2))
    print(f"Matrix inversion: {error=}, should be near 0")
    assert error < 0.0000001

    # Gauss-jordan elimination should work with a singular matrix and return the reduced echelon form
    singular_m1 = Matrix([[0, 1, 2], [0, 2, 4]])
    coeff, res = singular_m1.gauss_jordan_elimination(augmentation=Matrix.identity(2))
    # print(f"{coeff=}, {res=}")
    print(f"{coeff=}, should be [[0, 1, 2], [0, 0, 0]]")
    assert coeff == [[0, 1, 2], [0, 0, 0]]

    # Test determinant
    det_m1 = Matrix([[1, 2], [3, 4]])
    print(f"{det_m1.determinant()=}, should be -2")
    assert det_m1.determinant() == -2

    # Test solve
    solve_m1 = Matrix([[1, 2], [3, 4]])
    solve_b1 = Matrix([[5], [11]])
    print(f"{solve_m1.solve(solve_b1)=}, should be [[1], [2]]")
    assert solve_m1.solve(solve_b1) == [[1], [2]]

    # Test inverse
    inv_m1 = Matrix([[1, 2], [3, 4]])
    print(f"{inv_m1.inverse()=}, should be [[-2, 1], [1.5, -0.5]]")
    assert inv_m1.inverse() == [[-2, 1], [1.5, -0.5]]



# test_matrix_class();exit(0)

class DHMatrix(Matrix):
    """An applied 4x4 Denavit-Hartenberg matrix that stores the transformations for a particular joint angle.
    A series of multiplied matrices is used to calculate the position and orientation of the end effector of
    a robotic arm.

    :param matrix: A 4x4 matrix (list of lists) representing the transformation matrix
    """
    def __init__(self, matrix: list[list[float]] = None):
        """Initialize the matrix.
        :param matrix: A 4x4 matrix (list of lists) representing the transformation matrix (default: identity matrix)
        """
        if matrix is None:
                matrix = [[1 if i == j else 0 for j in range(4)] for i in range(4)]
        elif len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise ValueError(f"Matrix must be a 4x4 list of lists. Got {len(matrix)}x{len(matrix[0])}")
        elif any(not isinstance(cell, (int, float)) for row in matrix for cell in row):
            raise ValueError(f"Matrix must contain only numbers: {matrix}")
        elif matrix[3] != [0, 0, 0, 1]:
            raise ValueError(f"The bottom row of the matrix must be [0, 0, 0, 1]. It is {matrix[3]}")
        super().__init__(matrix)

    # def __mul__(self, other: DHMatrix):
    #     """Multiply two 4x4 matrices"""
    #     result = [[0 for _ in range(4)] for _ in range(4)]
    #     for i in range(4):
    #         for j in range(4):
    #             for k in range(4):
    #                 result[i][j] += self._matrix[i][k] * other._matrix[k][j]
    #     return DHMatrix(result)

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
        """Return the rotation angle around the x-joint in radians"""
        return atan2(self._matrix[2][1], self._matrix[2][2])

    @property
    def beta(self):
        """Return the rotation angle around the y-joint in radians"""
        return atan2(-self._matrix[2][0], sqrt(self._matrix[2][1]**2 + self._matrix[2][2]**2))

    @property
    def gamma(self):
        """Return the rotation angle around the z-joint in radians"""
        return atan2(self._matrix[1][0], self._matrix[0][0])

    @property
    def euler_angles(self):
        """Return the rotation angles around the x, y, and z axes in radians"""
        return self.alpha, self.beta, self.gamma

    @property
    def alpha_deg(self):
        """Return the rotation angle around the x-joint in degrees"""
        return self.alpha * 180 / pi

    @property
    def beta_deg(self):
        """Return the rotation angle around the y-joint in degrees"""
        return self.beta * 180 / pi

    @property
    def gamma_deg(self):
        """Return the rotation angle around the z-joint in degrees"""
        return self.gamma * 180 / pi

    @property
    def euler_angles_deg(self):
        """Return the rotation angles around the x, y, and z axes in degrees"""
        return [self.alpha_deg, self.beta_deg, self.gamma_deg]

    # def transpose(self):
    #     """Return the transpose of the matrix"""
    #     return DHMatrix([[self._matrix[j][i] for j in range(4)] for i in range(4)])
    #
    # def __getitem__(self, item):
    #     return self._matrix[item]
    #
    # def __str__(self):
    #     # pretty print the 4x4 matrix
    #     return "\n".join(" ".join(f"{cell:.2f}" for cell in row) for row in self._matrix)

    def __repr__(self):
        return f"DHMatrix({str(self._matrix)})"


class Link:
    def __init__(self, alpha, a, theta_offset, d, lower_bound=None, upper_bound=None, use_degrees=False, convention=MODIFIED, fixed=False):
        """Prepare a 4x4 Denavit-Hartenberg matrix for a single link
        :param a: The distance from the z-joint of the previous link to the common normal
        :param d: The distance from the x-joint of the previous link to the common normal
        :param alpha: The angle from the z-joint of the previous link to the common normal
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
            self.fixed_matrix = DHMatrix(self._rot_z())

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
        return DHMatrix(self._rot_z(theta))

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

    def forward_kinematics(self, thetas: list[float], link_indices=None, fix_bounds=True) -> DHMatrix:
        """Calculate the forward kinematics for a given set of link angles"""
        links = link_indices if link_indices is not None else self.links
        full_thetas = self._get_thetas_with_fixed_links(thetas, links)
        result = DHMatrix()
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
        result = DHMatrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
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
        result *= DHMatrix()  # end effector (identity matrix)
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
    #         result = DHMatrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
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
#         result = DHMatrix([[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]])
#         for link, theta in zip(self.links, thetas):
#             result *= link(theta, fix_bounds)
#             x, y, z = result.translation_vector
#             r = result.rotation_matrix
#             for i in range(3):
#                 ax.plot([x, x + r[i][0]], [y, y + r[i][1]], [z, z + r[i][2]])
#         plt.show()
#         return "3D plot displayed"
