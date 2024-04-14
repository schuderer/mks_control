from __future__ import annotations

from copy import deepcopy
from typing import Optional, Tuple, Union


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
