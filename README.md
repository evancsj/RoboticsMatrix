# RoboticsMatrix
This is used for matrix calculation in robotics. Of course, in robotics, it only relates 3X3 or 4X4 matrix, but some functions in RoboticsMatrix support all nXn matrix.

RoboticsMatrix(int _rank,float* _array)
--
_rank = the order of the matrix

*_array =  the point of the matrix element array

determinant()
--
mat.determinant() = the determinant of the matrix mat

It supports all nXn matrix.

transpose()
--
mat.transpose() = the transpose matrix of mat

It supports all nXn matrix.

inverse()
--
mat.inverse() = the inverse matrix of mat

It supports all nXn matrix who's rank is n.

pitch(),yaw(),roll()
--
mat.pitch(), mat.yaw(), mat.roll() = the z-y-x Euler angle for mat.

It just supports the rotation matrix.

operator * - +
--
mat*Mat, mat+Mat, mat-Mat
