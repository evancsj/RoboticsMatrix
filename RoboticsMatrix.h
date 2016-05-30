class RoboticsMatrix{
public:
    RoboticsMatrix(int _rank, float* _array);
    ~RoboticsMatrix();

    void print();   //print the matrix

    float determinant();        //get the determinant of the matrix

    RoboticsMatrix transpose();     //get the transpose of the matrix

    RoboticsMatrix inverse();       //get the inverse of the matrix

    float pitch();      //get the pitch angle of rotation matrix

    float yaw();        //get the yaw angle of rotation matrix

    float roll();       //get the roll angle of rotation matrix

    RoboticsMatrix operator *(RoboticsMatrix other);     //Matrix multiplication

    RoboticsMatrix operator +(RoboticsMatrix other);     //Matrix addition

    RoboticsMatrix operator -(RoboticsMatrix other);     //Matrix subtraction

    int rank;
    float* array;
private:
    float determinant(int rank,float* array);
};
