#include "RoboticsMatrix.h"
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

RoboticsMatrix::RoboticsMatrix(int _rank, float* _array){
    rank=_rank;
    array=new float[rank*rank];
    array=_array;
}

RoboticsMatrix::~RoboticsMatrix(){}

void RoboticsMatrix::print(){
    for(int i=0;i<rank;i++){
        for(int j=0;j<rank;j++)
            cout<<setprecision(3)<<fixed<<setw(10)<<*(array+i*rank+j);
        cout<<endl;
    }
}

float RoboticsMatrix::determinant(){
    return determinant(rank,array);
}

RoboticsMatrix RoboticsMatrix::transpose(){
    float* trans_array;
    trans_array=new float[rank*rank];
    for(int i=0;i<rank;i++)
        for(int j=0;j<rank;j++)
            *(trans_array+j*rank+i)=*(array+i*rank+j);
    return RoboticsMatrix(rank,trans_array);
}

RoboticsMatrix RoboticsMatrix::inverse(){
    float* inv_array;
    inv_array=new float[rank*rank];
    float det=determinant();
    for(int i=0;i<rank;i++)
        for(int j=0;j<rank;j++){
            float *temp;
            int num=0;
            temp = new float[(rank-1)*(rank-1)];
            for(int m=0;m<rank;m++)
                for(int n=0;n<rank;n++){
                    if(m!=i && n!=j)
                        *(temp+(num++))=*(array+m*rank+n);
                }
            *(inv_array+j*rank+i)=pow(-1,i+j)*determinant(rank-1,temp)/det;
            delete[] temp;
        }
    return RoboticsMatrix(rank,inv_array);
}

//reference to robotics, rotation matrix to zyx euler angle
//pitch angle, rotate around y axis, [-pi/2,pi/2]
float RoboticsMatrix::pitch(){
    float r31;
    r31=*(array+2*rank);
    float sp=-r31;                  //sin(pitch)
    return asin(sp);
}

//yaw angle, rotate around z axis, [-pi,pi]
float RoboticsMatrix::yaw(){
    float r11,r21;
    r11=*array;
    r21=*(array+rank);
    float p=pitch();
    float cy=r11/cos(p);      //cos(yaw)
    float sy=r21/cos(p);      //sin(yaw)
    float y=acos(cy);       //yaw angle
    if(sy<0)
        y=-y;
    return y;
}

//roll angle, rotate around x axis, [-pi,pi]
float RoboticsMatrix::roll(){
    float r32,r33;
    r32=*(array+2*rank+1);
    r33=*(array+2*rank+2);
    float p=pitch();
    float sr=r32/cos(p);        //sin(roll)
    float cr=r33/cos(p);        //cos(roll)
    float r=acos(cr);       //roll angle
    if(sr<0)
        r=-r;
    return r;
}

RoboticsMatrix RoboticsMatrix::operator *(RoboticsMatrix other){
    float* multiplication_array;
    multiplication_array=new float[rank*rank];
    for(int i=0;i<rank;i++)
        for(int j=0;j<rank;j++){
            *(multiplication_array+i*rank+j)=0;
            for(int m=0;m<rank;m++)
                *(multiplication_array+i*rank+j)=*(multiplication_array+i*rank+j)+*(array+i*rank+m)**(other.array+m*rank+j);
        }
    return RoboticsMatrix(rank,multiplication_array);
}

RoboticsMatrix RoboticsMatrix::operator +(RoboticsMatrix other){
    float* addition_array;
    addition_array=new float[rank*rank];
    for(int i=0;i<rank*rank;i++)
        *(addition_array+i)=*(array+i)+*(other.array+i);
    return RoboticsMatrix(rank,addition_array);
}

RoboticsMatrix RoboticsMatrix::operator -(RoboticsMatrix other){
    float* subtraction_array;
    subtraction_array=new float[rank*rank];
    for(int i=0;i<rank*rank;i++)
        *(subtraction_array+i)=*(array+i)-*(other.array+i);
    return RoboticsMatrix(rank,subtraction_array);
}

float RoboticsMatrix::determinant(int _rank,float* _array){
    float det=0;
    if(_rank==1)
        return *_array;
    else
        for(int i=0;i<_rank;i++){
            float *temp;
            temp = new float[(_rank-1)*(_rank-1)];
            int num=0;
            for(int m=1;m<_rank;m++)
                for(int n=0;n<_rank;n++)
                    if(n!=i)
                        *(temp+(num++))=*(_array+m*_rank+n);
            det=det+*(_array+i)*pow(-1,i)*determinant(_rank-1,temp);
            delete[] temp;
        }
    return det;
}
