#include "point.h"
#include <math.h>

Point::Point(unsigned int NUM,float THETA, float RHO){
	num=NUM;
	theta=THETA;
	rho=RHO;
}
Point::Point(float X, float Y){
	num=0;
	theta=atan2(Y,X);
	rho=sqrt(X*X+Y*Y);
}
float Point::x(){
	return rho*cos(theta);
}
float Point::y(){
	return rho*sin(theta);
}
float Point::operator-(const Point& rhs){
    return sqrt((this->rho)*(this->rho) + (rhs.rho)*(rhs.rho) - 2*(this->rho)*(rhs.rho)*cos(this->theta - rhs.theta));
}
float Point::operator/(const Point& rhs){
    return atan2((this->rho)*sin(this->theta) - (rhs.rho)*sin(rhs.theta), ((this->rho)*cos(this->theta) - (rhs.rho)*cos(rhs.theta)));
}
