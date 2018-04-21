#include <vector>
using namespace std;


class Point
{
	public:
	Point(unsigned int NUM,float THETA, float RHO);
	Point(float X, float Y);
	float theta, rho;
	unsigned int num;
	float x();
	float y();
	float operator-(const Point& rhs);
};

class Segment
{
	public:
	Segment(const vector<Point>& PTS);
	vector<Point> pts;
	vector<float> centerPt();
	float var(bool min);
	float length();
};