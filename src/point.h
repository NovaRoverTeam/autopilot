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
	float operator/(const Point& rhs);
};

