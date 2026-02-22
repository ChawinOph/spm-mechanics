#ifndef _MVECT_H_
#define _MVECT_H_

#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
using namespace std;

#ifndef MV_POINT_ELEM_ZERO
#define MV_POINT_ELEM_ZERO		1e-6
#endif


template <class SCALAR, class SCALAR2>
class mArray2 
{
public:
	SCALAR x;
	SCALAR2 y;

	mArray2( )
	{
		memset(this, 0, sizeof(*this));
	}

	mArray2(SCALAR _x, SCALAR2 _y)
		: x(_x), y(_y) { }

	friend ostream& operator << (ostream& os, mArray2<SCALAR, SCALAR2>& m) 
	{
		return os << m.x << " " << m.y;
	}

	friend istream& operator >> (istream& is, mArray2<SCALAR, SCALAR2>& m) 
	{
		return is >> m.x >> m.y;
	}

	// unary -
	mArray2<SCALAR, SCALAR2> operator - ( ) const 
	{
		return mArray2<SCALAR, SCALAR2>(-x, -y);
	}

	// vector/vector arithmatic
	mArray2<SCALAR, SCALAR2> operator + (mArray2<SCALAR, SCALAR2>& v) const 
	{
		return mArray2<SCALAR, SCALAR2>(x+v.x, y+v.y);
	}
	mArray2<SCALAR, SCALAR2> operator - (mArray2<SCALAR, SCALAR2>& v) const 
	{
		return mArray2<SCALAR, SCALAR2>(x-v.x, y-v.y);
	}
	mArray2<SCALAR, SCALAR2> operator += (mArray2<SCALAR, SCALAR2>& v) 
	{
		return (*this) = (*this) + v;
	}
	mArray2<SCALAR, SCALAR2> operator -= (mArray2<SCALAR, SCALAR2>& v) 
	{
		return (*this) = (*this) - v;
	}
	bool operator == (mArray2<SCALAR, SCALAR2>& v) 
	{
		return (x == v.x && y == v.y);
	}
};

template <class SCALAR, class SCALAR2, class SCALAR3>
class mArray3 
{
public:
	SCALAR x;
	SCALAR2 y;
	SCALAR3 z;

	mArray3( )
	{
		memset(this, 0, sizeof(*this));
	}

	mArray3(SCALAR _x, SCALAR2 _y, SCALAR3 _z)
		: x(_x), y(_y), z(_z) { }

	friend ostream& operator << (ostream& os, mArray3<SCALAR, SCALAR2, SCALAR3>& m) 
	{
		return os << m.x << " " << m.y << " " << m.z;
	}

	friend istream& operator >> (istream& is, mArray3<SCALAR, SCALAR2, SCALAR3>& m) 
	{
		return is >> m.x >> m.y >> m.z;
	}
	// unary -
	mArray3<SCALAR, SCALAR2, SCALAR3> operator - ( ) const 
	{
		return mArray3<SCALAR, SCALAR2, SCALAR3>(-x, -y, -z);
	}

	// vector/vector arithmatic
	mArray3<SCALAR, SCALAR2, SCALAR3> operator + (mArray3<SCALAR, SCALAR2, SCALAR3>& v) const 
	{
		return mArray3<SCALAR, SCALAR2, SCALAR3>(x+v.x, y+v.y, z+v.z);
	}
	mArray3<SCALAR, SCALAR2, SCALAR3> operator - (mArray3<SCALAR, SCALAR2, SCALAR3>& v) const 
	{
		return mArray3<SCALAR, SCALAR2, SCALAR3>(x-v.x, y-v.y, z-v.z);
	}
	mArray3<SCALAR, SCALAR2, SCALAR3> operator += (mArray3<SCALAR, SCALAR2, SCALAR3>& v) 
	{
		return (*this) = (*this) + v;
	}
	mArray3<SCALAR, SCALAR2, SCALAR3> operator -= (mArray3<SCALAR, SCALAR2, SCALAR3>& v) 
	{
		return (*this) = (*this) - v;
	}
	bool operator == (mArray3<SCALAR, SCALAR2, SCALAR3>& v) 
	{
		return (x == v.x && y == v.y && z == v.z);
	}
};

template <class SCALAR, class SCALAR2, class SCALAR3, class SCALAR4>
class mArray4 
{
public:
	SCALAR x;
	SCALAR2 y;
	SCALAR3 z;
	SCALAR4 w;

	mArray4( )
	{
		memset(this, 0, sizeof(*this));
	}

	mArray4(SCALAR _x, SCALAR2 _y, SCALAR3 _z, SCALAR4 _w)
		: x(_x), y(_y), z(_z), w(_w) { }

	friend ostream& operator << (ostream& os, mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& m) 
	{
		return os << m.x << " " << m.y << " " << m.z << " " << m.w;
	}

	friend istream& operator >> (istream& is, mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& m) 
	{
		return is >> m.x >> m.y >> m.z >> m.w;
	}
	// unary -
	mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4> operator - ( ) const 
	{
		return mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>(-x, -y, -z, -w);
	}

	// vector/vector arithmatic
	mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4> operator + (mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& v) const 
	{
		return mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>(x+v.x, y+v.y, z+v.z, w+v.w);
	}
	mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4> operator - (mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& v) const 
	{
		return mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>(x-v.x, y-v.y, z-v.z, w-v.w);
	}
	mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4> operator += (mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& v) 
	{
		return (*this) = (*this) + v;
	}
	mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4> operator -= (mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& v) 
	{
		return (*this) = (*this) - v;
	}
	bool operator == (mArray4<SCALAR, SCALAR2, SCALAR3, SCALAR4>& v) 
	{
		return (x == v.x && y == v.y && z == v.z && w == v.w);
	}
};


template <class SCALAR>
class mVector2
{
public:
	SCALAR x, y;

	mVector2( )
		: x(0), y(0) { }

	mVector2(SCALAR _x, SCALAR _y)
		: x(_x), y(_y) { }

	friend ostream& operator << (ostream& os, mVector2<SCALAR>& m) 
	{
		return os << m.x << " " << m.y;
	}

	friend istream& operator >> (istream& is, mVector2<SCALAR>& m) 
	{
		return is >> m.x >> m.y;
	}

	// unary -
	mVector2<SCALAR> operator - ( ) const 
	{
		return mVector2<SCALAR>(-x, -y);
	}

	// vector/vector arithmatic
	mVector2<SCALAR> operator + (const mVector2<SCALAR>& v) const 
	{
		return mVector2<SCALAR>(x+v.x, y+v.y);
	}
	mVector2<SCALAR> operator - (const mVector2<SCALAR>& v) const 
	{
		return mVector2<SCALAR>(x-v.x, y-v.y);
	}
	mVector2<SCALAR> operator += (mVector2<SCALAR>& v) 
	{
		return (*this) = (*this) + v;
	}
	mVector2<SCALAR> operator -= (mVector2<SCALAR>& v) 
	{
		return (*this) = (*this) - v;
	}
	bool operator == (mVector2<SCALAR>& v) 
	{
		return (x == v.x && y == v.y);
	}
	SCALAR operator ^ (const mVector2<SCALAR>& v) const 
	{	// dot product
		return (x*v.x) + (y*v.y);
	}
	SCALAR operator * (const mVector2<SCALAR>& v) const 
	{	// cross product
		return (x*v.y) - (y*v.x);
	}

	// vector/scalar arithmatic
	mVector2<SCALAR> operator * (SCALAR s) const 
	{
		return mVector2<SCALAR>(x*s, y*s);
	}
	mVector2<SCALAR> operator / (SCALAR s) const 
	{
		return mVector2<SCALAR>(x/s, y/s);
	}
	mVector2<SCALAR> operator *= (SCALAR s) 
	{
		return (*this) = (*this)*s;
	}
	mVector2<SCALAR> operator /= (SCALAR s) 
	{
		return (*this) = (*this)/s;
	}	
	void Zero() const
	{
		x = 0;
		y = 0;
		return;
	}
	SCALAR getSqrSize( ) const 
	{
		return x*x + y*y;
	}
	SCALAR getSize( ) const 
	{
		return (SCALAR)sqrt((double)(x*x + y*y));
	}
	bool isZero( ) const {
		return (x < MV_POINT_ELEM_ZERO && x > -MV_POINT_ELEM_ZERO &&
			y < MV_POINT_ELEM_ZERO && y > -MV_POINT_ELEM_ZERO);
	}

	mVector2<SCALAR> normalize( ) const 
	{
		if (isZero( )) {
			return mVector2<SCALAR>( );
		} else {
			return (*this)/getSize( );
		}
	}

	// single precision
	// rotate vector using cosine angle
	mVector2<SCALAR> rotate(float c, float s) const 
	{
		return mVector2<SCALAR>((SCALAR)(x*c - y*s), (SCALAR)(x*s + y*c));
	}
	mVector2<SCALAR> rotate(float theta) const 
	{
		return rotate((float)cos(theta), (float)sin(theta));
	}

	// double precision
	mVector2<SCALAR> rotate(double c, double s) const 
	{
		return mVector2<SCALAR>((SCALAR)(x*c - y*s), (SCALAR)(x*s + y*c));
	}
	mVector2<SCALAR> rotate(double theta) const 
	{
		return rotate((double)cos(theta), (double)sin(theta));
	}
};

template <class SCALAR>
class mVector3
{
public:
	union{
		struct{SCALAR x, y, z;};
		SCALAR m[3];	};


	mVector3( )
		: x(0), y(0), z(0) { }

	mVector3(SCALAR _x, SCALAR _y, SCALAR _z)
		: x(_x), y(_y), z(_z) { }

	friend ostream& operator << (ostream& os, mVector3<SCALAR>& m) 
	{
		return os << m.x << " " << m.y << " " << m.z;
	}

	friend istream& operator >> (istream& is, mVector3<SCALAR>& m) 
	{
		return is >> m.x >> m.y >> m.z;
	}
	// SubScript
	SCALAR& operator[] (unsigned int i)
	{
		if(i >2) return m[0];
		return m[i];
	}
	// unary -
	mVector3<SCALAR> operator - ( ) const 
	{
		return mVector3<SCALAR>(-x, -y, -z);
	}

	// vector/vector arithmatic
	mVector3<SCALAR> operator + (const mVector3<SCALAR>& v) const 
	{
		return mVector3<SCALAR>(x+v.x, y+v.y, z+v.z);
	}
	mVector3<SCALAR> operator - (const mVector3<SCALAR>& v) const 
	{
		return mVector3<SCALAR>(x-v.x, y-v.y, z-v.z);
	}
	mVector3<SCALAR> operator += (mVector3<SCALAR>& v) 
	{
		return (*this) = (*this) + v;
	}
	mVector3<SCALAR> operator -= (mVector3<SCALAR>& v) 
	{
		return (*this) = (*this) - v;
	}
	bool operator == (mVector3<SCALAR>& v) 
	{
		return (x == v.x && y == v.y && z == v.z);
	}
	SCALAR operator ^ (const mVector3<SCALAR>& v) const 
	{	// dot product
		return (x*v.x) + (y*v.y) + (z*v.z);
	}
	mVector3<SCALAR> operator * (const mVector3<SCALAR>& v) const 
	{	// cross product
		return mVector3<SCALAR>(	y*v.z - z*v.y,
									z*v.x - x*v.z,
									x*v.y - y*v.x 
								);
	}

	// vector/scalar arithmatic
	mVector3<SCALAR> operator * (SCALAR s) const 
	{
		return mVector3<SCALAR>(x*s, y*s, z*s);
	}
	mVector3<SCALAR> operator / (SCALAR s) const 
	{
		return mVector3<SCALAR>(x/s, y/s, z/s);
	}
	mVector3<SCALAR> operator *= (SCALAR s) 
	{
		return (*this) = (*this)*s;
	}
	mVector3<SCALAR> operator /= (SCALAR s) 
	{
		return (*this) = (*this)/s;
	}	
	void Zero()// const
	{
		x = 0;
		y = 0;
		z = 0;
		return;
	}
	SCALAR getSqrSize( ) const 
	{
		return x*x + y*y + z*z;
	}
	SCALAR getSize( ) const 
	{
		return (SCALAR)sqrt(x*x + y*y + z*z);
	}
	bool isZero( ) const 
	{
		return (x < MV_POINT_ELEM_ZERO && x > -MV_POINT_ELEM_ZERO &&
			y < MV_POINT_ELEM_ZERO && y > -MV_POINT_ELEM_ZERO &&
			z < MV_POINT_ELEM_ZERO && z > -MV_POINT_ELEM_ZERO);
	}

	mVector3<SCALAR> normalize( ) const 
	{
		if (isZero( )) {
			return mVector3<SCALAR>( );
		} else {
			return (*this)/getSize( );
		}
	}
	mVector3<SCALAR> fdiv() const
	{

		return mVector3<SCALAR>(1/(*this).x,1/(*this).y,1/(*this).z);
	}

};

template <class SCALAR>
class mVector4
{
public:
	union{
		struct{SCALAR x, y, z, w;};
		SCALAR m[4];
	};
	mVector4( )
		: x(0), y(0), z(0), w(0) { }

	mVector4(SCALAR _x, SCALAR _y, SCALAR _z, SCALAR _w)
		: x(_x), y(_y), z(_z), w(_w) { }

	friend ostream& operator << (ostream& os, mVector4<SCALAR>& m) 
	{
		return os << m.x << " " << m.y << " " << m.z << " " << m.w;
	}

	friend istream& operator >> (istream& is, mVector4<SCALAR>& m) 
	{
		return is >> m.x >> m.y >> m.z >> m.w;
	}

	//
	SCALAR& operator[] (unsigned int i)
	{
		if(i >3) return m[0];
		return m[i];
	}

	// unary -
	mVector4<SCALAR> operator - ( ) const 
	{
		return mVector4<SCALAR>(-x, -y, -z, -w);
	}

	// vector/vector arithmatic
	mVector4<SCALAR> operator + (const mVector4<SCALAR>& v) const 
	{
		return mVector4<SCALAR>(x+v.x, y+v.y, z+v.z, w+v.w);
	}
	mVector4<SCALAR> operator - (const mVector4<SCALAR>& v) const 
	{
		return mVector4<SCALAR>(x-v.x, y-v.y, z-v.z, w-v.w);
	}
	mVector4<SCALAR> operator += (const mVector4<SCALAR>& v) 
	{
		return (*this) = (*this) + v;
	}
	mVector4<SCALAR> operator -= (const mVector4<SCALAR>& v) 
	{
		return (*this) = (*this) - v;
	}
	bool operator == (mVector4<SCALAR>& v) 
	{
		return (x == v.x && y == v.y && z == v.z && w == v.w);
	}
	SCALAR operator ^ (const mVector4<SCALAR>& v) const 
	{	// dot product
		return (x*v.x) + (y*v.y) + (z*v.z) + (w*v.w);
	}
	mVector4<SCALAR> operator * (const mVector4<SCALAR>& v)
	{
		return mVector4<SCALAR>(	y*v.z - z*v.y,
									z*v.x - x*v.z,
									x*v.y - y*v.x,
									0
								);
	}
	// vector/scalar arithmatic
	mVector4<SCALAR> operator * (const SCALAR s) const 
	{
		return mVector4<SCALAR>(x*s, y*s, z*s, w*s);
	}
	mVector4<SCALAR> operator / (const SCALAR s) const 
	{
		return mVector4<SCALAR>(x/s, y/s, z/s, w/s);
	}
	mVector4<SCALAR> operator *= (const SCALAR s) 
	{
		return (*this) = (*this)*s;
	}
	mVector4<SCALAR> operator /= (SCALAR s) 
	{
		return (*this) = (*this)/s;
	}	

	void Zero() const
	{
		x = 0;
		y = 0;
		z = 0;
		w = 0;
		return;
	}

	SCALAR getSqrSize( ) const 
	{
		return x*x + y*y + z*z + w*w;
	}
	SCALAR getSize( ) const 
	{
		return (SCALAR)sqrt(x*x + y*y + z*z + w*w);
	}
	bool isZero( ) const 
	{
		return (x < MV_POINT_ELEM_ZERO && x > -MV_POINT_ELEM_ZERO &&
			y < MV_POINT_ELEM_ZERO && y > -MV_POINT_ELEM_ZERO &&
			z < MV_POINT_ELEM_ZERO && z > -MV_POINT_ELEM_ZERO &&
			w < MV_POINT_ELEM_ZERO && w > -MV_POINT_ELEM_ZERO);
	}

	mVector4<SCALAR> normalize( ) const 
	{
		if (isZero( )) {
			return mVector4<SCALAR>( );
		} else {
			return (*this)/getSize( );
		}
	}

};

typedef mVector2<int> mVector2i;
typedef mVector2<float> mVector2f;
typedef mVector2<double> mVector2d;

typedef mVector3<int> mVector3i;
typedef mVector3<float> mVector3f;
typedef mVector3<double> mVector3d;

typedef mVector4<int> mVector4i;
typedef mVector4<float> mVector4f;
typedef mVector4<double> mVector4d;

#endif
