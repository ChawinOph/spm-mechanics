#ifndef _M_MATRIX_H_
#define _M_MATRIX_H_

#include <math.h>
#ifdef IOSTREAM
#include <iostream>
#endif
#include "mVect.h"
//using namespace std;

#pragma region class Matrix22
template <class SCALAR>
class Matrix22
{
public:
	SCALAR m11, m12;
	SCALAR m21, m22;

	Matrix22( ) :
		m11(1), m12(0),
		m21(0), m22(1) { };

	Matrix22( SCALAR _m11, SCALAR _m12,
				SCALAR _m21, SCALAR _m22 )
	{
		m11 = _m11; m12 = _m12;
		m21 = _m21; m22 = _m22;
	}

	mVector2<SCALAR> TransformNormal(mVector2<SCALAR> p)
	{
		return mVector2<SCALAR>(
			m11*p.x + m12*p.y, 
			m21*p.x + m22*p.y);
	}

	// Matrix22<SCALAR>/Matrix22<SCALAR> arithmatic
	Matrix22<SCALAR> operator + (Matrix22<SCALAR> m) const 
	{
		Matrix22<SCALAR> m2;
		m2.m11 = m11 + m.m11; m2.m12 = m12 + m.m12;
		m2.m21 = m21 + m.m21; m2.m22 = m22 + m.m22;
		return m2;
	}
	Matrix22<SCALAR> operator - (Matrix22<SCALAR> m) const 
	{
		Matrix22<SCALAR> m2;
		m2.m11 = m11 - m.m11; m2.m12 = m12 - m.m12;
		m2.m21 = m21 - m.m21; m2.m22 = m22 - m.m22;
		return m2;
	}
	Matrix22<SCALAR> operator * (Matrix22<SCALAR> m) const 
	{
		Matrix22<SCALAR> m2;
		m2.m11 = m11*m.m11 + m12*m.m21;
		m2.m12 = m11*m.m12 + m12*m.m22;

		m2.m21 = m21*m.m11 + m22*m.m21;
		m2.m22 = m21*m.m12 + m22*m.m22;

		return m2;
	}
	mVector2<SCALAR> operator ^ (mVector2<SCALAR> v1) const 
	{
		mVector2<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y;
		v.y	= m21*v1.x + m22*v1.y;

		return v;
	}

	mVector2<SCALAR> operator * (mVector2<SCALAR> v1) const 
	{
		mVector2<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y;
		v.y	= m21*v1.x + m22*v1.y;

		return v;
	}

	Matrix22<SCALAR> operator += (Matrix22<SCALAR> m) 
	{
		return (*this) = (*this) + m;
	}
	Matrix22<SCALAR> operator -= (Matrix22<SCALAR> m) 
	{
		return (*this) = (*this) - m;
	}
	Matrix22<SCALAR> operator *= (Matrix22<SCALAR> m) 
	{
		return (*this) = (*this) * m;
	}

	// Matrix22<SCALAR>/scalar arithmatic
	Matrix22<SCALAR> operator * (SCALAR s) const 
	{
		Matrix22<SCALAR> m2	= *this;
		m2.m11 *= s; m2.m12 *= s;
		m2.m21 *= s; m2.m22 *= s;
		return m2;
	}
	Matrix22<SCALAR> operator / (SCALAR s) const 
	{
		Matrix22<SCALAR> m2	= *this;
		m2.m11 /= s; m2.m12 /= s;
		m2.m21 /= s; m2.m22 /= s;
		return m2;
	}
	Matrix22<SCALAR> operator *= (SCALAR s) 
	{
		return (*this) = (*this)*s;
	}
	Matrix22<SCALAR> operator /= (SCALAR s) 
	{
		return (*this) = (*this)/s;
	}	
#ifdef IOSTREAM
	friend ostream& operator << (ostream& os, Matrix22<SCALAR>& m) {
		return os << m.m11 << " " << m.m12 << endl
			<< m.m21 << " " << m.m22 << endl;
	}

	friend istream& operator >> (istream& is, Matrix22<SCALAR>& m) {
		return is >> m.m11 >> m.m12
			>> m.m21 >> m.m22;
	}
#endif
	void Identity();
	void Transpose();
	void Scale( SCALAR scaleX, SCALAR scaleY);
	void Rotation( SCALAR angle );
	void Rotation( SCALAR sin, SCALAR cos );

	static void Identity(Matrix22<SCALAR>* m1);
	static void Scale(Matrix22<SCALAR>* m1, SCALAR scaleX, SCALAR scaleY);
	static void Rotation( Matrix22<SCALAR>* m1, SCALAR angle );
	static void Rotation( Matrix22<SCALAR>* m1, SCALAR sin, SCALAR cos );

	static SCALAR Determinant( const Matrix22<SCALAR>* m );
	static void Adjoint( Matrix22<SCALAR>* result, const Matrix22<SCALAR>* m );
	static void Invert( Matrix22<SCALAR>* result, const Matrix22<SCALAR>* m );
	static void Transpose( Matrix22<SCALAR>* result, const Matrix22<SCALAR>* m );
};

typedef Matrix22<float> Matrix22f;
typedef Matrix22<double> Matrix22d;

template <class SCALAR>
void Matrix22<SCALAR>::Identity()
{
	m11 = 1; m12 = 0;
	m21 = 0; m22 = 1;
}

template <class SCALAR>
void Matrix22<SCALAR>::Transpose()
{
	Matrix22<SCALAR> srcMat = (*this);
	Matrix22<SCALAR>::Transpose(this, &srcMat);
}

template <class SCALAR>
void Matrix22<SCALAR>::Scale( SCALAR scaleX, SCALAR scaleY )
{
	m11 *= scaleX; m12 *= scaleX;
	m21 *= scaleY; m22 *= scaleY;
}

template <class SCALAR>
void Matrix22<SCALAR>::Rotation( SCALAR angle )
{
	Rotation( (SCALAR)sin(angle), (SCALAR)cos(angle) );
}

template <class SCALAR>
void Matrix22<SCALAR>::Rotation( SCALAR sin, SCALAR cos )
{
	SCALAR _11 = m11;
	SCALAR _21 = m21;
	m11 = m11*cos + m12*sin;
	m21 = m21*cos + m22*sin;

	m12 = m12*cos - _11*sin;
	m22 = m22*cos - _21*sin;
}

template <class SCALAR>
void Matrix22<SCALAR>::Identity(Matrix22<SCALAR>* m1)
{
	m1->m11 = 1; m1->m12 = 0;
	m1->m21 = 0; m1->m22 = 1;
}

template <class SCALAR>
void Matrix22<SCALAR>::Scale(Matrix22<SCALAR>* m1, SCALAR scaleX, SCALAR scaleY)
{
	m1->m11 = scaleX; m1->m12 = 0;
	m1->m21 = 0;      m1->m22 = scaleY;
}

template <class SCALAR>
void Matrix22<SCALAR>::Rotation( Matrix22<SCALAR>* m1, SCALAR angle )
{
	RotationX( m1, sin(angle), cos(angle) );
}

template <class SCALAR>
void Matrix22<SCALAR>::Rotation( Matrix22<SCALAR>* m1, SCALAR sin, SCALAR cos )
{
	m1->m11	= (SCALAR)cos;	m1->m12	= -(SCALAR)sin;
	m1->m21	= (SCALAR)sin;	m1->m22	= (SCALAR)cos;
}

template <class SCALAR>
SCALAR Matrix22<SCALAR>::Determinant( const Matrix22<SCALAR>* m )
{
	return m->m11*m->m22 - m->m12*m->m21;
}

template <class SCALAR>
void Matrix22<SCALAR>::Adjoint( Matrix22<SCALAR>* result, const Matrix22<SCALAR>* m )
{	
	result->m11 =  m->m22;
	result->m21 = -m->m21;
	
	result->m12 = -m->m12;
	result->m22 =  m->m11;
}

template <class SCALAR>
void Matrix22<SCALAR>::Invert( Matrix22<SCALAR>* result, const Matrix22<SCALAR>* m )
{
	SCALAR det = m->m11*m->m22 - m->m12*m->m21;
	result->m11 =  m->m22 / det;
	result->m21 = -m->m21 / det;
	
	result->m12 = -m->m12 / det;
	result->m22 =  m->m11 / det;
}

template <class SCALAR>
void Matrix22<SCALAR>::Transpose( Matrix22<SCALAR>* result, const Matrix22<SCALAR>* m )
{
	result->m11	= m->m11;	result->m12	= m->m21;
	result->m21	= m->m12;	result->m22	= m->m22;
}
#pragma endregion

#pragma region class Matrix33
template <class SCALAR>
class Matrix33
{
public:
	union{
		struct{
		SCALAR m11, m12, m13;
		SCALAR m21, m22, m23;
		SCALAR m31, m32, m33;
		};
		SCALAR d[9];

	};
	Matrix33( ) :
		m11(1), m12(0), m13(0),
		m21(0), m22(1), m23(0),
		m31(0), m32(0), m33(1) { };

	Matrix33( SCALAR _m11, SCALAR _m12, SCALAR _m13,
				SCALAR _m21, SCALAR _m22, SCALAR _m23,
				SCALAR _m31, SCALAR _m32, SCALAR _m33 )
	{
		m11 = _m11; m12 = _m12; m13 = _m13;
		m21 = _m21; m22 = _m22; m23 = _m23;
		m31 = _m31; m32 = _m32; m33 = _m33;
	}

	mVector2<SCALAR> TransformCoordinate(mVector2<SCALAR> p)
	{
		return mVector2<SCALAR>(
			m11*p.x + m12*p.y + m13, 
			m21*p.x + m22*p.y + m23);
	}

	mVector2<SCALAR> TransformNormal(mVector2<SCALAR> p)
	{
		return mVector2<SCALAR>(
			m11*p.x + m12*p.y, 
			m21*p.x + m22*p.y);
	}

	mVector3<SCALAR> TransformCoordinate(mVector3<SCALAR> p)
	{
		return mVector3<SCALAR>(
			m11*p.x + m12*p.y + m13*p.z, 
			m21*p.x + m22*p.y + m23*p.z, 
			m31*p.x + m32*p.y + m33*p.z);
	}

	SCALAR& operator [] (unsigned int i)
	{
		if(i >8) return d[0];
		return d[i];
	}
	SCALAR& e (unsigned int i =0,unsigned int j =0)
	{
		if(i >2) return d[0];
		if(j >2) return d[0];
		return d[i * 3 +j ];
	}
	// Matrix33<SCALAR>/Matrix33<SCALAR> arithmatic
	Matrix33<SCALAR> operator + (Matrix33<SCALAR> m) const 
	{
		Matrix33<SCALAR> m2;
		m2.m11 = m11 + m.m11; m2.m12 = m12 + m.m12; m2.m13 = m13 + m.m13;
		m2.m21 = m21 + m.m21; m2.m22 = m22 + m.m22; m2.m23 = m23 + m.m23;
		m2.m31 = m31 + m.m31; m2.m32 = m32 + m.m32; m2.m33 = m33 + m.m33;
		return m2;
	}
	Matrix33<SCALAR> operator - (Matrix33<SCALAR> m) const 
	{
		Matrix33<SCALAR> m2;
		m2.m11 = m11 - m.m11; m2.m12 = m12 - m.m12; m2.m13 = m13 - m.m13;
		m2.m21 = m21 - m.m21; m2.m22 = m22 - m.m22; m2.m23 = m23 - m.m23;
		m2.m31 = m31 - m.m31; m2.m32 = m32 - m.m32; m2.m33 = m33 - m.m33;
		return m2;
	}
	Matrix33<SCALAR> operator * (Matrix33<SCALAR> m) const 
	{
		Matrix33<SCALAR> m2;
		m2.m11 = m11*m.m11 + m12*m.m21 + m13*m.m31;
		m2.m12 = m11*m.m12 + m12*m.m22 + m13*m.m32;
		m2.m13 = m11*m.m13 + m12*m.m23 + m13*m.m33;

		m2.m21 = m21*m.m11 + m22*m.m21 + m23*m.m31;
		m2.m22 = m21*m.m12 + m22*m.m22 + m23*m.m32;
		m2.m23 = m21*m.m13 + m22*m.m23 + m23*m.m33;

		m2.m31 = m31*m.m11 + m32*m.m21 + m33*m.m31;
		m2.m32 = m31*m.m12 + m32*m.m22 + m33*m.m32;
		m2.m33 = m31*m.m13 + m32*m.m23 + m33*m.m33;

		return m2;
	}
	mVector2<SCALAR> operator ^ (mVector2<SCALAR> v1) const 
	{
		mVector2<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y;
		v.y	= m21*v1.x + m22*v1.y;

		return v;
	}

	mVector2<SCALAR> operator * (mVector2<SCALAR> v1) const 
	{
		mVector2<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y + m13;
		v.y	= m21*v1.x + m22*v1.y + m23;

		return v;
	}

	mVector3<SCALAR> operator * (mVector3<SCALAR> v1) const 
	{
		return mVector3<SCALAR>(
			m11*v1.x + m12*v1.y + m13*v1.z, 
			m21*v1.x + m22*v1.y + m23*v1.z, 
			m31*v1.x + m32*v1.y + m33*v1.z);
	}

	Matrix33<SCALAR> operator += (Matrix33<SCALAR> m) 
	{
		return (*this) = (*this) + m;
	}
	Matrix33<SCALAR> operator -= (Matrix33<SCALAR> m) 
	{
		return (*this) = (*this) - m;
	}
	Matrix33<SCALAR> operator *= (Matrix33<SCALAR> m) 
	{
		return (*this) = (*this) * m;
	}

	// Matrix33<SCALAR>/scalar arithmatic
	Matrix33<SCALAR> operator * (SCALAR s) const 
	{
		Matrix33<SCALAR> m2	= *this;
		m2.m11 *= s; m2.m12 *= s; m2.m13 *= s;
		m2.m21 *= s; m2.m22 *= s; m2.m23 *= s;
		m2.m31 *= s; m2.m32 *= s; m2.m33 *= s;
		return m2;
	}
	Matrix33<SCALAR> operator / (SCALAR s) const 
	{
		Matrix33<SCALAR> m2	= *this;
		m2.m11 /= s; m2.m12 /= s; m2.m13 /= s;
		m2.m21 /= s; m2.m22 /= s; m2.m23 /= s;
		m2.m31 /= s; m2.m32 /= s; m2.m33 /= s;
		return m2;
	}
	Matrix33<SCALAR> operator *= (SCALAR s) 
	{
		return (*this) = (*this)*s;
	}
	Matrix33<SCALAR> operator /= (SCALAR s) 
	{
		return (*this) = (*this)/s;
	}	
#ifdef IOSTREAM
	friend ostream& operator << (ostream& os, Matrix33<SCALAR>& m) {
		return os << m.m11 << " " << m.m12 << " " << m.m13 << endl
			<< m.m21 << " " << m.m22 << " " << m.m23 << endl
			<< m.m31 << " " << m.m32 << " " << m.m33 << endl;
	}

	friend istream& operator >> (istream& is, Matrix33<SCALAR>& m) {
		return is >> m.m11 >> m.m12 >> m.m13
			>> m.m21 >> m.m22 >> m.m23
			>> m.m31 >> m.m32 >> m.m33;
	}
#endif
	void Identity();
	void Transpose();
	void Scale(SCALAR scaleX, SCALAR scaleY);
	void Rotation( SCALAR angle );
	void Rotation( SCALAR sin, SCALAR cos );
	void Translation( SCALAR x, SCALAR y );

	void RotationAxis_vect( mVector3<SCALAR> vect  );
	void RotationAxis_twoframe( mVector3<SCALAR> Au,
											mVector3<SCALAR> Av,
											mVector3<SCALAR> Bu,
											mVector3<SCALAR> Bv);
	void RotationAxis( SCALAR angle, SCALAR x, SCALAR y, SCALAR z );
	void RotationAxis( SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z );

	static void Identity(Matrix33<SCALAR>* m1);
	static void Scale(Matrix33<SCALAR>* m1, SCALAR scaleX, SCALAR scaleY);
	static void Rotation( Matrix33<SCALAR>* m1, SCALAR angle );
	static void Rotation( Matrix33<SCALAR>* m1, SCALAR sin, SCALAR cos );
	static void Translation( Matrix33<SCALAR>* m1, SCALAR x, SCALAR y );

	
	static void RotationAxis( Matrix33<SCALAR>* m1, SCALAR angle, SCALAR x, SCALAR y, SCALAR z );
	static void RotationAxis( Matrix33<SCALAR>* m1, SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z );
	



	static SCALAR Determinant( const Matrix33<SCALAR>* m );
	static void Adjoint( Matrix33<SCALAR>* result, const Matrix33<SCALAR>* m );
	static void Invert( Matrix33<SCALAR>* result, const Matrix33<SCALAR>* m );
	static void Transpose( Matrix33<SCALAR>* result, const Matrix33<SCALAR>* m );
};

typedef Matrix33<float> Matrix33f;
typedef Matrix33<double> Matrix33d;

template <class SCALAR>
void Matrix33<SCALAR>::Identity()
{
	m11 = 1; m12 = 0; m13 = 0;
	m21 = 0; m22 = 1; m23 = 0;
	m31 = 0; m32 = 0; m33 = 1;
}

template <class SCALAR>
void Matrix33<SCALAR>::Transpose()
{
	Matrix33<SCALAR> srcMat = (*this);
	Matrix33<SCALAR>::Transpose(this, &srcMat);
}

template <class SCALAR>
void Matrix33<SCALAR>::Scale( SCALAR scaleX, SCALAR scaleY )
{
	m11 *= scaleX; m12 *= scaleX;
	m21 *= scaleY; m22 *= scaleY;
}

template <class SCALAR>
void Matrix33<SCALAR>::Rotation( SCALAR angle )
{
	Rotation( (SCALAR)sin(angle), (SCALAR)cos(angle) );
}

template <class SCALAR>
void Matrix33<SCALAR>::Rotation( SCALAR sin, SCALAR cos )
{
	SCALAR _11 = m11;
	SCALAR _21 = m21;
	SCALAR _31 = m31;
	m11 = m11*cos + m12*sin;
	m21 = m21*cos + m22*sin;
	m31 = m31*cos + m32*sin;

	m12 = m12*cos - _11*sin;
	m22 = m22*cos - _21*sin;
	m32 = m32*cos - _31*sin;



}
template <class SCALAR>
void Matrix33<SCALAR>::RotationAxis_vect( mVector3<SCALAR> vect  )
{
	SCALAR deg = vect.getSize() ;//* (180 / 3.1415926535897932384626433832795);
	vect = vect.normalize();
	RotationAxis((SCALAR)sin(deg), (SCALAR)cos(deg), vect.x, vect.y, vect.z );
}
// input vector must be unit
template <class SCALAR>
void Matrix33<SCALAR>::RotationAxis_twoframe( mVector3<SCALAR> Au,
											mVector3<SCALAR> Av,
											mVector3<SCALAR> Bu,
											mVector3<SCALAR> Bv)
{
	Matrix33<SCALAR> R1;
	Matrix33<SCALAR> R2;
	mVector3<SCALAR> U,V,T;
	U = (Au * Bu).normalize() *  acos(Au ^ Bu);
	R1.RotationAxis_vect(U);
	//T = R1 * Au;
	T = R1 * Av;
	V = (T * Bv).normalize() * acos(T ^ Bv);
	R2.RotationAxis_vect(V);
	(*this) = R2 * R1;

	/*mVector3<SCALAR> Aw;
	mVector3<SCALAR> Bw;
	Aw = Au * Av;	// perpendicular only
	Bw = Bu * Bv;

	m11 = Bu ^ Au;	m12 = Bv ^ Au;	m13 = Bw ^ Au;	
	m21 = Bu ^ Av;	m22 = Bv ^ Av;	m23 = Bw ^ Av;	
	m31 = Bu ^ Aw;	m32 = Bv ^ Aw;	m33 = Bw ^ Aw;	
	*/

}
template <class SCALAR>
void Matrix33<SCALAR>::RotationAxis( SCALAR angle, SCALAR x, SCALAR y, SCALAR z )
{
	RotationAxis((SCALAR)sin(angle), (SCALAR)cos(angle), x, y, z );
}

template <class SCALAR>
void Matrix33<SCALAR>::RotationAxis( SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z )
{
	Matrix33<SCALAR> rotMat;
	Matrix33<SCALAR>::RotationAxis(&rotMat, sin, cos, x, y, z);
	(*this) = (*this) * rotMat;
}
template <class SCALAR>
void Matrix33<SCALAR>::RotationAxis( Matrix33<SCALAR>* m1, SCALAR angle, SCALAR x, SCALAR y, SCALAR z )
{
	RotationAxis( m1, sin(angle), cos(angle), x, y, z );
}

template <class SCALAR>
void Matrix33<SCALAR>::RotationAxis( Matrix33<SCALAR>* m1, SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z )
{
	m1->m11	= 1 + (1 - cos)*(x*x - 1); m1->m12 = -z*sin + (1 - cos)*x*y;  m1->m13 =  y*sin + (1 - cos)*x*z;
	m1->m21	=  z*sin + (1 - cos)*x*y;  m1->m22 = 1 + (1 - cos)*(y*y - 1); m1->m23 = -x*sin + (1 - cos)*y*z;
	m1->m31	= -y*sin + (1 - cos)*x*z;  m1->m32 =  x*sin + (1 - cos)*y*z;  m1->m33 =  1 + (1 - cos)*(z*z - 1);
	
}

template <class SCALAR>
void Matrix33<SCALAR>::Translation( SCALAR x, SCALAR y )
{
	m13 += m11 * x + m12 * y;
	m23 += m21 * x + m22 * y;
}

template <class SCALAR>
void Matrix33<SCALAR>::Identity(Matrix33<SCALAR>* m1)
{
	m1->m11 = 1; m1->m12 = 0; m1->m13 = 0;
	m1->m21 = 0; m1->m22 = 1; m1->m23 = 0;
	m1->m31 = 0; m1->m32 = 0; m1->m33 = 1;
}

template <class SCALAR>
void Matrix33<SCALAR>::Scale(Matrix33<SCALAR>* m1, SCALAR scaleX, SCALAR scaleY)
{
	m1->m11 = scaleX; m1->m12 = 0;      m1->m13 = 0;
	m1->m21 = 0;      m1->m22 = scaleY; m1->m23 = 0;
	m1->m31 = 0;      m1->m32 = 0;      m1->m33 = 1;
}

template <class SCALAR>
void Matrix33<SCALAR>::Rotation( Matrix33<SCALAR>* m1, SCALAR angle )
{
	Rotation( m1, sin(angle), cos(angle) );
}

template <class SCALAR>
void Matrix33<SCALAR>::Rotation( Matrix33<SCALAR>* m1, SCALAR sin, SCALAR cos )
{
	m1->m11	= (SCALAR)cos;	m1->m12	= -(SCALAR)sin;	m1->m13	= 0;
	m1->m21	= (SCALAR)sin;	m1->m22	= (SCALAR)cos;	m1->m23	= 0;
	m1->m31	= 0;			m1->m32	= 0;			m1->m33	= 1;
}

template <class SCALAR>
void Matrix33<SCALAR>::Translation( Matrix33<SCALAR>* m1, SCALAR x, SCALAR y )
{
	m1->m11	= 1;	m1->m12	= 0;	m1->m13	= (SCALAR)x;
	m1->m21	= 0;	m1->m22	= 1;	m1->m23	= (SCALAR)y;
	m1->m31	= 0;	m1->m32	= 0;	m1->m33	= 1;
}

template <class SCALAR>
SCALAR Matrix33<SCALAR>::Determinant( const Matrix33<SCALAR>* m )
{
	return 
		m->m11*m->m22*m->m33 + m->m12*m->m23*m->m31 + m->m13*m->m21*m->m32 -
		m->m13*m->m22*m->m31 - m->m11*m->m23*m->m32 - m->m12*m->m21*m->m33;
}

template <class SCALAR>
void Matrix33<SCALAR>::Adjoint( Matrix33<SCALAR>* result, const Matrix33<SCALAR>* m )
{	
	result->m11 =   m->m22*m->m33 - m->m23*m->m32;
	result->m21 = - m->m21*m->m33 + m->m23*m->m31;
	result->m31 =   m->m21*m->m32 - m->m22*m->m31;
	
	result->m12 = - m->m12*m->m33 + m->m13*m->m32;
	result->m22 =   m->m11*m->m33 - m->m13*m->m31;
	result->m32 = - m->m11*m->m32 + m->m12*m->m31;

	result->m13 =   m->m12*m->m23 - m->m13*m->m22;
	result->m23 = - m->m11*m->m23 + m->m13*m->m21;
	result->m33 =   m->m11*m->m22 - m->m12*m->m21;
}

template <class SCALAR>
void Matrix33<SCALAR>::Invert( Matrix33<SCALAR>* result, const Matrix33<SCALAR>* m )
{
	SCALAR det = Determinant( m );
	Adjoint( result, m );

	result->m11	/= det;	result->m12	/= det;	result->m13	/= det;
	result->m21	/= det;	result->m22	/= det;	result->m23	/= det;
	result->m31	/= det;	result->m32	/= det;	result->m33	/= det;
}

template <class SCALAR>
void Matrix33<SCALAR>::Transpose( Matrix33<SCALAR>* result, const Matrix33<SCALAR>* m )
{
	result->m11	= m->m11;	result->m12	= m->m21;	result->m13	= m->m31;
	result->m21	= m->m12;	result->m22	= m->m22;	result->m23	= m->m32;
	result->m31	= m->m13;	result->m32	= m->m23;	result->m33	= m->m33;
}
#pragma endregion

#pragma region class Matrix44
template <class SCALAR>
class Matrix44
{
public:
	union{
		SCALAR m[16];
		struct {
			SCALAR m11, m21, m31, m41;
			SCALAR m12, m22, m32, m42;
			SCALAR m13, m23, m33, m43;
			SCALAR m14, m24, m34, m44;
		};
	};

	Matrix44( ) :
		m11(1), m12(0), m13(0), m14(0),
		m21(0), m22(1), m23(0), m24(0),
		m31(0), m32(0), m33(1), m34(0),
		m41(0), m42(0), m43(0), m44(1) { };

	Matrix44( SCALAR _m11, SCALAR _m12, SCALAR _m13, SCALAR _m14,
				SCALAR _m21, SCALAR _m22, SCALAR _m23, SCALAR _m24,
				SCALAR _m31, SCALAR _m32, SCALAR _m33, SCALAR _m34,
				SCALAR _m41, SCALAR _m42, SCALAR _m43, SCALAR _m44 )
	{
		m11 = _m11; m12 = _m12; m13 = _m13; m14 = _m14;
		m21 = _m21; m22 = _m22; m23 = _m23; m24 = _m24;
		m31 = _m31; m32 = _m32; m33 = _m33; m34 = _m34;
		m41 = _m41; m42 = _m42; m43 = _m43; m44 = _m44;
	}

	mVector3<SCALAR> TransformCoordinate(mVector3<SCALAR> p)
	{
		return mVector3<SCALAR>(
			m11*p.x + m12*p.y + m13*p.z + m14, 
			m21*p.x + m22*p.y + m23*p.z + m24, 
			m31*p.x + m32*p.y + m33*p.z + m34);
	}

	mVector3<SCALAR> TransformNormal(mVector3<SCALAR> p)
	{
		return mVector3<SCALAR>(
			m11*p.x + m12*p.y + m13*p.z, 
			m21*p.x + m22*p.y + m23*p.z, 
			m31*p.x + m32*p.y + m33*p.z);
	}

	// Matrix44<SCALAR>/Matrix44<SCALAR> arithmatic
	Matrix44<SCALAR> operator + (Matrix44<SCALAR> m) const 
	{
		Matrix44<SCALAR> m2;
		m2.m11 = m11 + m.m11; m2.m12 = m12 + m.m12; m2.m13 = m13 + m.m13; m2.m14 = m14 + m.m14;
		m2.m21 = m21 + m.m21; m2.m22 = m22 + m.m22; m2.m23 = m23 + m.m23; m2.m24 = m24 + m.m24;
		m2.m31 = m31 + m.m31; m2.m32 = m32 + m.m32; m2.m33 = m33 + m.m33; m2.m34 = m34 + m.m34;
		m2.m41 = m41 + m.m41; m2.m42 = m42 + m.m42; m2.m43 = m43 + m.m43; m2.m44 = m44 + m.m44;
		return m2;
	}
	Matrix44<SCALAR> operator - (Matrix44<SCALAR> m) const 
	{
		Matrix44<SCALAR> m2;
		m2.m11 = m11 - m.m11; m2.m12 = m12 - m.m12; m2.m13 = m13 - m.m13; m2.m14 = m14 - m.m14;
		m2.m21 = m21 - m.m21; m2.m22 = m22 - m.m22; m2.m23 = m23 - m.m23; m2.m24 = m24 - m.m24;
		m2.m31 = m31 - m.m31; m2.m32 = m32 - m.m32; m2.m33 = m33 - m.m33; m2.m34 = m34 - m.m34;
		m2.m41 = m41 - m.m41; m2.m42 = m42 - m.m42; m2.m43 = m43 - m.m43; m2.m44 = m44 - m.m44;
		return m2;
	}
	Matrix44<SCALAR> operator * (Matrix44<SCALAR> m) const 
	{
		Matrix44<SCALAR> m2;
		m2.m11 = m11*m.m11 + m12*m.m21 + m13*m.m31 + m14*m.m41;
		m2.m12 = m11*m.m12 + m12*m.m22 + m13*m.m32 + m14*m.m42;
		m2.m13 = m11*m.m13 + m12*m.m23 + m13*m.m33 + m14*m.m43;
		m2.m14 = m11*m.m14 + m12*m.m24 + m13*m.m34 + m14*m.m44;

		m2.m21 = m21*m.m11 + m22*m.m21 + m23*m.m31 + m24*m.m41;
		m2.m22 = m21*m.m12 + m22*m.m22 + m23*m.m32 + m24*m.m42;
		m2.m23 = m21*m.m13 + m22*m.m23 + m23*m.m33 + m24*m.m43;
		m2.m24 = m21*m.m14 + m22*m.m24 + m23*m.m34 + m24*m.m44;

		m2.m31 = m31*m.m11 + m32*m.m21 + m33*m.m31 + m34*m.m41;
		m2.m32 = m31*m.m12 + m32*m.m22 + m33*m.m32 + m34*m.m42;
		m2.m33 = m31*m.m13 + m32*m.m23 + m33*m.m33 + m34*m.m43;
		m2.m34 = m31*m.m14 + m32*m.m24 + m33*m.m34 + m34*m.m44;

		m2.m41 = m41*m.m11 + m42*m.m21 + m43*m.m31 + m44*m.m41;
		m2.m42 = m41*m.m12 + m42*m.m22 + m43*m.m32 + m44*m.m42;
		m2.m43 = m41*m.m13 + m42*m.m23 + m43*m.m33 + m44*m.m43;
		m2.m44 = m41*m.m14 + m42*m.m24 + m43*m.m34 + m44*m.m44;

		return m2;
	}
	mVector3<SCALAR> operator ^ (mVector3<SCALAR> v1) const 
	{
		mVector3<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y + m13*v1.z;
		v.y	= m21*v1.x + m22*v1.y + m23*v1.z;
		v.z	= m31*v1.x + m32*v1.y + m33*v1.z;

		return v;
	}

	mVector3<SCALAR> operator * (mVector3<SCALAR> v1) const 
	{
		mVector3<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y + m13*v1.z + m14;
		v.y	= m21*v1.x + m22*v1.y + m23*v1.z + m24;
		v.z	= m31*v1.x + m32*v1.y + m33*v1.z + m34;

		return v;
	}
	
	mVector4<SCALAR> operator * (mVector4<SCALAR> v1) const 
	{
		mVector4<SCALAR> v;
		v.x	= m11*v1.x + m12*v1.y + m13*v1.z + m14*v1.w;
		v.y	= m21*v1.x + m22*v1.y + m23*v1.z + m24*v1.w;
		v.z	= m31*v1.x + m32*v1.y + m33*v1.z + m34*v1.w;
		v.w	= m41*v1.x + m42*v1.y + m43*v1.z + m44*v1.w;

		return v;
	}

	Matrix44<SCALAR> operator += (Matrix44<SCALAR> m) 
	{
		return (*this) = (*this) + m;
	}
	Matrix44<SCALAR> operator -= (Matrix44<SCALAR> m) 
	{
		return (*this) = (*this) - m;
	}
	Matrix44<SCALAR> operator *= (Matrix44<SCALAR> m) 
	{
		return (*this) = (*this) * m;
	}

	// Matrix44<SCALAR>/scalar arithmatic
	Matrix44<SCALAR> operator * (SCALAR s) const 
	{
		Matrix44<SCALAR> m2	= *this;
		m2.m11 *= s; m2.m12 *= s; m2.m13 *= s; m2.m14 *= s;
		m2.m21 *= s; m2.m22 *= s; m2.m23 *= s; m2.m24 *= s;
		m2.m31 *= s; m2.m32 *= s; m2.m33 *= s; m2.m34 *= s;
		m2.m41 *= s; m2.m42 *= s; m2.m43 *= s; m2.m44 *= s;
		return m2;
	}
	Matrix44<SCALAR> operator / (SCALAR s) const 
	{
		Matrix44<SCALAR> m2	= *this;
		m2.m11 /= s; m2.m12 /= s; m2.m13 /= s; m2.m14 /= s;
		m2.m21 /= s; m2.m22 /= s; m2.m23 /= s; m2.m24 /= s;
		m2.m31 /= s; m2.m32 /= s; m2.m33 /= s; m2.m34 /= s;
		m2.m41 /= s; m2.m42 /= s; m2.m43 /= s; m2.m44 /= s;
		return m2;
	}
	Matrix44<SCALAR> operator *= (SCALAR s) 
	{
		return (*this) = (*this)*s;
	}
	Matrix44<SCALAR> operator /= (SCALAR s) 
	{
		return (*this) = (*this)/s;
	}	
#ifdef IOSTREAM
	friend ostream& operator << (ostream& os, Matrix44<SCALAR>& m) {
		return os << m.m11 << " " << m.m12 << " " << m.m13 << " " << m.m14 << endl
			<< m.m21 << " " << m.m22 << " " << m.m23 << " " << m.m24 << endl
			<< m.m31 << " " << m.m32 << " " << m.m33 << " " << m.m34 << endl
			<< m.m41 << " " << m.m42 << " " << m.m43 << " " << m.m44 << endl;
	}

	friend istream& operator >> (istream& is, Matrix44<SCALAR>& m) {
		return is >> m.m11 >> m.m12 >> m.m13 >> m.m14
			>> m.m21 >> m.m22 >> m.m23 >> m.m24
			>> m.m31 >> m.m32 >> m.m33 >> m.m34
			>> m.m41 >> m.m42 >> m.m43 >> m.m44;
	}
#endif
	void Identity();
	void Transpose();
	void Scale(SCALAR scaleX, SCALAR scaleY, SCALAR scaleZ);
	void RotationX( SCALAR angleX );
	void RotationX( SCALAR sin, SCALAR cos );
	void RotationY( SCALAR angleY );
	void RotationY( SCALAR sin, SCALAR cos );
	void RotationZ( SCALAR angleZ );
	void RotationZ( SCALAR sin, SCALAR cos );
	void Rotation( SCALAR angleX, SCALAR angleY, SCALAR angleZ );
	void RotationAxis( SCALAR angle, SCALAR x, SCALAR y, SCALAR z );
	void RotationAxis( SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z );
	void RotationAxis_vect( mVector3<SCALAR> vect  );
	void Translation( SCALAR x, SCALAR y, SCALAR z );
	void GetEulerRotation( SCALAR* alpha, SCALAR* beta, SCALAR* gammar );

	static void Identity(Matrix44<SCALAR>* m1);
	static void Scale(Matrix44<SCALAR>* m1, SCALAR scaleX, SCALAR scaleY, SCALAR scaleZ);
	static void RotationX( Matrix44<SCALAR>* m1, SCALAR angleX );
	static void RotationX( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos );
	static void RotationY( Matrix44<SCALAR>* m1, SCALAR angleY );
	static void RotationY( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos );
	static void RotationZ( Matrix44<SCALAR>* m1, SCALAR angleZ );
	static void RotationZ( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos );
	static void Rotation( Matrix44<SCALAR>* m1, SCALAR angleX, SCALAR angleY, SCALAR angleZ );
	static void RotationAxis( Matrix44<SCALAR>* m1, SCALAR angle, SCALAR x, SCALAR y, SCALAR z );
	static void RotationAxis( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z );
	//static void RotationAxis_vect( mVector3<SCALAR> vect  );
	static void Translation( Matrix44<SCALAR>* m1, SCALAR x, SCALAR y, SCALAR z );

	static SCALAR Determinant( const Matrix44<SCALAR>* m );
	static void Adjoint( Matrix44<SCALAR>* result, const Matrix44<SCALAR>* m );
	static void Invert( Matrix44<SCALAR>* result, const Matrix44<SCALAR>* m );
	static void Transpose( Matrix44<SCALAR>* result, const Matrix44<SCALAR>* m );

	static void PerspectiveFovLH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR aspectRatio, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveFovRH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR aspectRatio, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveFovXYLH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR fieldOfViewX, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveFovXYRH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR fieldOfViewX, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveLH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveRH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveOffCenterLH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane);
	static void PerspectiveOffCenterRH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane);
	static void OrthoLH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane);
	static void OrthoRH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane);
	static void OrthoOffCenterLH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane);
	static void OrthoOffCenterRH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane);
	static void LookAtLH(Matrix44<SCALAR>* mat, mVector3<SCALAR> cameraPosition, mVector3<SCALAR> cameraTarget, mVector3<SCALAR> cameraUpVector);
	static void LookAtRH(Matrix44<SCALAR>* mat, mVector3<SCALAR> cameraPosition, mVector3<SCALAR> cameraTarget, mVector3<SCALAR> cameraUpVector);
};

typedef Matrix44<float> Matrix44f;
typedef Matrix44<double> Matrix44d;

template <class SCALAR>
void Matrix44<SCALAR>::Identity()
{
	m11 = 1; m12 = 0; m13 = 0; m14 = 0;
	m21 = 0; m22 = 1; m23 = 0; m24 = 0;
	m31 = 0; m32 = 0; m33 = 1; m34 = 0;
	m41 = 0; m42 = 0; m43 = 0; m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::Transpose()
{
	Matrix44<SCALAR> srcMat = (*this);
	Matrix44<SCALAR>::Transpose(this, &srcMat);
}

template <class SCALAR>
void Matrix44<SCALAR>::Scale( SCALAR scaleX, SCALAR scaleY, SCALAR scaleZ )
{
	m11 *= scaleX; m12 *= scaleY; m13 *= scaleZ;
	m21 *= scaleX; m22 *= scaleY; m23 *= scaleZ;
	m31 *= scaleX; m32 *= scaleY; m33 *= scaleZ;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationX( SCALAR angleX )
{
	RotationX( (SCALAR)sin(angleX), (SCALAR)cos(angleX) );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationX( SCALAR sin, SCALAR cos )
{
	SCALAR _12 = m12;
	SCALAR _22 = m22;
	SCALAR _32 = m32;
	SCALAR _42 = m42;
	m12 = m12*cos + m13*sin;
	m22 = m22*cos + m23*sin;
	m32 = m32*cos + m33*sin;
	m42 = m42*cos + m43*sin;

	m13 = m13*cos - _12*sin;
	m23 = m23*cos - _22*sin;
	m33 = m33*cos - _32*sin;
	m43 = m43*cos - _42*sin;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationY( SCALAR angleY )
{
	RotationY( (SCALAR)sin(angleY), (SCALAR)cos(angleY) );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationY( SCALAR sin, SCALAR cos )
{
	SCALAR _11 = m11;
	SCALAR _21 = m21;
	SCALAR _31 = m31;
	SCALAR _41 = m41;
	m11 = m11*cos - m13*sin;
	m21 = m21*cos - m23*sin;
	m31 = m31*cos - m33*sin;
	m41 = m41*cos - m43*sin;

	m13 = m13*cos + _11*sin;
	m23 = m23*cos + _21*sin;
	m33 = m33*cos + _31*sin;
	m43 = m43*cos + _41*sin;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationZ( SCALAR angleZ )
{
	RotationZ( (SCALAR)sin(angleZ), (SCALAR)cos(angleZ) );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationZ( SCALAR sin, SCALAR cos )
{
	SCALAR _11 = m11;
	SCALAR _21 = m21;
	SCALAR _31 = m31;
	SCALAR _41 = m41;
	m11 = m11*cos + m12*sin;
	m21 = m21*cos + m22*sin;
	m31 = m31*cos + m32*sin;
	m41 = m41*cos + m42*sin;

	m12 = m12*cos - _11*sin;
	m22 = m22*cos - _21*sin;
	m32 = m32*cos - _31*sin;
	m42 = m42*cos - _41*sin;
}

template <class SCALAR>
void Matrix44<SCALAR>::Rotation( SCALAR angleX, SCALAR angleY, SCALAR angleZ )
{
	Matrix44<SCALAR> rotMat;
	Matrix44<SCALAR>::Rotation(&rotMat, angleX, angleY, angleZ);
	(*this) = (*this) * rotMat;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationAxis_vect( mVector3<SCALAR> vect  )
{
	SCALAR deg = vect.getSize();
	vect = vect.normalize();
	RotationAxis((SCALAR)sin(deg), (SCALAR)cos(deg), vect.x, vect.y, vect.z );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationAxis( SCALAR angle, SCALAR x, SCALAR y, SCALAR z )
{
	RotationAxis((SCALAR)sin(angle), (SCALAR)cos(angle), x, y, z );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationAxis( SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z )
{
	Matrix44<SCALAR> rotMat;
	Matrix44<SCALAR>::RotationAxis(&rotMat, sin, cos, x, y, z);
	(*this) = (*this) * rotMat;
}

template <class SCALAR>
void Matrix44<SCALAR>::Translation( SCALAR x, SCALAR y, SCALAR z )
{
	m14 += m11 * x + m12 * y + m13 * z;
	m24 += m21 * x + m22 * y + m23 * z;
	m34 += m31 * x + m32 * y + m33 * z;
}

template <class SCALAR>
void Matrix44<SCALAR>::GetEulerRotation( SCALAR* alpha, SCALAR* beta, SCALAR* gammar )
{
	*beta	= (SCALAR)atan2(-m31, (SCALAR)sqrt(m11*m11 + m21*m21));
	*gammar	= (SCALAR)atan2(m21, m11);
	*alpha	= (SCALAR)atan2(m32, m33);
}

template <class SCALAR>
void Matrix44<SCALAR>::Identity(Matrix44<SCALAR>* m1)
{
	m1->m11 = 1; m1->m12 = 0; m1->m13 = 0; m1->m14 = 0;
	m1->m21 = 0; m1->m22 = 1; m1->m23 = 0; m1->m24 = 0;
	m1->m31 = 0; m1->m32 = 0; m1->m33 = 1; m1->m34 = 0;
	m1->m41 = 0; m1->m42 = 0; m1->m43 = 0; m1->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::Scale(Matrix44<SCALAR>* m1, SCALAR scaleX, SCALAR scaleY, SCALAR scaleZ)
{
	m1->m11 = scaleX; m1->m12 = 0;      m1->m13 = 0;      m1->m14 = 0;
	m1->m21 = 0;      m1->m22 = scaleY; m1->m23 = 0;      m1->m24 = 0;
	m1->m31 = 0;      m1->m32 = 0;      m1->m33 = scaleZ; m1->m34 = 0;
	m1->m41 = 0;      m1->m42 = 0;      m1->m43 = 0;      m1->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationX( Matrix44<SCALAR>* m1, SCALAR angleX )
{
	RotationX( m1, sin(angleX), cos(angleX) );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationX( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos )
{
	m1->m11	= 1;	m1->m12	= 0;			m1->m13	= 0;			m1->m14	= 0;
	m1->m21	= 0;	m1->m22	= (SCALAR)cos;	m1->m23	= -(SCALAR)sin;	m1->m24	= 0;
	m1->m31	= 0;	m1->m32	= (SCALAR)sin;	m1->m33	= (SCALAR)cos;	m1->m34	= 0;
	m1->m41	= 0;	m1->m42	= 0;			m1->m43	= 0;			m1->m44	= 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationY( Matrix44<SCALAR>* m1, SCALAR angleY )
{
	RotationY( m1, sin(angleY), cos(angleY) );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationY( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos )
{
	m1->m11	= (SCALAR)cos;	m1->m12	= 0;	m1->m13	= (SCALAR)sin;	m1->m14	= 0;
	m1->m21	= 0;			m1->m22	= 1;	m1->m23	= 0;			m1->m24	= 0;
	m1->m31	= -(SCALAR)sin;	m1->m32	= 0;	m1->m33	= (SCALAR)cos;	m1->m34	= 0;
	m1->m41	= 0;			m1->m42	= 0;	m1->m43	= 0;			m1->m44	= 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationZ( Matrix44<SCALAR>* m1, SCALAR angleZ )
{
	RotationZ( m1, sin(angleZ), cos(angleZ) );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationZ( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos )
{
	m1->m11	= (SCALAR)cos;	m1->m12	= -(SCALAR)sin;	m1->m13	= 0;	m1->m14	= 0;
	m1->m21	= (SCALAR)sin;	m1->m22	= (SCALAR)cos;	m1->m23	= 0;	m1->m24	= 0;
	m1->m31	= 0;			m1->m32	= 0;			m1->m33	= 1;	m1->m34	= 0;
	m1->m41	= 0;			m1->m42	= 0;			m1->m43	= 0;	m1->m44	= 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::Translation( Matrix44<SCALAR>* m1, SCALAR x, SCALAR y, SCALAR z )
{
	m1->m11	= 1;	m1->m12	= 0;	m1->m13	= 0;	m1->m14	= (SCALAR)x;
	m1->m21	= 0;	m1->m22	= 1;	m1->m23	= 0;	m1->m24	= (SCALAR)y;
	m1->m31	= 0;	m1->m32	= 0;	m1->m33	= 1;	m1->m34	= (SCALAR)z;
	m1->m41	= 0;	m1->m42	= 0;	m1->m43	= 0;	m1->m44	= 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::Rotation( Matrix44<SCALAR>* m1, SCALAR angleX, SCALAR angleY, SCALAR angleZ )
{
	SCALAR sa = (SCALAR)sin((SCALAR)angleX);
	SCALAR ca = (SCALAR)cos((SCALAR)angleX);
	SCALAR sb = (SCALAR)sin((SCALAR)angleY);
	SCALAR cb = (SCALAR)cos((SCALAR)angleY);
	SCALAR sg = (SCALAR)sin((SCALAR)angleZ);
	SCALAR cg = (SCALAR)cos((SCALAR)angleZ);

	m1->m11 = cb*cg;	m1->m12 = cg*sa*sb - ca*sg; m1->m13 = sa*sg + ca*cg*sb;	m1->m14	= 0;
	m1->m21 = cb*sg;	m1->m22 = sa*sb*sg + ca*cg;	m1->m23 = ca*sb*sg - cg*sa;	m1->m24	= 0;
	m1->m31 = -sb;		m1->m32 = cb*sa;			m1->m33 = ca*cb;			m1->m34	= 0;
	m1->m41	= 0;		m1->m42	= 0;				m1->m43	= 0;				m1->m44	= 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationAxis( Matrix44<SCALAR>* m1, SCALAR angle, SCALAR x, SCALAR y, SCALAR z )
{
	RotationAxis( m1, sin(angle), cos(angle), x, y, z );
}

template <class SCALAR>
void Matrix44<SCALAR>::RotationAxis( Matrix44<SCALAR>* m1, SCALAR sin, SCALAR cos, SCALAR x, SCALAR y, SCALAR z )
{
	m1->m11	= 1 + (1 - cos)*(x*x - 1); m1->m12 = -z*sin + (1 - cos)*x*y;  m1->m13 =  y*sin + (1 - cos)*x*z;   m1->m14 = 0;
	m1->m21	=  z*sin + (1 - cos)*x*y;  m1->m22 = 1 + (1 - cos)*(y*y - 1); m1->m23 = -x*sin + (1 - cos)*y*z;   m1->m24 = 0;
	m1->m31	= -y*sin + (1 - cos)*x*z;  m1->m32 =  x*sin + (1 - cos)*y*z;  m1->m33 =  1 + (1 - cos)*(z*z - 1); m1->m34 = 0;
	m1->m41	= 0;                       m1->m42 = 0;                       m1->m43 = 0;                        m1->m44 = 1;
}

template <class SCALAR>
SCALAR Matrix44<SCALAR>::Determinant( const Matrix44<SCALAR>* m )
{
	return
		m->m11*m->m22*m->m33*m->m44 - m->m11*m->m22*m->m34*m->m43 -
		m->m11*m->m32*m->m23*m->m44 + m->m11*m->m32*m->m24*m->m43 +
		m->m11*m->m42*m->m23*m->m34 - m->m11*m->m42*m->m24*m->m33 -
		m->m21*m->m12*m->m33*m->m44 + m->m21*m->m12*m->m34*m->m43 +
		m->m21*m->m32*m->m13*m->m44 - m->m21*m->m32*m->m14*m->m43 -
		m->m21*m->m42*m->m13*m->m34 + m->m21*m->m42*m->m14*m->m33 +
		m->m31*m->m12*m->m23*m->m44 - m->m31*m->m12*m->m24*m->m43 -
		m->m31*m->m22*m->m13*m->m44 + m->m31*m->m22*m->m14*m->m43 +
		m->m31*m->m42*m->m13*m->m24 - m->m31*m->m42*m->m14*m->m23 -
		m->m41*m->m12*m->m23*m->m34 + m->m41*m->m12*m->m24*m->m33 +
		m->m41*m->m22*m->m13*m->m34 - m->m41*m->m22*m->m14*m->m33 -
		m->m41*m->m32*m->m13*m->m24 + m->m41*m->m32*m->m14*m->m23;
}

template <class SCALAR>
void Matrix44<SCALAR>::Adjoint( Matrix44<SCALAR>* result, const Matrix44<SCALAR>* m )
{	
	result->m11 = m->m22*m->m33*m->m44 - m->m22*m->m34*m->m43 - m->m32*m->m23*m->m44 + m->m32*m->m24*m->m43 + m->m42*m->m23*m->m34 - m->m42*m->m24*m->m33;
	result->m12 = -m->m12*m->m33*m->m44 + m->m12*m->m34*m->m43 + m->m32*m->m13*m->m44 - m->m32*m->m14*m->m43 - m->m42*m->m13*m->m34 + m->m42*m->m14*m->m33;
	result->m13 = m->m12*m->m23*m->m44 - m->m12*m->m24*m->m43 - m->m22*m->m13*m->m44 + m->m22*m->m14*m->m43 + m->m42*m->m13*m->m24 - m->m42*m->m14*m->m23;
	result->m14 = -m->m12*m->m23*m->m34 + m->m12*m->m24*m->m33 + m->m22*m->m13*m->m34 - m->m22*m->m14*m->m33 - m->m32*m->m13*m->m24 + m->m32*m->m14*m->m23;
	
	result->m21 = -m->m21*m->m33*m->m44 + m->m21*m->m34*m->m43 + m->m31*m->m23*m->m44 - m->m31*m->m24*m->m43 - m->m41*m->m23*m->m34 + m->m41*m->m24*m->m33;
	result->m22 = m->m11*m->m33*m->m44 - m->m11*m->m34*m->m43 - m->m31*m->m13*m->m44 + m->m31*m->m14*m->m43 + m->m41*m->m13*m->m34 - m->m41*m->m14*m->m33;
	result->m23 = -m->m11*m->m23*m->m44 + m->m11*m->m24*m->m43 + m->m21*m->m13*m->m44 - m->m21*m->m14*m->m43 - m->m41*m->m13*m->m24 + m->m41*m->m14*m->m23;
	result->m24 = m->m11*m->m23*m->m34 - m->m11*m->m24*m->m33 - m->m21*m->m13*m->m34 + m->m21*m->m14*m->m33 + m->m31*m->m13*m->m24 - m->m31*m->m14*m->m23;
	
	result->m31 = m->m21*m->m32*m->m44 - m->m21*m->m34*m->m42 - m->m31*m->m22*m->m44 + m->m31*m->m24*m->m42 + m->m41*m->m22*m->m34 - m->m41*m->m24*m->m32;
	result->m32 = -m->m11*m->m32*m->m44 + m->m11*m->m34*m->m42 + m->m31*m->m12*m->m44 - m->m31*m->m14*m->m42 - m->m41*m->m12*m->m34 + m->m41*m->m14*m->m32;
	result->m33 = m->m11*m->m22*m->m44 - m->m11*m->m24*m->m42 - m->m21*m->m12*m->m44 + m->m21*m->m14*m->m42 + m->m41*m->m12*m->m24 - m->m41*m->m14*m->m22;
	result->m34 = -m->m11*m->m22*m->m34 + m->m11*m->m24*m->m32 + m->m21*m->m12*m->m34 - m->m21*m->m14*m->m32 - m->m31*m->m12*m->m24 + m->m31*m->m14*m->m22;
	
	result->m41 = -m->m21*m->m32*m->m43 + m->m21*m->m33*m->m42 + m->m31*m->m22*m->m43 - m->m31*m->m23*m->m42 - m->m41*m->m22*m->m33 + m->m41*m->m23*m->m32;
	result->m42 = m->m11*m->m32*m->m43 - m->m11*m->m33*m->m42 - m->m31*m->m12*m->m43 + m->m31*m->m13*m->m42 + m->m41*m->m12*m->m33 - m->m41*m->m13*m->m32;
	result->m43 = -m->m11*m->m22*m->m43 + m->m11*m->m23*m->m42 + m->m21*m->m12*m->m43 - m->m21*m->m13*m->m42 - m->m41*m->m12*m->m23 + m->m41*m->m13*m->m22;
	result->m44 = m->m11*m->m22*m->m33 - m->m11*m->m23*m->m32 - m->m21*m->m12*m->m33 + m->m21*m->m13*m->m32 + m->m31*m->m12*m->m23 - m->m31*m->m13*m->m22;
}

template <class SCALAR>
void Matrix44<SCALAR>::Invert( Matrix44<SCALAR>* result, const Matrix44<SCALAR>* m )
{
	SCALAR det = Determinant( m );
	Adjoint( result, m );

	result->m11	/= det;	result->m12	/= det;	result->m13	/= det;	result->m14	/= det;
	result->m21	/= det;	result->m22	/= det;	result->m23	/= det;	result->m24	/= det;
	result->m31	/= det;	result->m32	/= det;	result->m33	/= det;	result->m34	/= det;
	result->m41	/= det;	result->m42	/= det;	result->m43	/= det;	result->m44	/= det;
}

template <class SCALAR>
void Matrix44<SCALAR>::Transpose( Matrix44<SCALAR>* result, const Matrix44<SCALAR>* m )
{
	result->m11	= m->m11;	result->m12	= m->m21;	result->m13	= m->m31;	result->m14	= m->m41;
	result->m21	= m->m12;	result->m22	= m->m22;	result->m23	= m->m32;	result->m24	= m->m42;
	result->m31	= m->m13;	result->m32	= m->m23;	result->m33	= m->m33;	result->m34	= m->m43;
	result->m41	= m->m14;	result->m42	= m->m24;	result->m43	= m->m34;	result->m44	= m->m44;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveFovLH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR aspectRatio, SCALAR znearPlane, SCALAR zfarPlane)
{
	SCALAR h = 1.0f / (SCALAR)tan(fieldOfViewY / 2);
	SCALAR w = h / aspectRatio;

	mat->m11 = w; mat->m12 = 0; mat->m13 = 0;                                  mat->m14 = 0;
	mat->m21 = 0; mat->m22 = h; mat->m23 = 0;                                  mat->m24 = 0;
	mat->m31 = 0; mat->m32 = 0; mat->m33 = zfarPlane/(zfarPlane - znearPlane); mat->m34 = -znearPlane*zfarPlane/(zfarPlane - znearPlane);
	mat->m41 = 0; mat->m42 = 0; mat->m43 = 1;                                  mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveFovRH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR aspectRatio, SCALAR znearPlane, SCALAR zfarPlane)
{
	SCALAR h = 1.0f / (SCALAR)tan(fieldOfViewY / 2);
	SCALAR w = h / aspectRatio;

	mat->m11 = w; mat->m12 = 0; mat->m13 = 0;                                  mat->m14 = 0;
	mat->m21 = 0; mat->m22 = h; mat->m23 = 0;                                  mat->m24 = 0;
	mat->m31 = 0; mat->m32 = 0; mat->m33 = zfarPlane/(znearPlane - zfarPlane); mat->m34 = znearPlane*zfarPlane/(znearPlane - zfarPlane);
	mat->m41 = 0; mat->m42 = 0; mat->m43 = -1;                                 mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveFovXYLH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR fieldOfViewX, SCALAR znearPlane, SCALAR zfarPlane)
{
	SCALAR h = 1.0f / (SCALAR)tan(fieldOfViewY / 2);
	SCALAR w = 1.0f / (SCALAR)tan(fieldOfViewX / 2);

	mat->m11 = w; mat->m12 = 0; mat->m13 = 0;                                  mat->m14 = 0;
	mat->m21 = 0; mat->m22 = h; mat->m23 = 0;                                  mat->m24 = 0;
	mat->m31 = 0; mat->m32 = 0; mat->m33 = zfarPlane/(zfarPlane - znearPlane); mat->m34 = -znearPlane*zfarPlane/(zfarPlane - znearPlane);
	mat->m41 = 0; mat->m42 = 0; mat->m43 = 1;                                  mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveFovXYRH(Matrix44<SCALAR>* mat, SCALAR fieldOfViewY, SCALAR fieldOfViewX, SCALAR znearPlane, SCALAR zfarPlane)
{
	SCALAR h = 1.0f / (SCALAR)tan(fieldOfViewY / 2);
	SCALAR w = 1.0f / (SCALAR)tan(fieldOfViewX / 2);

	mat->m11 = w; mat->m12 = 0; mat->m13 = 0;                                  mat->m14 = 0;
	mat->m21 = 0; mat->m22 = h; mat->m23 = 0;                                  mat->m24 = 0;
	mat->m31 = 0; mat->m32 = 0; mat->m33 = zfarPlane/(znearPlane - zfarPlane); mat->m34 = znearPlane*zfarPlane/(znearPlane - zfarPlane);
	mat->m41 = 0; mat->m42 = 0; mat->m43 = -1;                                 mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveLH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2*znearPlane/width; mat->m12 = 0;                   mat->m13 = 0;                                mat->m14 = 0;
	mat->m21 = 0;                  mat->m22 = 2*znearPlane/height; mat->m23 = 0;                                mat->m24 = 0;
	mat->m31 = 0;                  mat->m32 = 0;                   mat->m33 = zfarPlane/(zfarPlane-znearPlane); mat->m34 = znearPlane*zfarPlane/(znearPlane-zfarPlane);
	mat->m41 = 0;                  mat->m42 = 0;                   mat->m43 = 1;                                mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveRH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2*znearPlane/width; mat->m12 = 0;                   mat->m13 = 0;                                mat->m14 = 0;
	mat->m21 = 0;                  mat->m22 = 2*znearPlane/height; mat->m23 = 0;                                mat->m24 = 0;
	mat->m31 = 0;                  mat->m32 = 0;                   mat->m33 = zfarPlane/(znearPlane-zfarPlane); mat->m34 = znearPlane*zfarPlane/(znearPlane-zfarPlane);
	mat->m41 = 0;                  mat->m42 = 0;                   mat->m43 = -1;                               mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveOffCenterLH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2*znearPlane/(right-left); mat->m12 = 0;                         mat->m13 = (left+right)/(left-right);        mat->m14 = 0;
	mat->m21 = 0;                         mat->m22 = 2*znearPlane/(top-bottom); mat->m23 = (top+bottom)/(bottom-top);        mat->m24 = 0;
	mat->m31 = 0;                         mat->m32 = 0;                         mat->m33 = zfarPlane/(zfarPlane-znearPlane); mat->m34 = znearPlane*zfarPlane/(znearPlane-zfarPlane);
	mat->m41 = 0;                         mat->m42 = 0;                         mat->m43 = 1;                                mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::PerspectiveOffCenterRH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2*znearPlane/(right-left); mat->m12 = 0;                         mat->m13 = (left+right)/(right-left);        mat->m14 = 0;
	mat->m21 = 0;                         mat->m22 = 2*znearPlane/(top-bottom); mat->m23 = (top+bottom)/(top-bottom);        mat->m24 = 0;
	mat->m31 = 0;                         mat->m32 = 0;                         mat->m33 = zfarPlane/(znearPlane-zfarPlane); mat->m34 = znearPlane*zfarPlane/(znearPlane-zfarPlane);
	mat->m41 = 0;                         mat->m42 = 0;                         mat->m43 = -1;                               mat->m44 = 0;
}

template <class SCALAR>
void Matrix44<SCALAR>::OrthoLH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2 / width; mat->m12 = 0;        mat->m13 = 0;                          mat->m14 = 0;
	mat->m21 = 0;         mat->m22 = 2/height; mat->m23 = 0;                          mat->m24 = 0;
	mat->m31 = 0;         mat->m32 = 0;        mat->m33 = 1/(zfarPlane - znearPlane); mat->m34 = znearPlane/(znearPlane - zfarPlane);
	mat->m41 = 0;         mat->m42 = 0;        mat->m43 = 0;                          mat->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::OrthoRH(Matrix44<SCALAR>* mat, SCALAR width, SCALAR height, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2 / width; mat->m12 = 0;        mat->m13 = 0;                          mat->m14 = 0;
	mat->m21 = 0;         mat->m22 = 2/height; mat->m23 = 0;                          mat->m24 = 0;
	mat->m31 = 0;         mat->m32 = 0;        mat->m33 = 1/(znearPlane - zfarPlane); mat->m34 = znearPlane/(znearPlane - zfarPlane);
	mat->m41 = 0;         mat->m42 = 0;        mat->m43 = 0;                          mat->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::OrthoOffCenterLH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2/(right-left); mat->m12 = 0;               mat->m13 = 0;                        mat->m14 = (left+right)/(left-right);
	mat->m21 = 0;              mat->m22 =  2/(top-bottom); mat->m23 = 0;                        mat->m24 = (top+bottom)/(bottom-top);
	mat->m31 = 0;              mat->m32 = 0;               mat->m33 = 1/(zfarPlane-znearPlane); mat->m34 = znearPlane/(znearPlane-zfarPlane);
	mat->m41 = 0;              mat->m42 = 0;               mat->m43 = 0;                        mat->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::OrthoOffCenterRH(Matrix44<SCALAR>* mat, SCALAR left, SCALAR right, SCALAR bottom, SCALAR top, SCALAR znearPlane, SCALAR zfarPlane)
{
	mat->m11 = 2/(right-left); mat->m12 = 0;               mat->m13 = 0;                        mat->m14 = (left+right)/(right-left);
	mat->m21 = 0;              mat->m22 =  2/(top-bottom); mat->m23 = 0;                        mat->m24 = (top+bottom)/(top-bottom);
	mat->m31 = 0;              mat->m32 = 0;               mat->m33 = 1/(znearPlane-zfarPlane); mat->m34 = znearPlane/(znearPlane-zfarPlane);
	mat->m41 = 0;              mat->m42 = 0;               mat->m43 = 0;                        mat->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::LookAtLH(Matrix44<SCALAR>* mat, mVector3<SCALAR> cameraPosition, mVector3<SCALAR> cameraTarget, mVector3<SCALAR> cameraUpVector)
{
	mVector3<SCALAR> zaxis		= (cameraTarget - cameraPosition).normalize();
	mVector3<SCALAR> xaxis		= (cameraUpVector * zaxis).normalize();
	mVector3<SCALAR> yaxis		= zaxis * xaxis;

	mat->m11 = xaxis.x; mat->m12 = xaxis.y; mat->m13 = xaxis.z; mat->m14 = -(cameraPosition ^ xaxis);
	mat->m21 = yaxis.x; mat->m22 = yaxis.y; mat->m23 = yaxis.z; mat->m24 = -(cameraPosition ^ yaxis);
	mat->m31 = zaxis.x; mat->m32 = zaxis.y; mat->m33 = zaxis.z; mat->m34 = -(cameraPosition ^ zaxis);
	mat->m41 = 0;       mat->m42 = 0;       mat->m43 = 0;       mat->m44 = 1;
}

template <class SCALAR>
void Matrix44<SCALAR>::LookAtRH(Matrix44<SCALAR>* mat, mVector3<SCALAR> cameraPosition, mVector3<SCALAR> cameraTarget, mVector3<SCALAR> cameraUpVector)
{
	mVector3<SCALAR> zaxis		= (cameraPosition - cameraTarget).normalize();
	mVector3<SCALAR> xaxis		= (cameraUpVector * zaxis).normalize();
	mVector3<SCALAR> yaxis		= zaxis * xaxis;

	mat->m11 = xaxis.x; mat->m12 = xaxis.y; mat->m13 = xaxis.z; mat->m14 = -(cameraPosition ^ xaxis);
	mat->m21 = yaxis.x; mat->m22 = yaxis.y; mat->m23 = yaxis.z; mat->m24 = -(cameraPosition ^ yaxis);
	mat->m31 = zaxis.x; mat->m32 = zaxis.y; mat->m33 = zaxis.z; mat->m34 = -(cameraPosition ^ zaxis);
	mat->m41 = 0;       mat->m42 = 0;       mat->m43 = 0;       mat->m44 = 1;
}

#pragma endregion

#endif