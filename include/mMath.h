/**
 * @file mMath.h
 * @author Chawin Ophaswongse
 * @date 2026-02-23
 * @version 1.0.1
 * @brief General math utilities for embedded and desktop targets
 *
 * @details
 * Provides template math utilities including a quadratic equation solver
 * and a trigonometric equation solver.
 *
 * **Quadratic Solver**
 * Solves ax^2 + bx + c = 0 and returns real roots only.
 * hasRealRoots is false and x1/x2 are NAN when the discriminant is negative.
 * @code
 *   QuadraticResultf r = solveQuadratic(1.0f, -3.0f, 2.0f);
 *   // r.hasRealRoots == true, r.x1 == 2.0, r.x2 == 1.0
 *
 *   QuadraticResultf r2 = solveQuadratic(1.0f, 0.0f, 1.0f);
 *   // r2.hasRealRoots == false, r2.x1 == NAN, r2.x2 == NAN
 * @endcode
 *
 * **Printing – Quadratic (Arduino)**
 * @code
 *   printQuadraticResult(1.0f, -3.0f, 2.0f, r);
 * @endcode
 *
 * **Printing – Quadratic (desktop / standard C++)**
 * @code
 *   std::cout << r << std::endl;
 * @endcode
 *
 * **Trigonometric Equation Solver**
 * Solves A*cos(θ) + B*sin(θ) = C numerically using the Weierstrass
 * half-angle substitution t = tan(θ/2).
 * Returns 0, 1, or 2 solutions in radians stored in TrigResult.
 * A solution exists only when A² + B² ≥ C².
 * @code
 *   TrigResultf r = solveTrigEquation(1.0f, 0.0f, 0.5f);
 *   // r.count == 2, r.theta[0] and r.theta[1] in radians
 *
 *   TrigResultf r2 = solveTrigEquation(0.0f, 0.0f, 2.0f);
 *   // r2.count == 0  (no solution: 0² + 0² < 2²)
 * @endcode
 *
 * **Printing – TrigEq (Arduino)**
 * @code
 *   printTrigResult(A, B, C, r);
 * @endcode
 *
 * **Printing – TrigEq (desktop / standard C++)**
 * @code
 *   std::cout << r << std::endl;
 * @endcode
 */

#ifndef _M_MATH_H_
#define _M_MATH_H_

#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
using namespace std;
#endif

// ---------------------------------------------------------------------------
#pragma region QuadraticResult
// ---------------------------------------------------------------------------

/**
 * @brief Holds the two real roots of a quadratic equation.
 *
 * hasRealRoots is true  : x1 and x2 contain valid real roots.
 * hasRealRoots is false : discriminant < 0; x1 and x2 are NAN.
 */
template <class SCALAR>
struct QuadraticResult
{
	bool   hasRealRoots;
	SCALAR x1;
	SCALAR x2;

	QuadraticResult() :
		hasRealRoots(false),
		x1((SCALAR)NAN),
		x2((SCALAR)NAN) { }

#ifdef IOSTREAM
	friend ostream& operator << (ostream& os, const QuadraticResult<SCALAR>& r)
	{
		if (r.hasRealRoots) {
			os << "x1 = " << r.x1 << endl;
			os << "x2 = " << r.x2 << endl;
		} else {
			os << "No real roots (discriminant < 0)" << endl;
		}
		return os;
	}
#endif
};

typedef QuadraticResult<float>  QuadraticResultf;
typedef QuadraticResult<double> QuadraticResultd;

// ---------------------------------------------------------------------------
// solveQuadratic  –  solves ax^2 + bx + c = 0, real roots only
// ---------------------------------------------------------------------------

/**
 * @brief Solve the quadratic equation ax^2 + bx + c = 0 for real roots.
 *
 * @param a  Coefficient of x^2 (must not be zero)
 * @param b  Coefficient of x
 * @param c  Constant term
 * @return   QuadraticResult<SCALAR>; x1/x2 are NAN if no real roots exist
 */
template <class SCALAR>
QuadraticResult<SCALAR> solveQuadratic(SCALAR a, SCALAR b, SCALAR c)
{
	QuadraticResult<SCALAR> result;

	if (a == (SCALAR)0) {
#ifdef IOSTREAM
	cerr << "[Quadratic] Error: 'a' must not be zero." << endl;
#elif defined(ARDUINO)
	Serial.println("[Quadratic] Error: 'a' must not be zero.");
#endif
		return result;
	}

	SCALAR discriminant = b * b - (SCALAR)4 * a * c;

	if (discriminant < (SCALAR)0) {
		result.hasRealRoots = false;
		result.x1 = (SCALAR)NAN;
		result.x2 = (SCALAR)NAN;
	} else if (discriminant == (SCALAR)0) {
		result.hasRealRoots = true;
		result.x1 = -b / ((SCALAR)2 * a);  // repeated root
		result.x2 = result.x1;
	} else {
		SCALAR sqrtD      = (SCALAR)sqrt((double)discriminant);
		result.hasRealRoots = true;
		result.x1 = (-b + sqrtD) / ((SCALAR)2 * a);
		result.x2 = (-b - sqrtD) / ((SCALAR)2 * a);
	}

	return result;
}

// ---------------------------------------------------------------------------
// printQuadraticResult  –  Arduino Serial output helper
// ---------------------------------------------------------------------------

#ifdef ARDUINO
/**
 * @brief Print the quadratic equation and its real roots over Serial.
 *
 * @param a  Coefficient of x^2
 * @param b  Coefficient of x
 * @param c  Constant term
 * @param r  Result from solveQuadratic()
 */
template <class SCALAR>
void printQuadraticResult(SCALAR a, SCALAR b, SCALAR c, const QuadraticResult<SCALAR>& r)
{
	Serial.print("[Quadratic] ");
	Serial.print(a); Serial.print("x^2 + ");
	Serial.print(b); Serial.print("x + ");
	Serial.print(c); Serial.println(" = 0");

	if (r.hasRealRoots) {
		Serial.print("  x1 = "); Serial.println(r.x1, 6);
		Serial.print("  x2 = "); Serial.println(r.x2, 6);
	} else {
		Serial.println("  No real roots (discriminant < 0)");
	}
}
#endif

#pragma endregion

// ---------------------------------------------------------------------------
#pragma region TrigResult
// ---------------------------------------------------------------------------

/**
 * @brief Holds up to two solutions of A*cos(θ) + B*sin(θ) = C (in radians).
 *
 * count == 0 : no solution (A² + B² < C², or numerical instability)
 * count == 1 : one unique solution in theta[0]
 * count == 2 : two solutions in theta[0] and theta[1]
 */
template <class SCALAR>
struct TrigResult
{
	int    count;
	SCALAR theta[2];

	TrigResult() : count(0) {
		theta[0] = (SCALAR)NAN;
		theta[1] = (SCALAR)NAN;
	}

#ifndef ARDUINO
	friend ostream& operator << (ostream& os, const TrigResult<SCALAR>& r)
	{
		if (r.count == 0) {
			os << "No solution (A^2 + B^2 < C^2)" << endl;
		} else {
			for (int i = 0; i < r.count; i++)
				os << "theta[" << i << "] = " << r.theta[i] << " rad" << endl;
		}
		return os;
	}
#endif
};

typedef TrigResult<float>  TrigResultf;
typedef TrigResult<double> TrigResultd;

// ---------------------------------------------------------------------------
// solveTrigEquation  –  solves A*cos(θ) + B*sin(θ) = C
// ---------------------------------------------------------------------------

/**
 * @brief Solve A*cos(θ) + B*sin(θ) = C numerically.
 *
 * Uses the Weierstrass half-angle substitution t = tan(θ/2), which converts
 * the equation into the quadratic (A+C)t² - 2Bt + (C-A) = 0, solved
 * internally via solveQuadratic(). Solutions exist only when A² + B² ≥ C².
 *
 * @param A  Cosine coefficient
 * @param B  Sine coefficient
 * @param C  Right-hand side constant
 * @return   TrigResult<SCALAR> with count = 0, 1, or 2 and theta[] in radians
 */
template <class SCALAR>
TrigResult<SCALAR> solveTrigEquation(SCALAR A, SCALAR B, SCALAR C)
{
	TrigResult<SCALAR> result;

	// Special case: A + C ≈ 0  →  leading coefficient of the quadratic is zero
	if ((SCALAR)fabs((double)(A + C)) < (SCALAR)1e-10) {
		if ((SCALAR)fabs((double)B) < (SCALAR)1e-10)
			return result;  // fully degenerate
		SCALAR t     = (C - A) / ((SCALAR)2 * B);
		SCALAR denom = (SCALAR)1 + t * t;
		result.count    = 1;
		result.theta[0] = (SCALAR)atan2((double)(((SCALAR)2 * t) / denom),
		                                (double)(((SCALAR)1 - t * t) / denom));
		return result;
	}

	// Weierstrass substitution: (A+C)t² - 2Bt + (C-A) = 0
	QuadraticResult<SCALAR> tRoots = solveQuadratic(A + C, (SCALAR)(-2) * B, C - A);

	if (!tRoots.hasRealRoots)
		return result;  // no solution: A² + B² < C²

	// Recover theta from each t via half-angle formulas
	SCALAR tol      = (SCALAR)1e-6;
	SCALAR t_vals[2]     = { tRoots.x1, tRoots.x2 };
	SCALAR theta_vals[2] = { (SCALAR)NAN, (SCALAR)NAN };
	int    valid    = 0;

	for (int i = 0; i < 2; i++) {
		SCALAR t     = t_vals[i];
		SCALAR denom = (SCALAR)1 + t * t;
		SCALAR sinT  = ((SCALAR)2 * t) / denom;
		SCALAR cosT  = ((SCALAR)1 - t * t) / denom;

		// Numerical stability: sin²+cos² must be ≈ 1
		if ((SCALAR)fabs((double)(sinT*sinT + cosT*cosT - (SCALAR)1)) > tol) {
#ifdef ARDUINO
			Serial.println("[TrigEq] Warning: numerical instability detected.");
#else
			cerr << "[TrigEq] Warning: numerical instability detected." << endl;
#endif
			return result;
		}

		theta_vals[valid++] = (SCALAR)atan2((double)sinT, (double)cosT);
	}

	// Deduplicate if both roots map to the same angle
	if (valid == 2 && (SCALAR)fabs((double)(theta_vals[0] - theta_vals[1])) < (SCALAR)1e-10) {
		result.count    = 1;
		result.theta[0] = theta_vals[0];
	} else {
		result.count    = valid;
		result.theta[0] = theta_vals[0];
		result.theta[1] = theta_vals[1];
	}

	return result;
}

// ---------------------------------------------------------------------------
// printTrigResult  –  Arduino Serial output helper
// ---------------------------------------------------------------------------

#ifdef ARDUINO
/**
 * @brief Print the trig equation and its solutions over Serial.
 *
 * @param A  Cosine coefficient
 * @param B  Sine coefficient
 * @param C  Right-hand side constant
 * @param r  Result from solveTrigEquation()
 */
template <class SCALAR>
void printTrigResult(SCALAR A, SCALAR B, SCALAR C, const TrigResult<SCALAR>& r)
{
	Serial.print("[TrigEq] ");
	Serial.print(A); Serial.print("*cos(t) + ");
	Serial.print(B); Serial.print("*sin(t) = ");
	Serial.println(C);

	if (r.count == 0) {
		Serial.println("  No solution (A^2 + B^2 < C^2)");
	} else {
		for (int i = 0; i < r.count; i++) {
			Serial.print("  theta["); Serial.print(i);
			Serial.print("] = "); Serial.print(r.theta[i], 6);
			Serial.println(" rad");
		}
	}
}
#endif

#pragma endregion

#endif // _M_MATH_H_
