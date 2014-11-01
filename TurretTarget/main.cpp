//
//  main.cpp
//  TurretTarget
//
//  Created by James Wucher on 8/17/14.
//  Copyright (c) 2014 James Wucher. All rights reserved.
//

#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <stdexcept>
#include <ostream>
#include <fstream>
#include <limits>
#include <math.h>
#include <float.h>

using namespace std;

typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef signed long long int64;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef double float64;

#define Min(a,b) (a<b?a:b)
#define Max(a,b) (a<b?b:a)

/// A 2D column vector.
struct b2Vec2
{
	/// Default constructor does nothing (for performance).
	b2Vec2() :
   x(0.0),
   y(0.0)
   {
   }
   
	/// Construct using coordinates.
	b2Vec2(float64 x_, float64 y_) : x(x_), y(y_) {}
   
	/// Set this vector to all zeros.
	void SetZero() { x = 0.0; y = 0.0; }
   
	/// Set this vector to some specified coordinates.
	void Set(float64 x_, float64 y_) { x = x_; y = y_; }
   
	/// Negate this vector.
	b2Vec2 operator -() const { b2Vec2 v; v.Set(-x, -y); return v; }
	
	/// Read from and indexed element.
	float64 operator () (int32 i) const
	{
		return (&x)[i];
	}
   
	/// Write to an indexed element.
	float64& operator () (int32 i)
	{
		return (&x)[i];
	}
   
	/// Add a vector to this vector.
	void operator += (const b2Vec2& v)
	{
		x += v.x; y += v.y;
	}
	
	/// Subtract a vector from this vector.
	void operator -= (const b2Vec2& v)
	{
		x -= v.x; y -= v.y;
	}
   
	/// Multiply this vector by a scalar.
	void operator *= (float64 a)
	{
		x *= a; y *= a;
	}
   
	/// Get the length of this vector (the norm).
	float64 Length() const
	{
		return sqrt(x * x + y * y);
	}
   
	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	float64 LengthSquared() const
	{
		return x * x + y * y;
	}
   
   // Calculate the dot product between this vec2 and
   // another one.
   float64 Dot(const b2Vec2& other) const
   {
      return x*other.x + y*other.y;
   }
   
   inline static b2Vec2 FromPolar(float64 radius, float64 angleRads)
   {
      return b2Vec2(radius*cos(angleRads),radius*sin(angleRads));
   }
   
	/// Convert this vector into a unit vector. Returns the length.
	float64 Normalize()
	{
		float64 length = Length();
		if (length < DBL_MIN*2)
		{
			return 0.0;
		}
		float64 invLength = 1.0 / length;
		x *= invLength;
		y *= invLength;
      
		return length;
	}
   
	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	b2Vec2 Skew() const
	{
		return b2Vec2(-y, x);
	}
   
   float64 AngleRads() const { return atan2(y,x); }
   float64 AngleDegs() const { return atan2(y,x)*180/M_PI; }
   
   inline friend std::ostream& operator<<(std::ostream& out, const b2Vec2& s)
   {
      out << "(" << s.x << "," << s.y << ")";
      return out;
   }
   
   std::string ToString() const
   {
      char buffer[1024];
      sprintf(buffer,"(%lf,%lf)",x,y);
      return std::string(buffer);
   }
   
	float64 x, y;
};

/// Perform the dot product on two vectors.
inline float64 b2Dot(const b2Vec2& a, const b2Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float64 b2Cross(const b2Vec2& a, const b2Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross(const b2Vec2& a, float64 s)
{
	return b2Vec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross(float64 s, const b2Vec2& a)
{
	return b2Vec2(-s * a.y, s * a.x);
}

/// Add two vectors component-wise.
inline b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(a.x + b.x, a.y + b.y);
}

/// Subtract two vectors component-wise.
inline b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(a.x - b.x, a.y - b.y);
}

inline b2Vec2 operator * (float64 s, const b2Vec2& a)
{
	return b2Vec2(s * a.x, s * a.y);
}

inline bool operator == (const b2Vec2& a, const b2Vec2& b)
{
	return a.x == b.x && a.y == b.y;
}

inline float64 b2Distance(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = a - b;
	return c.Length();
}

inline float64 b2DistanceSquared(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = a - b;
	return b2Dot(c, c);
}


typedef b2Vec2 Vec2;

/* Calculate the future position of a moving target so that 
 * a turret can turn to face the position and fire a projectile.
 *
 * This algorithm works by "guessing" an intial time of impact
 * for the projectile 0.5*(tMin + tMax).  It then calculates
 * the position of the target at that time and computes what the 
 * time for the turret to rotate to that position (tRot0) and
 * the flight time of the projectile (tFlight).  The algorithms
 * drives the difference between tImpact and (tFlight + tRot) to 
 * zero using a binary search. 
 *
 * The "solution" returned by the algorithm is the impact 
 * location.  The shooter should rotate to towards this 
 * position and fire immediately.
 *
 * The algorithm will fail (and return false) under the 
 * following conditions:
 * 1. The target is out of range.  It is possible that the 
 *    target is out of range only for a short time but in
 *    range the rest of the time, but this seems like an 
 *    unnecessary edge case.  The turret is assumed to 
 *    "react" by checking range first, then plot to shoot.
 * 2. The target is heading away from the shooter too fast
 *    for the projectile to reach it before tMax.
 * 3. The solution cannot be reached in the number of steps
 *    allocated to the algorithm.  This seems very unlikely
 *    since the default value is 40 steps.
 *
 *  This algorithm uses a call to sqrt and atan2, so it 
 *  should NOT be run continuously.
 *
 *  On the other hand, nominal runs show convergence usually
 *  in about 7 steps, so this may be a good 'do a step per
 *  frame' calculation target.
 *
 */
bool CalculateInterceptShotPosition(const Vec2& pShooter,
                                    const Vec2& vShooter,
                                    const Vec2& pSFacing0,
                                    const Vec2& pTarget0,
                                    const Vec2& vTarget,
                                    float64 sProjectile,
                                    float64 wShooter,
                                    float64 maxDist,
                                    Vec2& solution,
                                    float64 tMax = 4.0,
                                    float64 tMin = 0.0
                                    )
{
   cout << "----------------------------------------------" << endl;
   cout << " Starting Calculation [" << tMin << "," << tMax << "]" << endl;
   cout << "----------------------------------------------" << endl;
   
   float64 tImpact = (tMin + tMax)/2;
   float64 tImpactLast = tImpact;
   // Tolerance in seconds
   float64 SOLUTION_TOLERANCE_SECONDS = 0.01;
   const int MAX_STEPS = 40;
   for(int idx = 0; idx < MAX_STEPS; idx++)
   {
      // Calculate the position of the target at time tImpact.
      Vec2 pTarget = pTarget0 + tImpact*vTarget;
      // Calulate the angle between the shooter and the target
      // when the impact occurs.
      Vec2 toTarget = pTarget - pShooter;
      float64 dist = toTarget.Length();
      Vec2 pSFacing = (pTarget - pShooter);
      float64 pShootRots = pSFacing.AngleRads();
      float64 tRot = fabs(pShootRots)/wShooter;
      float64 tFlight = dist/sProjectile;
      float64 tShot = tImpact - (tRot + tFlight);
      cout << "Iteration: " << idx
      << " tMin: " << tMin
      << " tMax: " << tMax
      << " tShot: " << tShot
      << " tImpact: " << tImpact
      << " tRot: " << tRot
      << " tFlight: " << tFlight
      << " Impact: " << pTarget.ToString()
      << endl;
      if(dist >= maxDist)
      {
         cout << "FAIL:  TARGET OUT OF RANGE (" << dist << "m >= " << maxDist << "m)" << endl;
         return false;
      }
      tImpactLast = tImpact;
      if(tShot > 0.0)
      {
         tMax = tImpact;
         tImpact = (tMin + tMax)/2;
      }
      else
      {
         tMin = tImpact;
         tImpact = (tMin + tMax)/2;
      }
      if(fabs(tImpact - tImpactLast) < SOLUTION_TOLERANCE_SECONDS)
      {  // WE HAVE A WINNER!!!
         solution = pTarget;
         return true;
      }
   }
   return false;
}

int main(int argc, const char * argv[])
{
   // Initial Conditions
   Vec2 pShooter(7.0,0.0);       // m
   Vec2 pTarget0(0.0,5.0);       // m
   Vec2 vTarget(3.0,-1.0);       // m/s
   Vec2 vShooter(0.0,0.0);       // m/s
   Vec2 pSFacing0(1,0);          // m
   float64 sProjectile = 8.0;    // m/s
   float64 wShooter = M_PI/2;    // rads/s
   float64 tMin = 0.0;           // Min Firing Time
   float64 tMax = 10.0;          // Max Firing Time
   const float64 MAX_DIST = 10.0;
   
   const int32 STEPS_PER_SEC = 100;
   float64 pShootRots0 = pSFacing0.AngleRads();
   Vec2 solution;

   bool foundSolution = CalculateInterceptShotPosition(pShooter,
                                                       vShooter,
                                                       pSFacing0,
                                                       pTarget0,
                                                       vTarget,
                                                       sProjectile,
                                                       wShooter,
                                                       MAX_DIST,
                                                       solution);

   if(!foundSolution)
   {
      cout << "----------------------------------------------" << endl;
      cout << " NO SOLUTION FOUND!!!" << endl;
      cout << "----------------------------------------------" << endl;
   }
   else
   {
      // Shooting variables.
      Vec2 pShot = solution;
      Vec2 pSFacing1 = (pShot - pShooter);
      const float64 COLL_DIST = 0.25;
      const float64 COLL_DIST_SQ = COLL_DIST*COLL_DIST;
      pSFacing1.Normalize();
      float64 pShootRots1 = pSFacing1.AngleRads();
      // Positive or negative rotation.
      float64 wMult = (pShootRots1 - pShootRots0) > 0? 1.0 : -1.0;
      cout << "psFacing0: " << pSFacing0 << " (" << pSFacing0.AngleDegs() << " degs)" << endl;
      cout << "pSFacing1: " << pSFacing1 << " (" << pSFacing1.AngleDegs() << " degs)" << endl;
      
      cout << "----------------------------------------------" << endl;
      cout << " Starting Simulation [" << tMin << "," << tMax << "]" << endl;
      cout << " Intercept Position: " << solution.ToString() << endl;
      cout << "----------------------------------------------" << endl;
      
      Vec2 pSFacing = pSFacing0;
      Vec2 pProjectile = pShooter;
      float64 fireTime = 0.0;
      bool rotating = true;
      bool fired = false;
      bool hitTarget = false;
      
      for(int idx = 0; idx <= tMax*STEPS_PER_SEC && !hitTarget; ++idx)
      {
         float64 now = idx * 1.0/STEPS_PER_SEC;
         // Update the position of the target.
         Vec2 pTarget = pTarget0 + now*vTarget;
         printf("[%05d] [%.2f sec] ",idx,now);
         // Update the position of the rotation.
         if(rotating)
         {
            if(fabs(pSFacing.AngleDegs() - pSFacing1.AngleDegs()) > 1.0)
            {
               pSFacing.x = cos(pShootRots0 + now*wMult*wShooter);
               pSFacing.y = sin(pShootRots0 + now*wMult*wShooter);
               cout << "pSFacing: " << pSFacing.AngleDegs() << " degs ";
            }
            else
            {
               cout << endl;
               cout << "----------------------------------------------" << endl;
               cout << " FIRING PROJETILE " << endl;
               cout << "----------------------------------------------" << endl;
               rotating = false;
               fired = true;
               fireTime = now;
               pSFacing.Normalize();
            }
         }
         if(fired)
         {
            pProjectile = pShooter + sProjectile*(now - fireTime)*pSFacing;
            cout << "pProjectile: " << pProjectile.ToString() << " ";
            cout << "pTarget: " << pTarget.ToString() << " ";
            cout << "DIST: " << (pTarget-pProjectile).Length() << " meters";
            cout << endl;
            if(!hitTarget && (pProjectile - pTarget).LengthSquared() < COLL_DIST_SQ)
            {
               hitTarget = true;
               cout << "----------------------------------------------" << endl;
               cout << " HIT TARGET!!!!! " << endl;
               cout << "----------------------------------------------" << endl;
            }
         }
         else
         {
            cout << "pTarget: " << pTarget.ToString() << " ";
            cout << endl;
            
         }
      }
   }
}

