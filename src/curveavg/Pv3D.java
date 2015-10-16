/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import processing.core.*;
import static java.lang.Math.*;

/**
 *
 * @author Sebastian Weiss
 */
public class Pv3D {
	// points, vectors, frames in 3D

	static class vec {

		float x = 0, y = 0, z = 0;

		vec() {
		}

		; 
		vec(float px, float py, float pz) {
			x = px;
			y = py;
			z = pz;
		}

		;
		vec(float px, float py) {
			x = px;
			y = py;
		}

		;
		vec set(float px, float py, float pz) {
			x = px;
			y = py;
			z = pz;
			return this;
		}

		; 
		vec setTo(vec V) {
			x = V.x;
			y = V.y;
			z = V.z;
			return this;
		}

		; 
		vec set(vec V) {
			x = V.x;
			y = V.y;
			z = V.z;
			return this;
		}

		; 
		vec add(vec V) {
			x += V.x;
			y += V.y;
			z += V.z;
			return this;
		}

		;
		vec add(float s, vec V) {
			x += s * V.x;
			y += s * V.y;
			z += s * V.z;
			return this;
		}

		;
		vec sub(vec V) {
			x -= V.x;
			y -= V.y;
			z -= V.z;
			return this;
		}

		;
		vec mul(float f) {
			x *= f;
			y *= f;
			z *= f;
			return this;
		}

		;
		vec div(float f) {
			x /= f;
			y /= f;
			z /= f;
			return this;
		}

		;
		vec div(int f) {
			x /= f;
			y /= f;
			z /= f;
			return this;
		}

		;
		vec rev() {
			x = -x;
			y = -y;
			z = -z;
			return this;
		}

		;
		float norm() {
			return (float) (sqrt(x * x + y * y + z * z));
		}

		; 
		vec normalize() {
			float n = norm();
			if (n > 0.000001) {
				div(n);
			};
			return this;
		}

		;
		vec rotate(float a, vec I, vec J) { // Rotate this by angle a parallel in plane (I,J) Assumes I and J are orthogonal
			float x = d(this, I), y = d(this, J); // dot products
			float c = (float) cos(a), s = (float) sin(a);
			add(x * c - x - y * s, I);
			add(x * s + y * c - y, J);
			return this;
		}
	;

	} // end class vec
  
static class pt {

		float x = 0, y = 0, z = 0;

		pt() {
		}

		; 
		pt(float px, float py) {
			x = px;
			y = py;
		}

		;
		pt(float px, float py, float pz) {
			x = px;
			y = py;
			z = pz;
		}

		;
		pt set(float px, float py, float pz) {
			x = px;
			y = py;
			z = pz;
			return this;
		}

		; 
		pt set(pt P) {
			x = P.x;
			y = P.y;
			z = P.z;
			return this;
		}

		; 
		pt setTo(pt P) {
			x = P.x;
			y = P.y;
			z = P.z;
			return this;
		}

		; 
		pt setTo(float px, float py, float pz) {
			x = px;
			y = py;
			z = pz;
			return this;
		}

		; 
		pt add(pt P) {
			x += P.x;
			y += P.y;
			z += P.z;
			return this;
		}
		
		pt add(float x, float y, float z) {
			this.x += x;
			this.y += y;
			this.z += z;
			return this;
		}

		;
		pt add(vec V) {
			x += V.x;
			y += V.y;
			z += V.z;
			return this;
		}

		;
		pt sub(vec V) {
			x -= V.x;
			y -= V.y;
			z -= V.z;
			return this;
		}

		;
		pt add(float s, vec V) {
			x += s * V.x;
			y += s * V.y;
			z += s * V.z;
			return this;
		}

		;
		pt sub(pt P) {
			x -= P.x;
			y -= P.y;
			z -= P.z;
			return this;
		}

		;
		pt mul(float f) {
			x *= f;
			y *= f;
			z *= f;
			return this;
		}

		;
		pt div(float f) {
			x /= f;
			y /= f;
			z /= f;
			return this;
		}

		;
		pt div(int f) {
			x /= f;
			y /= f;
			z /= f;
			return this;
		}
	;

	}
   
// =====  vector functions
static vec V() {
		return new vec();
	}

	;                                                                          // make vector (x,y,z)
static vec V(float x, float y, float z) {
		return new vec(x, y, z);
	}

	;                                            // make vector (x,y,z)
static vec V(vec V) {
		return new vec(V.x, V.y, V.z);
	}

	;                                                          // make copy of vector V
static vec A(vec A, vec B) {
		return new vec(A.x + B.x, A.y + B.y, A.z + B.z);
	}

	;                                       // A+B
static vec A(vec U, float s, vec V) {
		return V(U.x + s * V.x, U.y + s * V.y, U.z + s * V.z);
	}

	;                               // U+sV
static vec M(vec U, vec V) {
		return V(U.x - V.x, U.y - V.y, U.z - V.z);
	}

	;                                              // U-V
static vec M(vec V) {
		return V(-V.x, -V.y, -V.z);
	}

	;                                                              // -V
static vec V(vec A, vec B) {
		return new vec((A.x + B.x) / 2.0f, (A.y + B.y) / 2.0f, (A.z + B.z) / 2.0f);
	}                      // (A+B)/2

	static vec V(vec A, float s, vec B) {
		return new vec(A.x + s * (B.x - A.x), A.y + s * (B.y - A.y), A.z + s * (B.z - A.z));
	}

	;      // (1-s)A+sB
static vec V(vec A, vec B, vec C) {
		return new vec((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
	}

	;  // (A+B+C)/3
static vec V(vec A, vec B, vec C, vec D) {
		return V(V(A, B), V(C, D));
	}

	;                                         // (A+B+C+D)/4
static vec V(float s, vec A) {
		return new vec(s * A.x, s * A.y, s * A.z);
	}

	;                                           // sA
static vec V(float a, vec A, float b, vec B) {
		return A(V(a, A), V(b, B));
	}                                       // aA+bB 

	static vec V(float a, vec A, float b, vec B, float c, vec C) {
		return A(V(a, A, b, B), V(c, C));
	}                   // aA+bB+cC

	static vec V(pt P, pt Q) {
		return new vec(Q.x - P.x, Q.y - P.y, Q.z - P.z);
	}

	;                                          // PQ
static vec U(vec V) {
		float n = V.norm();
		if (n < 0.0000001) {
			return V(0, 0, 0);
		} else {
			return V.div(n);
		}
	}

	;             // V/||V||
static vec U(pt P, pt Q) {
		return U(V(P, Q));
	}

	;                                                                 // PQ/||PQ||
static vec U(float x, float y, float z) {
		return U(V(x, y, z));
	}

	;                                               // make vector (x,y,z)
static vec N(vec U, vec V) {
		return V(U.y * V.z - U.z * V.y, U.z * V.x - U.x * V.z, U.x * V.y - U.y * V.x);
	}

	;                  // UxV cross product (normal to both)
static vec N(pt A, pt B, pt C) {
		return N(V(A, B), V(A, C));
	}

	;                                                   // normal to triangle (A,B,C), not normalized (proportional to area)
static vec B(vec U, vec V) {
		return U(N(N(U, V), U));
	}

	static vec Normal(vec V) {
		if (abs(V.z) <= min(abs(V.x), abs(V.y))) {
			return V(-V.y, V.x, 0);
		}
		if (abs(V.x) <= min(abs(V.z), abs(V.y))) {
			return V(0, -V.z, V.y);
		}
		return V(V.z, 0, -V.x);
	}

// ===== point functions
	static pt P() {
		return new pt();
	}

	;                                                                          // point (x,y,z)
static pt P(float x, float y, float z) {
		return new pt(x, y, z);
	}

	;                                            // point (x,y,z)
static pt P(float x, float y) {
		return new pt(x, y);
	}

	;                                                       // make point (x,y)
static pt P(pt A) {
		return new pt(A.x, A.y, A.z);
	}

	;                                                           // copy of point P
static pt P(pt A, float s, pt B) {
		return new pt(A.x + s * (B.x - A.x), A.y + s * (B.y - A.y), A.z + s * (B.z - A.z));
	}

	;        // A+sAB
static pt L(pt A, float s, pt B) {
		return new pt(A.x + s * (B.x - A.x), A.y + s * (B.y - A.y), A.z + s * (B.z - A.z));
	}

	;        // A+sAB
static pt P(pt A, pt B) {
		return P((A.x + B.x) / 2.0f, (A.y + B.y) / 2.0f, (A.z + B.z) / 2.0f);
	}                             // (A+B)/2

	static pt P(pt A, pt B, pt C) {
		return new pt((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
	}

	;     // (A+B+C)/3
static pt P(pt A, pt B, pt C, pt D) {
		return P(P(A, B), P(C, D));
	}

	;                                            // (A+B+C+D)/4
static pt P(float s, pt A) {
		return new pt(s * A.x, s * A.y, s * A.z);
	}

	;                                            // sA
static pt A(pt A, pt B) {
		return new pt(A.x + B.x, A.y + B.y, A.z + B.z);
	}

	;                                         // A+B
static pt P(float a, pt A, float b, pt B) {
		return A(P(a, A), P(b, B));
	}                                        // aA+bB 

	static pt P(float a, pt A, float b, pt B, float c, pt C) {
		return A(P(a, A), P(b, B, c, C));
	}                     // aA+bB+cC 

	static pt P(float a, pt A, float b, pt B, float c, pt C, float d, pt D) {
		return A(P(a, A, b, B), P(c, C, d, D));
	}   // aA+bB+cC+dD

	static pt P(pt P, vec V) {
		return new pt(P.x + V.x, P.y + V.y, P.z + V.z);
	}                                 // P+V

	static pt P(pt P, float s, vec V) {
		return new pt(P.x + s * V.x, P.y + s * V.y, P.z + s * V.z);
	}                           // P+sV

	static pt P(pt O, float x, vec I, float y, vec J) {
		return P(O.x + x * I.x + y * J.x, O.y + x * I.y + y * J.y, O.z + x * I.z + y * J.z);
	}  // O+xI+yJ

	static pt P(pt O, float x, vec I, float y, vec J, float z, vec K) {
		return P(O.x + x * I.x + y * J.x + z * K.x, O.y + x * I.y + y * J.y + z * K.y, O.z + x * I.z + y * J.z + z * K.z);
	}  // O+xI+yJ+kZ

	static void makePts(pt[] C) {
		for (int i = 0; i < C.length; i++) {
			C[i] = P();
		}
	}

// ===== measures
	static float d(vec U, vec V) {
		return U.x * V.x + U.y * V.y + U.z * V.z;
	}

	;                                            //U*V dot product
static float dot(vec U, vec V) {
		return U.x * V.x + U.y * V.y + U.z * V.z;
	}

	;                                            //U*V dot product
static float det2(vec U, vec V) {
		return -U.y * V.x + U.x * V.y;
	}

	;                                       // U|V det product
static float det3(vec U, vec V) {
		return (float) sqrt(d(U, U) * d(V, V) - (d(U, V) * d(U, V)));
	}

	;                                       // U|V det product
static float m(vec U, vec V, vec W) {
		return d(U, N(V, W));
	}

	;                                                 // (UxV)*W  mixed product, determinant
static float m(pt E, pt A, pt B, pt C) {
		return m(V(E, A), V(E, B), V(E, C));
	}                                    // det (EA EB EC) is >0 when E sees (A,B,C) clockwise

	static float n2(vec V) {
		return V.x * V.x + V.y * V.y + V.z * V.z;
	}

	;                                                   // V*V    norm squared
static float n(vec V) {
		return (float) sqrt(n2(V));
	}

	;                                                                // ||V||  norm
static float d(pt P, pt Q) {
		return (float) sqrt((Q.x - P.x) * (Q.x - P.x) + (Q.y - P.y) * (Q.y - P.y) + (Q.z - P.z) * (Q.z - P.z));
	}

	;                            // ||AB|| distance
static float area(pt A, pt B, pt C) {
		return n(N(A, B, C)) / 2;
	}

	;                                               // area of triangle 
static float volume(pt A, pt B, pt C, pt D) {
		return m(V(A, B), V(A, C), V(A, D)) / 6;
	}

	;                           // volume of tet 
static boolean parallel(vec U, vec V) {
		return n(N(U, V)) < n(U) * n(V) * 0.00001;
	}                              // true if U and V are almost parallel

	static float angle(vec U, vec V) {
		return (float) acos(d(U, V) / n(V) / n(U));
	}

	;                                       // angle(U,V)
static boolean cw(vec U, vec V, vec W) {
		return m(U, V, W) > 0;
	}

	;                                               // (UxV)*W>0  U,V,W are clockwise
static boolean cw(pt A, pt B, pt C, pt D) {
		return volume(A, B, C, D) > 0;
	}

	;                                     // tet is oriented so that A sees B, C, D clockwise 
static boolean projectsBetween(pt P, pt A, pt B) {
		return dot(V(A, P), V(A, B)) > 0 && dot(V(B, P), V(B, A)) > 0;
	}

	;
static float disToLine(pt P, pt A, pt B) {
		return det3(U(A, B), V(A, P));
	}

	;
static pt projectionOnLine(pt P, pt A, pt B) {
		return P(A, dot(V(A, B), V(A, P)) / dot(V(A, B), V(A, B)), V(A, B));
	}

// ===== rotate 
	static vec R(vec V) {
		return V(-V.y, V.x, V.z);
	} // rotated 90 degrees in XY plane

	static pt R(pt P, float a, vec I, vec J, pt G) {
		float x = d(V(G, P), I), y = d(V(G, P), J);
		float c = (float) cos(a), s = (float) sin(a);
		return P(P, x * c - x - y * s, I, x * s + y * c - y, J);
	}

	; // Rotated P by a around G in plane (I,J)
static vec R(vec V, float a, vec I, vec J) {
		float x = d(V, I), y = d(V, J);
		float c = (float) cos(a), s = (float) sin(a);
		return A(V, V(x * c - x - y * s, I, x * s + y * c - y, J));
	}

	; // Rotated V by a parallel to plane (I,J)
static pt R(pt Q, pt C, pt P, pt R) { // returns rotated version of Q by angle(CP,CR) parallel to plane (C,P,R)
		vec I0 = U(C, P), I1 = U(C, R), V = V(C, Q);
		float c = d(I0, I1), s = (float) sqrt(1. - c*c);
		if (abs(s) < 0.00001) {
			return Q;
		}
		vec J0 = V(1f / s, I1, -c / s, I0);
		vec J1 = V(-s, I0, c, J0);
		float x = d(V, I0), y = d(V, J0);
		//  stroke(red); show(C,400,I0); stroke(blue); show(C,400,I1); stroke(orange); show(C,400,J0); stroke(magenta); show(C,400,J1); noStroke();
		return P(Q, x, M(I1, I0), y, M(J1, J0));
	}

	static pt R(pt Q, float a) {
		float dx = Q.x, dy = Q.y, c = (float) cos(a), s = (float) sin(a);
		return P(c * dx + s * dy, -s * dx + c * dy, Q.z);
	}

	;  // Q rotated by angle a around the origin
static pt R(pt Q, float a, pt C) {
		float dx = Q.x - C.x, dy = Q.y - C.y, c = (float) cos(a), s = (float) sin(a);
		return P(C.x + c * dx - s * dy, C.y + s * dx + c * dy, Q.z);
	}
;  // Q rotated by angle a around point P

}
