/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import processing.core.*;
import processing.event.*;
import java.util.List;
import java.util.Random;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.NewtonRaphsonSolver;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.exception.NumberIsTooLargeException;
import static processing.core.PConstants.QUAD_STRIP;
import static processing.core.PConstants.TWO_PI;
/**
 * Implements the medial axis transformation between two curves.
 * @author Sebastian Weiss
 */
public class MedialAxisTransform {
            
        public static class Line {
            public Vector3f p;
            public Vector3f v;
        }
        
        public static class SolverResult implements Cloneable, java.io.Serializable {
            public float[] re ;
            public float[] im;
        }
	
        public static class ClosestInfo implements Cloneable, java.io.Serializable {
            boolean found;
            Vector3f Pt;
            float time;
            Vector3f tangent, dir;
            int curveIndex;      /// Index of the control point
            ClosestInfo () { Pt = tangent = dir = new Vector3f(-1f,-1f,-1f); time = -1; curveIndex = -1; found = false;}
            ClosestInfo (Vector3f Pt_, Vector3f t_, Vector3f d_, float time_, int c) {
                Pt = Pt_; tangent = t_; time = time_; curveIndex = c; dir = d_; found = true;
            }
        }
          
        /**
	 * Represents a point on the medial axis, 
	 */
	public static class TracePoint {
		/**
		 * The position on the medial axis
		 */
		public Vector3f center;
		/**
		 * The radius of the medial axis transformation at this point
		 */
		public float radius;
		/**
		 * The times on the first curve of the closest projection of this point.
		 * <br>
		 * Normally, this array only contains one element, the unique closest projection.
		 * <br>
		 * However, if the two curves are not compatible, the medial axis branches
		 * and there are two or more closest projections. In that case, these times
		 * specify an interval of the curve in that they are not compatible. This
		 * is used in the UI to show the user that he has to change the curve.
		 * <br>
		 * The following must hold:
		 * {@code Curve.interpolate(curveA, projectionOnA[i]).distance(center) == radius} for each i.
		 */
		public float[] projectionOnA;
		/**
		 * The times on the second curve of the closest projection of this point.
		 * <br>
		 * Normally, this array only contains one element, the unique closest projection.
		 * <br>
		 * However, if the two curves are not compatible, the medial axis branches
		 * and there are two or more closest projections. In that case, these times
		 * specify an interval of the curve in that they are not compatible. This
		 * is used in the UI to show the user that he has to change the curve.
		 * <br>
		 * The following must hold:
		 * {@code Curve.interpolate(curveB, projectionOnB[i]).distance(center) == radius} for each i.
		 */
		public float[] projectionOnB;
	}
	
        /**
         * Solve for the intersection parameters s and u for a system
         * of equations such that x1+s*vx1=x2+u*vx2 and y1+s*vy1=y2+u*vy2.
         */
        public static Vector3f solveIntersection (float x1, float vx1, float
                x2, float vx2, float y1, float vy1, float y2, float vy2) {
            float denom = (vx1*vy2 - vx2*vy1);
            if(Math.abs(denom) < 1e-5) return new Vector3f(0.0f, 0.0f, -1.0f);
            float s = -(vy2*x1 - vy2*x2 - vx2*y1 + vx2*y2)/denom;
            float u = -(vy1*x1 - vy1*x2 - vx1*y1 + vx1*y2)/denom;
            return new Vector3f(s, u, 1);
        }

        /**
         * Find the medial axis of two lines defined by a point and a unit vector
         * each. 
         */
        public static Line medialAxisLine (Vector3f p1, Vector3f v1, 
                Vector3f p2, Vector3f v2) {
        
            // Check if the two lines are parallel. If so, return the average
            // of their constant.
            Line maLine = new Line();
            boolean debug = false;
            if((v1.subtract(v2)).length() < 1e-3) {
                maLine.p = (p1.add(p2)).mult(0.5f);
                maLine.v = v1;
                if(debug) System.out.println("Parallel case.");
                return maLine;
            }
            
            // Check if the two lines are skew
            Vector3f u = (v1.cross(v2)).normalize();
            float g = (p2.subtract(p1)).dot(u);
            if(Math.abs(g) > 1e-3) {
                
                // Find the closest point on each line to each other
                // http://2000clicks.com/mathhelp/GeometryPointsAndLines3D.aspx
                Vector3f res = solveIntersection((g * u.x + p1.x), v1.x, p2.x, v2.x, 
                        (g * u.y + p1.y), v1.y, p2.y, v2.y);
                assert(res.z > 0);
                Vector3f p1c = p1.addScale(res.x, v1);
                Vector3f p2c = p2.addScale(res.y, v2);
                maLine.p = (p1c.add(p2c)).mult(0.5f);
                
                // Compute the direction of the bisecting line
                maLine.v = (v1.add(v2)).normalize();
                if(debug) {
                    System.out.println("Skew case.");
                    System.out.println("p1= " + p1.toString() + ", v1= " + v1.toString() + 
                        ", \np2= " + p2.toString() + ", v2= " + v2.toString());
                    System.out.println("Shortest dist1: " + (p1c.subtract(maLine.p)).length());
                    System.out.println("Shortest dist2: " + (p2c.subtract(maLine.p)).length());
                    System.out.println("pm: " + maLine.p.toString());
                }
                return maLine;
            }
            
            // TODO Check for denominator.
            Vector3f res1 = solveIntersection(p1.x, v1.x, p2.x, v2.x, p1.y, v1.y, p2.y, v2.y);
            Vector3f res2 = solveIntersection(p1.x, v1.x, p2.x, v2.x, p1.z, v1.z, p2.z, v2.z);
            Vector3f res3 = solveIntersection(p1.y, v1.y, p2.y, v2.y, p1.z, v1.z, p2.z, v2.z);

            Vector3f res = new Vector3f(0f,0f,0f);
            if(res1.z > 0) res = res1;
            else if(res2.z > 0) res = res2;
            else if(res3.z > 0) res = res3;
            else assert(false);

            // Perform the operation for intersecting lines
            maLine.p = new Vector3f(p1.addScale(res.x, v1));
            maLine.v = new Vector3f((v1.add(v2)).normalize());
            
            if(debug) {
                System.out.println("pm: " + maLine.p.toString());
                System.out.println("Intersecting case.");
            }
            return maLine;
        }
        
        /**
         * Perform Newton's algorithm to find the zero of the derivative wrt time
         * of the perpendicularity constraint between a point and a cubic hermite function. The equation
         * is a quintic, thus no closed form.
         * The function is f(P(t), Q) = dot((Q-P(t)), P'(t));
         */
        public static float perpPointOnCubicHermite (Vector3f P0, Vector3f T0, Vector3f P1, Vector3f T1, Vector3f Q) {
            
            // Get the parameters of the quintic function dfdt
//            float f = - 2*T0.x*(Q.x - P0.x) - 2*T0.y*(Q.y - P0.y) - 2*T0.z*(Q.z - P0.z);
//            float e = 2*(Q.x - P0.x)*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) + 2*(Q.y - P0.y)*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) + 2*(Q.z - P0.z)*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) + 2*T0.x*T0.x + 2*T0.y*T0.y + 2*T0.z*T0.z;
//            float d = - 2*T0.x*(2*T0.x + T1.x + 3*P0.x - 3*P1.x) - 2*T0.y*(2*T0.y + T1.y + 3*P0.y - 3*P1.y) - 2*T0.z*(2*T0.z + T1.z + 3*P0.z - 3*P1.z) - 2*T0.x*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) - 2*T0.y*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) - 2*T0.z*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) - 2*(Q.x - P0.x)*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) - 2*(Q.y - P0.y)*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) - 2*(Q.z - P0.z)*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z);
//            float c = 2*(2*T0.x + T1.x + 3*P0.x - 3*P1.x)*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) + 2*(2*T0.y + T1.y + 3*P0.y - 3*P1.y)*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) + 2*(2*T0.z + T1.z + 3*P0.z - 3*P1.z)*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) + 2*T0.x*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) + 2*T0.y*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) + 2*T0.z*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z) + 2*T0.x*(T0.x + T1.x + 2*P0.x - 2*P1.x) + 2*T0.y*(T0.y + T1.y + 2*P0.y - 2*P1.y) + 2*T0.z*(T0.z + T1.z + 2*P0.z - 2*P1.z);
//            float b = - 2*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x)*(T0.x + T1.x + 2*P0.x - 2*P1.x) - 2*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y)*(T0.y + T1.y + 2*P0.y - 2*P1.y) - 2*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z)*(T0.z + T1.z + 2*P0.z - 2*P1.z) - 2*(2*T0.x + T1.x + 3*P0.x - 3*P1.x)*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) - 2*(2*T0.y + T1.y + 3*P0.y - 3*P1.y)*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) - 2*(2*T0.z + T1.z + 3*P0.z - 3*P1.z)*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z);
//            float a = 2*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x)*(T0.x + T1.x + 2*P0.x - 2*P1.x) + 2*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y)*(T0.y + T1.y + 2*P0.y - 2*P1.y) + 2*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z)*(T0.z + T1.z + 2*P0.z - 2*P1.z);
            float f = T0.x*(Q.x - P0.x) + T0.y*(Q.y - P0.y) + T0.z*(Q.z - P0.z);
            float e = - (Q.x - P0.x)*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) - (Q.y - P0.y)*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) - (Q.z - P0.z)*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) - T0.x*T0.x - T0.y*T0.y - T0.z*T0.z;
            float d = T0.x*(2*T0.x + T1.x + 3*P0.x - 3*P1.x) + T0.y*(2*T0.y + T1.y + 3*P0.y - 3*P1.y) + T0.z*(2*T0.z + T1.z + 3*P0.z - 3*P1.z) + T0.x*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) + T0.y*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) + T0.z*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) + (Q.x - P0.x)*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) + (Q.y - P0.y)*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) + (Q.z - P0.z)*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z);
            float c = - (2*T0.x + T1.x + 3*P0.x - 3*P1.x)*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) - (2*T0.y + T1.y + 3*P0.y - 3*P1.y)*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) - (2*T0.z + T1.z + 3*P0.z - 3*P1.z)*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) - T0.x*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) - T0.y*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) - T0.z*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z) - T0.x*(T0.x + T1.x + 2*P0.x - 2*P1.x) - T0.y*(T0.y + T1.y + 2*P0.y - 2*P1.y) - T0.z*(T0.z + T1.z + 2*P0.z - 2*P1.z);
            float b = (4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x)*(T0.x + T1.x + 2*P0.x - 2*P1.x) + (4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y)*(T0.y + T1.y + 2*P0.y - 2*P1.y) + (4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z)*(T0.z + T1.z + 2*P0.z - 2*P1.z) + (2*T0.x + T1.x + 3*P0.x - 3*P1.x)*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) + (2*T0.y + T1.y + 3*P0.y - 3*P1.y)*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) + (2*T0.z + T1.z + 3*P0.z - 3*P1.z)*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z);
            float a = - (3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x)*(T0.x + T1.x + 2*P0.x - 2*P1.x) - (3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y)*(T0.y + T1.y + 2*P0.y - 2*P1.y) - (3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z)*(T0.z + T1.z + 2*P0.z - 2*P1.z);

            // Get the real roots of the quintic
            SolverResult res = solveQuinticLaguerre(a/f,b/f,c/f,d/f,e/f,1.0f);
            
            // Check which endpoint is closer
            float dist0 = Q.distance(P0);
            float dist1 = Q.distance(P1);
            float minEndpointDist = Math.min(dist0, dist1);
//            System.out.println("Minimum endpoint dist: " + minEndpointDist);
            
            // For each nonimaginary root, with real part within [0,1], compute 
            // the distance to the input point and find the smallest value
            float bestDist = 1e6f;
            float time = -1f;
            for(int i = 0; i < 6; i++) {
                if(Math.abs(res.im[i]) > 1e-3) continue;
                if(res.re[i] > 1 || res.re[i] < 0) continue;
                Vector3f Pt = Curve.cubicHermite(P0,T0,P1,T1,res.re[i]);
                float dist = Q.distance(Pt);
//                System.out.println("\troot t: " + res.re[i] + ", pt: " + Pt.toString() + ", dist:" + dist);
                if(dist < bestDist) {
                    time = res.re[i];
                    bestDist = dist;
                }
            }
            
            return time;
        }

        /**
         * Uses the Laguerre algorithm iteratively to span the range [0,1]
         * and find all the real roots. 
         */
        public static SolverResult solveQuinticLaguerre (float a, float b, float c, float d, float e, float f) {
            
            // Initialize the result
//            System.out.println("quintic params: " + a + ", "+ b + ", "+ c + ", "+ d + ", "+ e + ", "+ f);
            SolverResult res = new SolverResult();
            res.re = new float [6];
            res.im = new float [6];
            for(int i = 0; i < 6; i++) res.im[i] = 1.0f;
            
            // Create the quintic function and the solver
            double coefficients [] = {f,e,d,c,b,a};
            PolynomialFunction qf = new PolynomialFunction(coefficients);
            LaguerreSolver solver = new LaguerreSolver();
                
            // Iteratively call the NewtonRaphson solution until no solution 
            // exists
            double minBound = 0.0;
            double sol;
            int root_idx = 0;
            final double dt = 0.001;
            for(double t = -dt; t < 1.0; t += dt) {
                
                // Check if there is a sign-crossing between the two times
                double t2 = t + dt;
                double f1 = qf.value(t);
                double f2 = qf.value(t2);
                if(Math.signum(f1) == Math.signum(f2)) continue;

                // Attempt to get the solution
                try {
                    sol = solver.solve(100, qf, t, t2);
                }
                catch (TooManyEvaluationsException|NumberIsTooLargeException exc) {
                    break;
                }

                // Save the root and update the minimum bound
                res.re[root_idx] = (float) sol;
                res.im[root_idx] = 0.0f;
                minBound = sol+1e-3;
                root_idx++;
            }
            
//            System.out.println("#quintic roots: " + root_idx);
            return res;
        }
        
        /**
         * Uses the NewtonRaphson algorithm iteratively to span the range [0,1]
         * and find all the real roots. 
         */
        public static SolverResult solveQuinticNewtonRaphson (float a, float b, float c, float d, float e, float f) {
            
            // Initialize the result
            System.out.println("quintic params: " + a + ", "+ b + ", "+ c + ", "+ d + ", "+ e + ", "+ f);
            SolverResult res = new SolverResult();
            res.re = new float [6];
            res.im = new float [6];
            for(int i = 0; i < 6; i++) res.im[i] = 1.0f;
            
            // Create the quintic function and the solver
            QuinticFunction qf = new QuinticFunction (a,b,c,d,e,f);
            NewtonRaphsonSolver solver = new NewtonRaphsonSolver();
                
            // Iteratively call the NewtonRaphson solution until no solution 
            // exists
            double minBound = 0.0;
            double sol;
            int root_idx = 0;
            while(true) {
                
                // Attempt to get the solution
                try {
                    sol = solver.solve(100, qf, minBound, 1.0);
                }
                catch (TooManyEvaluationsException|NumberIsTooLargeException exc) {
                    break;
                }
                
                // Check if the solution is within the bounds (shouldn't be 
                // necessary but it is...)
                if(sol < minBound || sol > 1) break;
                
                // Save the root and update the minimum bound
                res.re[root_idx] = (float) sol;
                res.im[root_idx] = 0.0f;
                minBound = sol+1e-3;
                root_idx++;
            }
            
            System.out.println("#quintic roots: " + root_idx);
            return res;
        }
        
        /**
         * Solves a cubic equation using a closed-form approach.
         * http://stackoverflow.com/questions/13328676/c-solving-cubic-equations
         */
        public static SolverResult solveCubic (float a, float b, float c, float d) {
            
            // Solve quadratic
            if(Math.abs(d) < 1e-5) {
                assert(false);
            }
            
            SolverResult res = new SolverResult();
            res.re = new float[3];
            res.im = new float[3];
            b /= a;
            c /= a;
            d /= a;
            float disc, q, r, dum1, s, t, term1, r13;
            q = (3.0f*c - (b*b))/9.0f;
            r = -(27.0f*d) + b*(9.0f*c - 2.0f*(b*b));
            r /= 54.0f;
            disc = q*q*q + r*r;
            term1 = (b/3.0f);
            res.im[0] = 0.0f;
            if (disc > 0) { // one root real, two are complex
                s = r + (float) Math.sqrt(disc);
                s = (float) ((s < 0) ? -Math.pow(-s, (1.0/3.0)) : Math.pow(s, (1.0/3.0)));
                t = r - (float) Math.sqrt(disc);
                t = (float) ((t < 0) ? -Math.pow(-t, (1.0/3.0)) : Math.pow(t, (1.0/3.0)));
                res.re[0] = -term1 + s + t;
                term1 += (s + t)/2.0;
                res.re[2] = res.re[1] = -term1;
                term1 = (float) Math.sqrt(3.0)*(-t + s)/2;
                res.im[1] = term1;
                res.im[2] = -term1;
                return res;
            } 
            
            // The remaining options are all real
            res.im[1] = res.im[2] = 0;
            if (disc == 0){ // All roots real, at least two are equal.
                r13 = (float) ((r < 0) ? -Math.pow(-r,(1.0/3.0)) : Math.pow(r,(1.0/3.0)));
                res.re[0] = -term1 + 2.0f*r13;
                res.re[0] = res.re[1] = -(r13 + term1);
                return res;
            } 
    
            // Only option left is that all roots are real and unequal (to get here, q < 0)
            q = -q;
            dum1 = q*q*q;
            dum1 = (float) Math.acos(r/Math.sqrt(dum1));
            r13 = (float) (2.0f*Math.sqrt(q));
            res.re[0] = -term1 + (float) (r13*Math.cos(dum1/3.0));
            res.re[1] = -term1 + (float) (r13*Math.cos((dum1 + 2.0*Math.PI)/3.0));
            res.re[2] = -term1 + (float) (r13*Math.cos((dum1 + 4.0*Math.PI)/3.0));
            return res;
        }
        
        /**
         * Find the zero of the derivative wrt time of the perpendicularity
         * constraint between a point and a quadratic hermite function in cubic closed-form.
         * * The function is f(P(t), Q) = dot((Q-P(t)), P'(t));
         */
        public static float perpPointOnQuadraticHermite (Vector3f P0, Vector3f T0, Vector3f P1, Vector3f Q) {
 
            System.out.println("perpPoint call: P0: " + P0.toString() +",T0: " + T0.toString() +", P1: " + P1.toString() +", Q: " + Q.toString());

            // Get the parameters of the cubic function dfdt
//            float d = 2*(Q.x - P0.x)*(T0.x + 2*P0.x - 2*P1.x) + 2*(Q.y - P0.y)*(T0.y + 2*P0.y - 2*P1.y) + 2*(Q.z - P0.z)*(T0.z + 2*P0.z - 2*P1.z);
//            float c = (float) (2*Math.pow(T0.x + 2*P0.x - 2*P1.x,2) + 2*Math.pow(T0.y + 2*P0.y - 2*P1.y,2) + 2*Math.pow(T0.z + 2*P0.z - 2*P1.z,2) - 2*(Q.x - P0.x)*(2*T0.x + 2*P0.x - 2*P1.x) - 2*(Q.y - P0.y)*(2*T0.y + 2*P0.y - 2*P1.y) - 2*(Q.z - P0.z)*(2*T0.z + 2*P0.z - 2*P1.z));
//            float b = (float) (- 2*(T0.x + P0.x - P1.x)*(T0.x + 2*P0.x - 2*P1.x) - 2*(T0.y + P0.y - P1.y)*(T0.y + 2*P0.y - 2*P1.y) - 2*(T0.z + P0.z - P1.z)*(T0.z + 2*P0.z - 2*P1.z) - 2*(T0.x + 2*P0.x - 2*P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) - 2*(T0.y + 2*P0.y - 2*P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) - 2*(T0.z + 2*P0.z - 2*P1.z)*(2*T0.z + 2*P0.z - 2*P1.z));
//            float a = 2*(T0.x + P0.x - P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) + 2*(T0.y + P0.y - P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) + 2*(T0.z + P0.z - P1.z)*(2*T0.z + 2*P0.z - 2*P1.z);
            float d = - (Q.x - P0.x)*(T0.x + 2*P0.x - 2*P1.x) - (Q.y - P0.y)*(T0.y + 2*P0.y - 2*P1.y) - (Q.z - P0.z)*(T0.z + 2*P0.z - 2*P1.z);
            float c = (float) ((Q.x - P0.x)*(2*T0.x + 2*P0.x - 2*P1.x) - Math.pow(T0.y + 2*P0.y - 2*P1.y,2) - Math.pow(T0.z + 2*P0.z - 2*P1.z,2) - Math.pow(T0.x + 2*P0.x - 2*P1.x,2) + (Q.y - P0.y)*(2*T0.y + 2*P0.y - 2*P1.y) + (Q.z - P0.z)*(2*T0.z + 2*P0.z - 2*P1.z));
            float b = (float) ((T0.x + P0.x - P1.x)*(T0.x + 2*P0.x - 2*P1.x) + (T0.y + P0.y - P1.y)*(T0.y + 2*P0.y - 2*P1.y) + (T0.z + P0.z - P1.z)*(T0.z + 2*P0.z - 2*P1.z) + (T0.x + 2*P0.x - 2*P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) + (T0.y + 2*P0.y - 2*P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) + (T0.z + 2*P0.z - 2*P1.z)*(2*T0.z + 2*P0.z - 2*P1.z));
            float a = - (T0.x + P0.x - P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) - (T0.y + P0.y - P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) - (T0.z + P0.z - P1.z)*(2*T0.z + 2*P0.z - 2*P1.z);

            // Compute the roots of the cubic
            SolverResult res = solveCubic(a,b,c,d);
            
            // Check which endpoint is closer
            float dist0 = Q.distance(P0);
            float dist1 = Q.distance(P1);
            float minEndpointDist = Math.min(dist0, dist1);
            // System.out.println("Minimum endpoint dist: " + minEndpointDist);
            
            // For each nonimaginary root, with real part within [0,1], compute 
            // the distance to the input point and find the smallest value
            float bestDist = 1e6f;
            float time = -1f;
            for(int i = 0; i < 3; i++) {
                System.out.println("Root " + i + ": " + res.re[i] + ", " + res.im[i]);
                if(Math.abs(res.im[i]) > 1e-3) continue;
                if(res.re[i] > 1 || res.re[i] < 0) continue;
                Vector3f Pt = Curve.quadraticHermite(P0,T0,P1,res.re[i]);
                float dist = Q.distance(Pt);
                System.out.println("\troot time: " + res.re[i] + ", pt: " + Pt.toString() + ", dist:" + dist);
                if(dist <= (bestDist+1e-3)) {
                    time = res.re[i];
                    bestDist = dist;
                }
            }
            
            System.out.println("Final time: " + time);
            return time;
        }
        
        /**
         * Given a point Q, finds the closest point on the given curve. Returns
         * the point and the tangent to the curve at that point.
         * @param curve
         * @param Q 
         */
        public static ClosestInfo findClosest (Vector3f[] curve, Vector3f Q) { 
            
            System.out.println("\nfindClosest for " + Q.toString() + " -----------------");
            // Go through the curve segment by segment and treat quadratic and
            // cubic segments differently. If the point is found to belong
            // to one of the segments stop.
            int n = curve.length;
            float minDist = 1e6f;
            ClosestInfo info = new ClosestInfo();
            for(int i = 0; i < n-1; i++) {
                
                // First segment
                if (i==0) {
                    Vector3f P0 = curve[0];
                    Vector3f P1 = curve[1];
                    Vector3f T0 = curve[2].subtract(curve[0]).multLocal(Curve.TANGENT_SCALE);
                    float time = perpPointOnQuadraticHermite(P0, T0, P1, Q);
                    // System.out.println("Time for first segment: " + time + ", for Q: " + Q.toString());
                    // System.out.println("\tP0: " + P0.toString() +", P1: " + P1.toString());
                    if(time > -1e-3) {
//                        System.out.println("Returning the first segment.");
                        Vector3f Pt = Curve.quadraticHermite(P0, T0, P1, time);
                        float dist = Pt.distance(Q);
                        if(dist < minDist) {
                            info = new ClosestInfo(Pt, Curve.quadraticHermiteTangent(P0, T0, P1, time), 
                                Q.subtract(Pt).normalize(), time, i);
                            minDist = dist;
                        }
                    }
		} 
                
                // Last segment (had to switch p0 and p1 around)
                else if (i==n-2) {
                    Vector3f P0 = curve[n-2];
                    Vector3f P1 = curve[n-1];
                    Vector3f T0 = curve[n-3].subtract(curve[n-2]).multLocal(Curve.TANGENT_SCALE);
                    float time = perpPointOnQuadraticHermite(P0, T0, P1, Q);
                    if(time > -1e-3) {
                        Vector3f Pt = Curve.quadraticHermite(P0, T0, P1, time);
                        float dist = Pt.distance(Q);
                        if(dist < minDist) {
                            info = new ClosestInfo(Pt, Curve.quadraticHermiteTangent(P0, T0, P1, time), 
                                Q.subtract(Pt).normalize(), time, i);
                            minDist = dist;
                        }
                    }
		}
                
                // Middle segment
                else {
//                    if(true) continue;
                    Vector3f P0 = curve[i];
                    Vector3f P1 = curve[i+1];
                    Vector3f T0 = curve[i+1].subtract(curve[i-1]).multLocal(Curve.TANGENT_SCALE);
                    Vector3f T1 = curve[i+2].subtract(curve[i]).multLocal(Curve.TANGENT_SCALE);
                    float time = perpPointOnCubicHermite(P0, T0, P1, T1, Q);
                    if(time > -1e-3) {
                        Vector3f Pt = Curve.cubicHermite(P0, T0, P1, T1, time);
                        float dist = Pt.distance(Q);
                        if(dist < minDist) {
                            info = new ClosestInfo(Pt, Curve.cubicHermiteTangent(P0, T0, P1, T1, time), 
                                Q.subtract(Pt).normalize(), time, i);
                            minDist = dist;
                        }
                    }
		}
            }
            
//            assert(minDist < 1e6);
            return info;
        }
        
	
       
}
