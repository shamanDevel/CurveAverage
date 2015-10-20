/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import java.util.List;
import java.util.Random;

/**
 * Implements the medial axis transformation between two curves.
 * @author Sebastian Weiss
 */
public class MedialAxisTransform {
	
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
        public static Vector2f solveIntersection (float x1, float vx1, float
                x2, float vx2, float y1, float vy1, float y2, float vy2) {
            float denom = (vx1*vy2 - vx2*vy1);
            assert(Math.abs(denom) > 1e-5);
            float s = -(vy2*x1 - vy2*x2 - vx2*y1 + vx2*y2)/denom;
            float u = -(vy1*x1 - vy1*x2 - vx1*y1 + vx1*y2)/denom;
            return new Vector2f(s, u);
        }
        
        public static class Line {
            public Vector3f p;
            public Vector3f v;
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
                Vector2f res = solveIntersection((g * u.x + p1.x), v1.x, p2.x, v2.x, 
                        (g * u.y + p1.y), v1.y, p2.y, v2.y);
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
            Vector2f res1 = solveIntersection(p1.x, v1.x, p2.x, v2.x, p1.y, v1.y, p2.y, v2.y);
            Vector2f res2 = solveIntersection(p1.x, v1.x, p2.x, v2.x, p1.z, v1.z, p2.z, v2.z);
            Vector2f res3 = solveIntersection(p1.y, v1.y, p2.y, v2.y, p1.z, v1.z, p2.z, v2.z);
            System.out.println("res1: " + res1.toString());
            System.out.println("res2: " + res2.toString());
            System.out.println("res3: " + res3.toString());
            
            // Perform the operation for intersecting lines
            maLine.p = new Vector3f(p1.addScale(res1.x, v1));
            maLine.v = new Vector3f((v1.add(v2)).normalize());
            
            if(debug) {
                System.out.println("pm: " + maLine.p.toString());
                System.out.println("Intersecting case.");
            }
            return maLine;
        }
        
        /**
         * Perform Newton's algorithm to find the zero of the derivative wrt time
         * of the distance between a point and a cubic hermite function. The equation
         * is a quintic, thus no closed form.
         */
        public static float closestPointOnCubicHermite () {
            return 0f;
        }
        
        public static class CubicResult implements Cloneable, java.io.Serializable {
            public float[] re ;
            public float[] im;
        }

        /**
         * http://stackoverflow.com/questions/13328676/c-solving-cubic-equations
         */
        public static CubicResult solveCubic (float a, float b, float c, float d) {
            
            // Solve quadratic
            if(Math.abs(d) < 1e-5) {
                assert(false);
            }
            
            CubicResult res = new CubicResult();
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
         * Find the zero of the derivative wrt time of the distance between a 
         * point and a quadratic hermite function in cubic closed-form.
         * Use Matlab to find the coefficients of the distance function.
         * Matlab: clear all;
            syms x0 y0 z0 x1 y1 z1 t0x t0y t0z t qx qy qz real;
            p0 = [x0; y0; z0]; p1 = [x1;y1;z1]; t0 = [t0x; t0y; t0z]; q = [qx;qy;qz];
            pt = (t^2 - 2*t + 1) * p0 + (-t^2 + 2*t) * p1 + (t^2 - t) * t0;
            ft = (q-pt)'*(q-pt);
            dftdt = diff(ft,t);
            c = coeffs(dftdt,t)
         */
        public static float closestPointOnQuadraticHermite (Vector3f P0, Vector3f P1, Vector3f T0, Vector3f Q) {
 
            // Get the parameters of the cubic function dfdt
            float d = 2*(Q.x - P0.x)*(T0.x + 2*P0.x - 2*P1.x) + 2*(Q.y - P0.y)*(T0.y + 2*P0.y - 2*P1.y) + 2*(Q.z - P0.z)*(T0.z + 2*P0.z - 2*P1.z);
            float c = (float) (2*Math.pow(T0.x + 2*P0.x - 2*P1.x,2) + 2*Math.pow(T0.y + 2*P0.y - 2*P1.y,2) + 2*Math.pow(T0.z + 2*P0.z - 2*P1.z,2) - 2*(Q.x - P0.x)*(2*T0.x + 2*P0.x - 2*P1.x) - 2*(Q.y - P0.y)*(2*T0.y + 2*P0.y - 2*P1.y) - 2*(Q.z - P0.z)*(2*T0.z + 2*P0.z - 2*P1.z));
            float b = (float) (- 2*(T0.x + P0.x - P1.x)*(T0.x + 2*P0.x - 2*P1.x) - 2*(T0.y + P0.y - P1.y)*(T0.y + 2*P0.y - 2*P1.y) - 2*(T0.z + P0.z - P1.z)*(T0.z + 2*P0.z - 2*P1.z) - 2*(T0.x + 2*P0.x - 2*P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) - 2*(T0.y + 2*P0.y - 2*P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) - 2*(T0.z + 2*P0.z - 2*P1.z)*(2*T0.z + 2*P0.z - 2*P1.z));
            float a = 2*(T0.x + P0.x - P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) + 2*(T0.y + P0.y - P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) + 2*(T0.z + P0.z - P1.z)*(2*T0.z + 2*P0.z - 2*P1.z);

            // Compute the roots of the cubic
            CubicResult res = solveCubic(a,b,c,d);
            
            // For each nonimaginary root, with real part within [0,1], compute 
            // the distance to the input point and find the smallest value
            float bestDist = 1e6f;
            float time = -1f;
            for(int i = 0; i < 3; i++) {
                if(Math.abs(res.im[i]) > 1e-3) continue;
                if(res.re[i] > 1 || res.re[i] < 0) continue;
                Vector3f Pt = Curve.quadraticHermite(P0,P1,T0,res.re[i]);
                float dist = Q.distance(Pt);
                if(dist < bestDist) {
                    time = res.re[i];
                    bestDist = dist;
                }
            }
            
            return time;
        }
        
	/**
	 * Traces the medial axis of the two curves {@code curveA} and {@code curveB}.
	 * <br>
	 * Both curves start and end in the same points ({@code curveA[0]=curveB[0]}
	 * and {@code curveA[curveA.length-1]=curveB[curveB.length-1]}).
	 * The curves are interpolated using {@link Curve#interpolate(curveavg.Vector3f[], float) }.
	 * <br>
	 * The traced points on the medial axis are represented by instances of the class
	 * {@link TracePoint}. Place the points in the provided output list.
	 * @param curveA the control points of the first curve.
	 * @param curveB the control points of the second curve
	 * @param output an empty list, place the traced point in here
	 */
	public static void trace(Vector3f[] curveA, Vector3f[] curveB, List<TracePoint> output) {
		
            // Start with the common point 
            Vector3f point = curveA[0];
            
            while(true) {
                
                // Find which segment of the curves the point coincides to
                
            }
                
	}
}
