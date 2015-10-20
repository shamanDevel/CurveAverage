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
         * Test the computation of the medial axis for two lines.
         */
        public static void testMALines () {
            
            // Perform a number of random tests
            Random rand = new Random();
            for(int i = 0; i < 100; i++) {
                
                // Generate random points and vectors
                Vector3f p1 = new Vector3f (rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
                Vector3f p2 = new Vector3f (rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
                Vector3f v1 = new Vector3f (rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
                Vector3f v2 = new Vector3f (rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
                v1 = v1.normalize();
                v2 = v2.normalize();
                
                // Call the function
                Line l = medialAxisLine(p1,v1,p2,v2);
                System.out.println("p1= " + p1.toString() + ", v1= " + v1.toString() + 
                        ", \np2= " + p2.toString() + ", v2= " + v2.toString() + 
                        ", \npm= " + l.p.toString() + ", vm= " + l.v.toString());
                
                // Get a random point on the medial axis and compute its distance
                // to each line
                for(int j = 0; j < 5; j++) {
                    Vector3f randPoint = l.p.addScale(rand.nextFloat(), l.v);
                    System.out.println("pr=" + randPoint.toString());
                    float proj1 = v1.dot(randPoint.subtract(p1));
                    float dist1 = (float) Math.sqrt((randPoint.subtract(p1)).lengthSquared() - proj1 * proj1);
                    float proj2 = v2.dot(randPoint.subtract(p2));
                    float dist2 = (float) Math.sqrt((randPoint.subtract(p2)).lengthSquared() - proj2 * proj2);
                    System.out.println("Distances: " + dist1 + ", " + dist2);
                    assert(Math.abs(dist1-dist2) < 1e-5);
                }
            }
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
		//TODO
            System.out.println("Hi");
            testMALines();
	}
}
