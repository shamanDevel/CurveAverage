/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import java.util.Random;
import org.junit.Test;

import static curveavg.MedialAxisTransform.medialAxisLine;
import static org.junit.Assert.*;

/**
 *
 * @author Can Erdogan
 */
public class TestMedialAxisTransform {

    // -------------------------------------------------------------------------
    /// Test the computation of the closest point to a cubic hermite by
    /// sampling the spline and ensuring that all the points are further.
    @Test
    public void testClosestCubicHermite () {
        
        int counter = 0, LIMIT = 100;
        boolean dbg = true;
        while(true) {
            
            // Check number of experiments
            if(counter >= LIMIT) break;
        
            // Generate the Hermite
            Random rand = new Random();
            Vector3f p1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f p2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f v1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f v2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());

            // Generate the random point
            Vector3f q = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            if(dbg) System.out.println("p1= " + p1.toString() + ", v1= " + v1.toString()
               + ", \np2= " + p2.toString() + ", v2= " + v2.toString() + ", \nq= " + q.toString());
            
            // Compute the closest point
            float time = MedialAxisTransform.closestPointOnCubicHermite(p1, v1, p2, v2, q);
            if(time < 0 || time > 1) {
                if(dbg) System.out.println("Bad time, trying again");
                continue;
            }
            counter++;
            
            // Compute the distance
            Vector3f p = Curve.cubicHermite(p1, v1, p2, v2, time);
            float shortestDist = p.distance(q);
            if(dbg) System.out.println("\ttime: " + time + ", pt: " + p.toString() + ", shortest dist: " + shortestDist);
            
            // Sample the curve
            for(float dt = 0.0f; dt <= 1.0f; dt+=0.01f) {
                Vector3f p_ = Curve.cubicHermite(p1, v1, p2, v2, dt);
                float dist = p_.distance(q);
                if(dbg) System.out.println("\tdt: " + dt + ", dist: " + dist);    
                assert(dist > (shortestDist-1e-3));
            }
        }
    }
    
    // -------------------------------------------------------------------------
    /// Test the computation of the closest point to a quadratic hermite by
    /// sampling the spline and ensuring that all the points are further.
    @Test
    public void testClosestQuadraticHermite () {
        
        int counter = 0, LIMIT = 0;
        boolean dbg = false;
        while(true) {
            
            // Check number of experiments
            if(counter >= LIMIT) break;
        
            // Generate the Hermite
            Random rand = new Random();
            Vector3f p1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f p2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f v1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());

            // Generate the random point
            Vector3f q = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            if(dbg) System.out.println("p1= " + p1.toString() + ", v1= " + v1.toString()
               + ", \np2= " + p2.toString() + ", q= " + q.toString());
            
            // Compute the closest point
            float time = MedialAxisTransform.closestPointOnQuadraticHermite(p1, v1, p2, q);
            if(time < 0 || time > 1) {
                if(dbg) System.out.println("Bad time, trying again");
                continue;
            }
            counter++;
            
            // Compute the distance
            Vector3f p = Curve.quadraticHermite(p1, v1, p2, time);
            float shortestDist = p.distance(q);
            if(dbg) System.out.println("\ttime: " + time + ", pt: " + p.toString() + ", shortest dist: " + shortestDist);
            
            // Sample the curve
            for(float dt = 0.0f; dt <= 1.0f; dt+=0.01f) {
                Vector3f p_ = Curve.quadraticHermite(p1, v1, p2, dt);
                float dist = p_.distance(q);
                if(dbg) System.out.println("\tdt: " + dt + ", dist: " + dist);    
                assert(dist > (shortestDist-1e-3));
            }
        }
    }
    
    // -------------------------------------------------------------------------
    /// Test the computation of the medial axis for two lines.
    @Test
    public void testMALines () {

        // Perform a number of random tests
        int LIMIT = 0; // 100
        Random rand = new Random();
        for (int i = 0; i < 0; i++) {

            // Generate random points and vectors
            Vector3f p1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f p2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f v1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            Vector3f v2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
            v1 = v1.normalize();
            v2 = v2.normalize();

            // Make sure the two lines intersect
            if(rand.nextFloat() < 0.2) {
                Vector3f pTemp = p1.addScale(rand.nextFloat(), v1);
                p2 = pTemp.addScale(rand.nextFloat(), v2);
            }

            // Call the function
            MedialAxisTransform.Line l = medialAxisLine(p1, v1, p2, v2);
            System.out.println("p1= " + p1.toString() + ", v1= " + v1.toString()
                            + ", \np2= " + p2.toString() + ", v2= " + v2.toString()
                            + ", \npm= " + l.p.toString() + ", vm= " + l.v.toString());

            // Get a random point on the medial axis and compute its distance
            // to each line
            for (int j = 0; j < 5; j++) {
                Vector3f randPoint = l.p.addScale(rand.nextFloat(), l.v);
                System.out.println("pr=" + randPoint.toString());
                float proj1 = v1.dot(randPoint.subtract(p1));
                float dist1 = (float) Math.sqrt((randPoint.subtract(p1)).lengthSquared() - proj1 * proj1);
                float proj2 = v2.dot(randPoint.subtract(p2));
                float dist2 = (float) Math.sqrt((randPoint.subtract(p2)).lengthSquared() - proj2 * proj2);
                System.out.println("Distances: " + dist1 + ", " + dist2);
                assertEquals(0, Math.abs(dist1-dist2), 1e-3);
            }
        }
    }
}
