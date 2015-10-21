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
 * @author Sebastian Weiss
 */
public class TestMedialAxisTransform {

	public TestMedialAxisTransform() {
	}

	/**
	 * Test the computation of the medial axis for two lines.
	 */
	@Test
	public void testMALines() {

		// Perform a number of random tests
		Random rand = new Random();
		for (int i = 0; i < 100; i++) {

			// Generate random points and vectors
			Vector3f p1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
			Vector3f p2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
			Vector3f v1 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
			Vector3f v2 = new Vector3f(rand.nextFloat(), rand.nextFloat(), rand.nextFloat());
			v1 = v1.normalize();
			v2 = v2.normalize();

			p2 = p1.addScale(rand.nextFloat(), v1);

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
