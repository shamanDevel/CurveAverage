/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import java.util.Random;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 *
 * @author Sebastian Weiss
 */
public class CircularArcTest {
	private final Random RAND = new Random(1);
	
	public CircularArcTest() {
	}
	
	private float randFloat(float min, float max) {
		float v = RAND.nextFloat();
		v *= (max-min);
		v += min;
		return v;
	}
	
	@Test
	public void testDegeneratedArc() {
		for (int i=0; i<20; ++i) {
			Vector3f A = new Vector3f(randFloat(-10, 10), randFloat(-10, 10), randFloat(-10, 10));
			Vector3f B = new Vector3f(randFloat(-10, 10), randFloat(-10, 10), randFloat(-10, 10));
			Vector3f P = new Vector3f();
			P.interpolateLocal(A, B, 0.5f);
			CircularArc arc = new CircularArc(P, A, B);
			assertTrue(arc.isDegenerated());
			assertEquals(A, arc.getPointOnArc(0));
			assertEquals(B, arc.getPointOnArc(1));
			assertEquals(P, arc.getPointOnArc(0.5f));
		}
	}
	
	@Test
	public void testRandomPoints() {
		for (int i=0; i<50; ++i) {
			//create A,B randomly on a sphere around P
			Vector3f P = new Vector3f(randFloat(-50, 50), randFloat(-50, 50), randFloat(-50, 50));
			Vector3f PA = new Vector3f(randFloat(-10, 10), randFloat(-10, 10), randFloat(-10, 10));
			Vector3f PB = new Vector3f(randFloat(-10, 10), randFloat(-10, 10), randFloat(-10, 10));
			PA.normalizeLocal();
			PB.normalizeLocal();
			float r = randFloat(0.1f, 50);
			Vector3f A = P.addScaled(r, PA);
			Vector3f B = P.addScaled(r, PB);
			//create circular arc
			CircularArc arc = new CircularArc(P, A, B);
			if (arc.isDegenerated()) {
				assertEquals(0, PA.cross(PB).length(), 1e-4f);
				continue;
			}
			System.out.println(arc);
			//test it
			Vector3f C = arc.getCenter();
			assertEquals("center not equidistant to A", arc.getRadius(), A.distance(C), 1e-4f);
			assertEquals("center not equidistant to B", arc.getRadius(), B.distance(C), 1e-4f);
			assertEquals("Does not interpolate A", A, arc.getPointOnArc(0));
			assertEquals("Does not interpolate B", B, arc.getPointOnArc(1));
		}
	}
}
