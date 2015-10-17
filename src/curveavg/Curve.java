/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

/**
 * Implements the curve interpolation using piecewise Hermite.
 * @author Sebastian Weiss
 */
public class Curve {
	private static final float EPSILON = 0.001f;
	private static final float TANGENT_SCALE = 1f;
	
	/**
	 * Computes the interpolation at the specified time.
	 * 
	 * The time is given in uniform steps: t=0 is the first point, t=1 is the
	 * last point and all middle point are spaced equally in t (NOT in arc length).
	 * The i-th point is therefore returned by time {@code t=i/(controlPoints.length-1)}.
	 * @param controlPoints the control points
	 * @param time the time from 0 to 1
	 * @return the interpolated vector
	 */
	public static Vector3f interpolate(Vector3f[] controlPoints, float time) {
		int n = controlPoints.length;
		time *= n-1;
		int i = (int) time;
		float frac = time % 1;
		
		if (frac < EPSILON) {
			return controlPoints[i];
		}
		if (i==0) {
			//first part
			Vector3f P0 = controlPoints[0];
			Vector3f P1 = controlPoints[1];
			Vector3f T0 = controlPoints[2].subtract(controlPoints[0]).multLocal(TANGENT_SCALE);
			return quadraticHermite(P0, T0, P1, frac);
		} else if (i==n-2) {
			//last part
			Vector3f P0 = controlPoints[n-1];
			Vector3f P1 = controlPoints[n-2];
			Vector3f T0 = controlPoints[n-3].subtract(controlPoints[n-1]).multLocal(TANGENT_SCALE);
			return quadraticHermite(P0, T0, P1, 1-frac);
		} else {
			//middle
			Vector3f P0 = controlPoints[i];
			Vector3f P1 = controlPoints[i+1];
			Vector3f T0 = controlPoints[i+1].subtract(controlPoints[i-1]).multLocal(TANGENT_SCALE);
			Vector3f T1 = controlPoints[i+2].subtract(controlPoints[i]).multLocal(TANGENT_SCALE);
			return cubicHermite(P0, T0, P1, T1, frac);
		}
	}
	
	/**
	 * Hermite interpolation of the points P0 to P1 at time t=0 to t=1 with the
	 * specified velocities T0 and T1.
	 * @param P0
	 * @param T0
	 * @param P1
	 * @param T1
	 * @param t
	 * @return 
	 */
	public static Vector3f cubicHermite(Vector3f P0, Vector3f T0, Vector3f P1, Vector3f T1, float t) {
		float t2 = t*t;
		float t3 = t2*t;
		Vector3f P = new Vector3f();
		P.addScaleLocal(2*t3 - 3*t2 + 1, P0);
		P.addScaleLocal(t3 - 2*t2 + t, T0);
		P.addScaleLocal(-2*t3 + 3*t2, P1);
		P.addScaleLocal(t3 - t2, T1);
		return P;
	}
	
	/**
	 * A variation of Hermite where a velocity is given only for the first point.
	 * It interpolates P0 at t=0 and P1 at t=1.
	 * @param P0
	 * @param T0
	 * @param P1
	 * @param t
	 * @return 
	 */
	public static Vector3f quadraticHermite(Vector3f P0, Vector3f T0, Vector3f P1, float t) {
		float t2 = t*t;
		Vector3f P = new Vector3f();
		P.addScaleLocal(t2 - 2*t + 1, P0);
		P.addScaleLocal(-t2 + 2*t, P1);
		P.addScaleLocal(t2 - t, T0);
		return P;
	}
}
