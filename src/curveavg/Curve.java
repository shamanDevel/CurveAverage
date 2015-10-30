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
	public static final float EPSILON = 0.001f;
	public static final float TANGENT_SCALE = 0.5f;
	
	//TODO: compute the curve interpolation uniform distributed over the arc-length
	
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
		time = Math.min(1, Math.max(0, time));
		int n = controlPoints.length;
		time *= n-1;
		int i = (int) time;
		float frac = time % 1;
		// --> time = (i+frac) / (n-1)
		
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
	 * Interpolates uniformly in time t, not equispaced in arc length!
	 * @param controlPoints
	 * @param numSamples
	 * @return 
	 */
	public static Vector3f[] interpolateUniformly(Vector3f[] controlPoints, int numSamples) {
		Vector3f[] samples = new Vector3f[numSamples];
		float step = 1f / (numSamples-1);
		for (int i=0; i<numSamples; ++i) {
			samples[i] = interpolate(controlPoints, i*step);
		}
		return samples;
	}
	
	/**
	 * Approximates the arc length of the interpolated curve given by the control
	 * points from the given start time to the given end time.
	 * @param controlPoints the control points of the curve
	 * @param start the start time
	 * @param end the end time
	 * @param numSamples the resultion used to approximate the arc length
	 * @return the arc length
	 * @see #interpolate(curveavg.Vector3f[], float) 
	 */
	public static float computeArcLength(Vector3f[] controlPoints, float start, float end, int numSamples) {
		float step = (end-start) / numSamples;
		float len = 0;
		Vector3f P1 = interpolate(controlPoints, start);
		for (int i=1; i<=numSamples; ++i) {
			Vector3f P2 = interpolate(controlPoints, start + i*step);
			len += P2.distance(P1);
			P1 = P2;
		}
		return len;
	}
	
	/**
	 * Produces a equispaced sampling in arc length of the curve interpolation. 
	 * @param controlPoints the control points
	 * @param numSamples the number of samples
	 * @return equispaced samplings of the interpolated curve.
	 * @see #interpolate(curveavg.Vector3f[], float) 
	 */
	public static Vector3f[] interpolateEquispacedArcLength(Vector3f[] controlPoints, int numSamples) {
		//First step: Oversample curve to calculate the total arc length
		Vector3f[] samples1 = new Vector3f[numSamples * 2];
		float step1 = samples1.length-1;
		for (int i=0; i<samples1.length; ++i) {
			samples1[i] = interpolate(controlPoints, i/step1);
		}
		//Second step: calculate cumulative distances between samples and sum them
		float[] distances = new float[samples1.length];
		float arcLength = 0;
		distances[0] = 0;
		for (int i=1; i<distances.length; ++i) {
			distances[i] = samples1[i].distance(samples1[i-1]);
			distances[i] += distances[i-1];
		}
		arcLength = distances[distances.length-1];
		//Third step: sample final curve equispaced
		Vector3f[] samples2 = new Vector3f[numSamples];
		samples2[0] = controlPoints[0]; //fix first and last point
		samples2[numSamples-1] = controlPoints[controlPoints.length-1];
		int j=0;
		float step2 = arcLength / (numSamples-1);
		for (int i=1; i<numSamples-1; ++i) {
			float d = i*step2;
			//search j so that distances[j]<=d<distances[j+1]
			while (j<distances.length-1) {
				if (distances[j]<=d && distances[j+1]>=d) {
					break;
				}
				j++;
			}
			float t = (j + (d-distances[j])/(distances[j+1]-distances[j])) / step1;
			samples2[i] = interpolate(controlPoints, t);
		}
		return samples2;
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
		P.addScaledLocal(2*t3 - 3*t2 + 1, P0);
		P.addScaledLocal(t3 - 2*t2 + t, T0);
		P.addScaledLocal(-2*t3 + 3*t2, P1);
		P.addScaledLocal(t3 - t2, T1);
		return P;
	}
	

	
	/**
	 * A variation of Hermite where a velocity is given only for the first point.
	 * It interpolates P0 at t=0 and P1 at t=1.
         * P(t) = (t^2 - 2*t + 1) * P0 + (-t^2 + 2*t) * P1 + (t^2 - t) * T0
	 * @param P0
	 * @param T0
	 * @param P1
	 * @param t
	 * @return 
	 */
	public static Vector3f quadraticHermite(Vector3f P0, Vector3f T0, Vector3f P1, float t) {
		float t2 = t*t;
		Vector3f P = new Vector3f();
		P.addScaledLocal(t2 - 2*t + 1, P0);
		P.addScaledLocal(-t2 + 2*t, P1);
		P.addScaledLocal(t2 - t, T0);
		return P;
	}
        
        /**
	 * Tangent for a quadratic Hermite.
         * P'(t) = (2*t - 2) * P0 + (-2*t + 2) * P1 + (2*t - 1) * T0
	 */
	public static Vector3f quadraticHermiteTangent(Vector3f P0, Vector3f T0, Vector3f P1, float t) {
            Vector3f T = new Vector3f();
            T.addScaledLocal(2*t - 2, P0);
            T.addScaledLocal(-2*t + 2, P1);
            T.addScaledLocal(2*t-1, T0);
            Vector3f Pt1 = quadraticHermite(P0, T0, P1, t);
            Vector3f Pt2 = quadraticHermite(P0, T0, P1, t+1e-5f);
            Vector3f T2 = (Pt2.subtract(Pt1)).normalize();
//            System.out.println("P0: " + P0.toString() + ", P1: " + P1.toString() + ", T0: " + T0.toString() + ", time: " + t + ", T: " + T.toString() + ", T2: " + T2.toString());
            return T.normalize();
	}
        
        /**
	 * Tangent for a cubic Hermite.
         * P(t) = (2*t^3 - 3*t^2 + 1) * P0 + (t^3-2*t^2+t) * T0 + (-2*t^3+3*t^2) * P1 + (t^3-t^2) * T1
         * P'(t) = (6*t^2 - 6*t) * P0 + (3*t^2-4*t+1) * T0 + (-6*t^2+6*t) * P1 + (3*t^2-2*t) * T1
	 */
	public static Vector3f cubicHermiteTangent(Vector3f P0, Vector3f T0, Vector3f P1, Vector3f T1, float t) {
		float t2 = t*t;
		Vector3f T = new Vector3f();
		T.addScaledLocal(6*t2-6*t, P0);
		T.addScaledLocal(3*t2 - 4*t + 1, T0);
		T.addScaledLocal(-6*t2 + 6*t, P1);
		T.addScaledLocal(3*t2 - 2*t, T1);
		return T.normalize();
	}
	
}
