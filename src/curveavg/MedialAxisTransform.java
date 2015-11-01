/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.analysis.solvers.NewtonRaphsonSolver;
import org.apache.commons.math3.exception.NumberIsTooLargeException;
import org.apache.commons.math3.exception.TooManyEvaluationsException;

/**
 * Implements the medial axis transformation between two curves.
 *
 * @author Sebastian Weiss
 */
public class MedialAxisTransform {
	private static final Logger LOG = Logger.getLogger(MedialAxisTransform.class.getName());
        private static final float STEPSIZE = 1f;
        private static final int MAX_MA_ITERATION = 500;
        
	public static class Line {

		public Vector3f p;
		public Vector3f v;
	}

	public static class SolverResult implements Cloneable, java.io.Serializable {

		public float[] re;
		public float[] im;
	}

	public static class ClosestInfo implements Cloneable, java.io.Serializable {

		boolean found;
		Vector3f Pt;
		float time;
		Vector3f tangent, dir;
		int curveIndex;      /// Index of the control point

		ClosestInfo() {
			Pt = tangent = dir = new Vector3f(-1f, -1f, -1f);
			time = -1;
			curveIndex = -1;
			found = false;
		}

		ClosestInfo(Vector3f Pt_, Vector3f t_, Vector3f d_, float time_, int c) {
			Pt = Pt_;
			tangent = t_;
			time = time_;
			curveIndex = c;
			dir = d_;
			found = true;
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
		 * Normally, this array only contains one element, the unique closest
		 * projection.
		 * <br>
		 * However, if the two curves are not compatible, the medial axis
		 * branches and there are two or more closest projections. In that case,
		 * these times specify an interval of the curve in that they are not
		 * compatible. This is used in the UI to show the user that he has to
		 * change the curve.
		 * <br>
		 * The following must hold:
		 * {@code Curve.interpolate(curveA, projectionOnA[i]).distance(center) == radius}
		 * for each i.
		 */
		public float[] projectionOnA;
		/**
		 * The times on the second curve of the closest projection of this
		 * point.
		 * <br>
		 * Normally, this array only contains one element, the unique closest
		 * projection.
		 * <br>
		 * However, if the two curves are not compatible, the medial axis
		 * branches and there are two or more closest projections. In that case,
		 * these times specify an interval of the curve in that they are not
		 * compatible. This is used in the UI to show the user that he has to
		 * change the curve.
		 * <br>
		 * The following must hold:
		 * {@code Curve.interpolate(curveB, projectionOnB[i]).distance(center) == radius}
		 * for each i.
		 */
		public float[] projectionOnB;

		public TracePoint() {
		}

		public TracePoint(Vector3f center, float radius, float[] projectionOnA, float[] projectionOnB) {
			this.center = center;
			this.radius = radius;
			this.projectionOnA = projectionOnA;
			this.projectionOnB = projectionOnB;
		}

		public TracePoint(Vector3f center, float radius, float projectionOnA, float projectionOnB) {
			this.center = center;
			this.radius = radius;
			this.projectionOnA = new float[]{projectionOnA};
			this.projectionOnB = new float[]{projectionOnB};
		}
	}

	/**
	 * Traces the medial axis of the two curves {@code curveA} and
	 * {@code curveB}. The medial axis are sampled in that way that the distances
	 * traveled on the first curve plus the distance traveled on the second curve
	 * stays constant from sample to sample.
	 * <br>
	 * Both curves start and end in the same points ({@code curveA[0]=curveB[0]}
	 * and {@code curveA[curveA.length-1]=curveB[curveB.length-1]}). The curves
	 * are interpolated using {@link Curve#interpolate(curveavg.Vector3f[], float)
	 * }.
	 * <br>
	 * The traced points on the medial axis are represented by instances of the
	 * class {@link TracePoint}. Place the points in the provided output list.
	 *
	 * @param curveA the control points of the first curve.
	 * @param curveB the control points of the second curve
	 * @param resolution the expected resolution of the target curve
	 * @param output an empty list, place the traced point in here
	 */
	public static void geodesicTrace(Vector3f[] curveA, Vector3f[] curveB,
			int resolution, List<TracePoint> output) {
		//trace it normally
		List<TracePoint> traceList = new ArrayList<>();
		trace(curveA, curveB, traceList);
		TracePoint[] trace = traceList.toArray(new TracePoint[traceList.size()]);
		System.out.println("pre-trace produced "+trace.length+" points");
		//compute arc lengths of both control curves
		float lengthA = Curve.computeArcLength(curveA, 0, 1, resolution*4);
		float lengthB = Curve.computeArcLength(curveB, 0, 1, resolution*4);
		float lengthSum = lengthA + lengthB;
		System.out.println("arc length A="+lengthA+" B="+lengthB+" Sum="+lengthSum);
		//from tracepoint to tracepoint I have to make progress of this value:
		float targetStepSize = lengthSum / resolution;
		System.out.println("target step size: "+targetStepSize);
		//now interpolate traced points to find equispaced points
		output.clear();
		output.add(trace[0]);
		int i=0;
		while (true) {
			TracePoint start = output.get(output.size()-1);
			//find trace point that is behind the target step size
			int lowerIndex = i;
			float lowerDist = 0;
			int upperIndex = -1;
			float upperDist = 0;
			for (; i<trace.length; ++i) {
				TracePoint tp = trace[i];
				float sa = start.projectionOnA[0];
				float ea = tp.projectionOnA[0];
				float sb = start.projectionOnB[0];
				float eb = tp.projectionOnB[0];
				float distA = Curve.computeArcLength(curveA, sa, ea, 16);
				float distB = Curve.computeArcLength(curveB, sb, eb, 16);
				float dist = distA + distB;
				if (dist < targetStepSize) {
					//not reached yet
					lowerIndex = i;
					lowerDist = dist;
				} else {
					//we passed the next trace point
					upperIndex = i;
					upperDist = dist;
					break;
				}
			}
			if (i==trace.length) {
				return; //we have exhausted our original trace, terminate
			}
			//now interpolate between lower and upper trace point linearly
			Vector3f current = new Vector3f();
			current.interpolateLocal(trace[lowerIndex].center, trace[upperIndex].center, (targetStepSize - lowerDist) / (upperDist - lowerDist));
			//perform snap
			SnapResult r = snap(curveA, curveB, current);
			if (r==null) {
				LOG.log(Level.SEVERE, "unable to snap point {0} on the curve", current);
				return;
			}
			//evaluate real performance
			float realDist = 
					Curve.computeArcLength(curveA, start.projectionOnA[0], r.ta, 32)
					+ Curve.computeArcLength(curveB, start.projectionOnB[0], r.tb, 32);
			System.out.println("real step size: "+realDist);
			//add add trace point
			TracePoint tp = new TracePoint(r.center, r.radius, r.ta, r.tb);
			output.add(tp);
			//insert into trace list
			//TODO: check if this works all the time and produces the correct result
			//my hope is that this solves the endless-loop issue happening in some cases.
			trace = ArrayUtils.add(trace, upperIndex, tp);
		}
	}
	
	/**
	 * Traces the medial axis of the two curves {@code curveA} and
	 * {@code curveB}.
	 * <br>
	 * Both curves start and end in the same points ({@code curveA[0]=curveB[0]}
	 * and {@code curveA[curveA.length-1]=curveB[curveB.length-1]}). The curves
	 * are interpolated using {@link Curve#interpolate(curveavg.Vector3f[], float)
	 * }.
	 * <br>
	 * The traced points on the medial axis are represented by instances of the
	 * class {@link TracePoint}. Place the points in the provided output list.
	 *
	 * @param curveA the control points of the first curve.
	 * @param curveB the control points of the second curve
	 * @param output an empty list, place the traced point in here
	 */
	public static void trace(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {
		long time1 = System.currentTimeMillis();
		try {
//			trace1(curveA, curveB, output);
			trace2(curveA, curveB, output);
		} catch (Exception e) {
			LOG.log(Level.SEVERE, "exception while tracing medial axis", e);
		}
		long time2 = System.currentTimeMillis();
		LOG.log(Level.INFO, "tracing medial axis with {0} points within {1} seconds", new Object[]{output.size(), (time2-time1)/1000f});
	}
	
	public static void trace1(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {

		final boolean dbg = false;
		if (dbg) {
			System.out.println("\n");
		}

		// Clear the output
		output.clear();

		// Pick two points close to the intersection on the first segments and find their tangents
		Vector3f tA = Curve.quadraticHermiteTangent(curveA[0], curveA[1], curveA[2].subtract(curveA[0]), 0.01f);
		Vector3f tB = Curve.quadraticHermiteTangent(curveB[0], curveB[1], curveB[2].subtract(curveB[0]), 0.01f);

		// Compute the medial axis of the tangents and move off the intersection.
		// NOTE: The jump needs to be large to avoid weird phenomena.
		MedialAxisTransform.Line line = MedialAxisTransform.medialAxisLine(curveA[0], tA, curveB[0], tB);
		Vector3f Q1 = curveA[0].addScaled(30.0f, line.v);

		// Project out from the current point in the direction of the medial axis of its projected tangents
		// and refine iteratively by moving in the perpendicular direction.
		final float stepSize = 10.0f;
		for (int idx = 0; idx < 55; idx++) {

			// Stop condition
			if (Q1.distance(curveA[curveA.length - 1]) < 1) {
				break;
			}
			if (dbg) {
				System.out.println("Iteration " + idx + "---------------------");
			}
			if (dbg) {
				System.out.println("Line p: " + line.p.toString() + ", line v: " + line.v.toString());
			}

			// Project out the point
			if (dbg) {
				System.out.println("Q1 initial: " + Q1.toString());
			}
			Q1 = Q1.addScaled(stepSize, line.v);
			if (dbg) {
				System.out.println("Q1 projected: " + Q1.toString());
			}

			// Move the estimate in the negative average direction of the projections
			// until the difference in their distances vanishes
			int counter = 0;
			while (true) {

				// Find the projections 
				MedialAxisTransform.ClosestInfo infoA = MedialAxisTransform.findClosest(curveA, Q1);
				MedialAxisTransform.ClosestInfo infoB = MedialAxisTransform.findClosest(curveB, Q1);
				assert (infoA.found);
				assert (infoB.found);

				// Find the distances and their difference
				float distA = Q1.distance(infoA.Pt);
				float distB = Q1.distance(infoB.Pt);
				float err = distA - distB;
				if (dbg) {
					System.out.println("distA: " + distA + ", distB: " + distB + ", |diff|: " + Math.abs(err) + ", dirA: " + infoA.dir.toString() + ", dirB: " + infoB.dir.toString());
				}

				// Stop if the difference is small enough and add to the list
				if (Math.abs(err) < 1e-3) {

					// Generate the trace point
					MedialAxisTransform.TracePoint tp = new MedialAxisTransform.TracePoint();
					tp.center = Q1;
					tp.projectionOnA = new float[1];
					tp.projectionOnA[0] = (infoA.time + infoA.curveIndex) / (curveA.length-1);
					tp.projectionOnB = new float[1];
					tp.projectionOnB[0] = (infoB.time + infoB.curveIndex) / (curveB.length-1);
					tp.radius = (distA + distB) / 2.0f;
					output.add(tp);

					// Stop
					if (dbg) {
						System.out.println("Converged...");
					}
					break;
				}

				// Move the point in the average direction
				Vector3f dir = (infoA.dir.subtract(infoB.dir)).normalize();
				Q1 = Q1.addScaledLocal(-0.2f * err, dir);
				if (counter++ > 50) {
					break;
				}
			}

			// Visualize the medial axis
			if (dbg) {
				System.out.println("Q1 later: " + Q1.toString());
			}

			// Find the closest points to the new medial axis
			MedialAxisTransform.ClosestInfo infoA = MedialAxisTransform.findClosest(curveA, Q1);
			MedialAxisTransform.ClosestInfo infoB = MedialAxisTransform.findClosest(curveB, Q1);

			// End condition: if the projected point on one of the curves is
			// too close to the final intersection
			if (infoA.curveIndex == (curveA.length - 2)) {
				if (infoA.time > 0.4) {
					break;
				}
				System.out.println("iter: " + idx + ", curveIndex: " + infoA.curveIndex + ", time: " + infoA.time);
			}

			// Find the medial axis of the tangent lines 
			line = MedialAxisTransform.medialAxisLine(infoA.Pt, infoA.tangent, infoB.Pt, infoB.tangent);
		}
	}
	
	//Alternative tracing algorithm
	public static boolean trace2(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {
		//add start point
		output.add(new MedialAxisTransform.TracePoint(curveA[0], 0, 0, 0));
		
		Vector3f current = curveA[0].clone();
		float ta = 0;
		float tb = 0;
		//now run the tracing
                int numIters = 0;
		while (true) {
			//compute tangents numerically
			float tangentStepSize = 0.001f;
			Vector3f tA = Curve.interpolate(curveA, Math.min(1, ta+tangentStepSize))
					.subtract(Curve.interpolate(curveA, ta));
			Vector3f tB = Curve.interpolate(curveB, Math.min(1, tb+tangentStepSize))
					.subtract(Curve.interpolate(curveB, tb));
			tA.normalizeLocal();
			tB.normalizeLocal();
			Vector3f t = tA.add(tB).multLocal(STEPSIZE);
			//move current position along this tangent
			current = current.add(t);
			//find closest projections
			SnapResult r = snap(curveA, curveB, current);
			if (r == null) {
                            LOG.severe("snap failed");
				return false;
			}
			float nextTA = r.ta;
			float nextTB = r.tb;
			current = r.center;
//			System.out.println("current="+current+", tA="+nextTA+", tB="+nextTB);
			if (nextTA >= 1 || nextTB >= 1) {
                                LOG.info("reached end");
				return true; //end reached
			}
			if (nextTA < ta || nextTB < tb) {
				//use this as a stop condition
				
				LOG.severe("going backwards!!!");
                                return true;
//				return false;
			}
			//update ta, tb, add trace point
			ta = nextTA;
			tb = nextTB;
			output.add(new MedialAxisTransform.TracePoint(current, r.radius, ta, tb));
                        if(numIters++ > MAX_MA_ITERATION) {
                                LOG.severe("Reached maximum # iters");                            
                                return true;
                        }
		}
	}
	private static class SnapResult {
		Vector3f center;
		float radius;
		float ta, tb;
	}
	//Snaps current on the medial axis
	private static SnapResult snap(Vector3f[] curveA, Vector3f[] curveB, Vector3f current) {
		int maxSteps = 20;
		float stepSize = 0.2f;
		float tangentSize = 0.001f;
		for (int i=0; i<maxSteps; ++i) {
			
			//snap to get the same distances to the control curves
			MedialAxisTransform.ClosestInfo cA =
					MedialAxisTransform.findClosest(curveA, current);
			MedialAxisTransform.ClosestInfo cB =
					MedialAxisTransform.findClosest(curveB, current);
			if (cA==null || cB==null) {
				LOG.log(Level.SEVERE, "unable to compute closest projection of {0}", 
						new Object[]{current});
				return null;
			}
			float nextTA = (cA.curveIndex + cA.time) / (curveA.length-1);
			float nextTB = (cB.curveIndex + cB.time) / (curveB.length-1);
			if (nextTA >= 1 || nextTB >= 1) {
				break;
			}
			Vector3f A = cA.Pt;
			Vector3f B = cB.Pt;
			float distA = A.distance(current);
			float distB = B.distance(current);
			float delta = distA - distB;
//			System.out.println(" snap current="+current+", distA="+distA+", distB="+distB+", delta="+delta);
			if (Math.abs(delta) < 1e-4) {
				break;
			}
			Vector3f dir = (cA.dir.subtract(cB.dir)).normalizeLocal().multLocal(-delta);
			current.addLocal(dir);
			
			//snap into the plane of the line
			Vector3f NA = new Vector3f();
			Vector3f NB = new Vector3f();
			float distPA = distancePointPlane(current, A, B, Curve.interpolate(curveA, nextTA + tangentSize), NA);
			float distPB = distancePointPlane(current, B, A, Curve.interpolate(curveB, nextTB + tangentSize), NB);
			//in the optimal position, distPA = distPB
//			System.out.println("  distPA="+distPA+", distPB="+distPB);
			dir.zero();
			dir.addScaledLocal(-distPA * 0.01f, NA);
			dir.addScaledLocal(-distPB * 0.01f, NB);
			current.addLocal(dir);
		}
		//Build snap result
		MedialAxisTransform.ClosestInfo cA =
					MedialAxisTransform.findClosest(curveA, current);
		MedialAxisTransform.ClosestInfo cB =
				MedialAxisTransform.findClosest(curveB, current);
		if (cA==null || cB==null) {
			LOG.log(Level.SEVERE, "unable to compute closest projection of {0}", 
					new Object[]{current});
			return null;
		}
		float ta = (cA.curveIndex + cA.time) / (curveA.length-1);
		float tb = (cB.curveIndex + cB.time) / (curveB.length-1);
		SnapResult r = new SnapResult();
		r.center = current;
		r.ta = ta;
		r.tb = tb;
		r.radius = (current.distance(cA.Pt) + current.distance(cB.Pt)) / 2f;
		return r;
	}
	private static float distancePointPlane(Vector3f P, Vector3f A, Vector3f B, Vector3f C, Vector3f N) {
		//Compute normal
		N.set((B.subtract(A)).cross(C.subtract(A)));
		N.normalizeLocal();
		//compute plane equation ax+bx+cz+d=0
		float a = N.x;
		float b = N.y;
		float c = N.z;
		float d = -a*A.x - b*A.y - c*A.z;
		//compute signed distance
		float dist = a*P.x + b*P.y + c*P.z + d;
		return dist;
	}
	
	/**
	 * Solve for the intersection parameters s and u for a system of equations
	 * such that x1+s*vx1=x2+u*vx2 and y1+s*vy1=y2+u*vy2.
	 */
	public static Vector3f solveIntersection(float x1, float vx1, float x2, float vx2, float y1, float vy1, float y2, float vy2) {
		float denom = (vx1 * vy2 - vx2 * vy1);
		if (Math.abs(denom) < 1e-5) {
			return new Vector3f(0.0f, 0.0f, -1.0f);
		}
		float s = -(vy2 * x1 - vy2 * x2 - vx2 * y1 + vx2 * y2) / denom;
		float u = -(vy1 * x1 - vy1 * x2 - vx1 * y1 + vx1 * y2) / denom;
		return new Vector3f(s, u, 1);
	}

	/**
	 * Find the medial axis of two lines defined by a point and a unit vector
	 * each.
	 */
	public static Line medialAxisLine(Vector3f p1, Vector3f v1,
			Vector3f p2, Vector3f v2) {

            // Check if the two lines are parallel. If so, return the average
		// of their constant.
		Line maLine = new Line();
		boolean debug = false;
		if ((v1.subtract(v2)).length() < 1e-3) {
			maLine.p = (p1.add(p2)).mult(0.5f);
			maLine.v = v1;
			if (debug) {
				System.out.println("Parallel case.");
			}
			return maLine;
		}

		// Check if the two lines are skew
		Vector3f u = (v1.cross(v2)).normalize();
		float g = (p2.subtract(p1)).dot(u);
		if (Math.abs(g) > 1e-3) {

                // Find the closest point on each line to each other
			// http://2000clicks.com/mathhelp/GeometryPointsAndLines3D.aspx
			Vector3f res = solveIntersection((g * u.x + p1.x), v1.x, p2.x, v2.x,
					(g * u.y + p1.y), v1.y, p2.y, v2.y);
			assert (res.z > 0);
			Vector3f p1c = p1.addScaled(res.x, v1);
			Vector3f p2c = p2.addScaled(res.y, v2);
			maLine.p = (p1c.add(p2c)).mult(0.5f);

			// Compute the direction of the bisecting line
			maLine.v = (v1.add(v2)).normalize();
			if (debug) {
				System.out.println("Skew case.");
				System.out.println("p1= " + p1.toString() + ", v1= " + v1.toString()
						+ ", \np2= " + p2.toString() + ", v2= " + v2.toString());
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

		Vector3f res = new Vector3f(0f, 0f, 0f);
		if (res1.z > 0) {
			res = res1;
		} else if (res2.z > 0) {
			res = res2;
		} else if (res3.z > 0) {
			res = res3;
		} else {
			assert (false);
		}

		// Perform the operation for intersecting lines
		maLine.p = new Vector3f(p1.addScaled(res.x, v1));
		maLine.v = new Vector3f((v1.add(v2)).normalize());

		if (debug) {
			System.out.println("pm: " + maLine.p.toString());
			System.out.println("Intersecting case.");
		}
		return maLine;
	}

	/**
	 * Perform Newton's algorithm to find the zero of the derivative wrt time of
	 * the perpendicularity constraint between a point and a cubic hermite
	 * function. The equation is a quintic, thus no closed form. The function is
	 * f(P(t), Q) = dot((Q-P(t)), P'(t));
	 */
	public static float perpPointOnCubicHermite(Vector3f P0, Vector3f T0, Vector3f P1, Vector3f T1, Vector3f Q) {

		// Get the parameters of the quintic function dfdt
//            float f = - 2*T0.x*(Q.x - P0.x) - 2*T0.y*(Q.y - P0.y) - 2*T0.z*(Q.z - P0.z);
//            float e = 2*(Q.x - P0.x)*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) + 2*(Q.y - P0.y)*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) + 2*(Q.z - P0.z)*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) + 2*T0.x*T0.x + 2*T0.y*T0.y + 2*T0.z*T0.z;
//            float d = - 2*T0.x*(2*T0.x + T1.x + 3*P0.x - 3*P1.x) - 2*T0.y*(2*T0.y + T1.y + 3*P0.y - 3*P1.y) - 2*T0.z*(2*T0.z + T1.z + 3*P0.z - 3*P1.z) - 2*T0.x*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) - 2*T0.y*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) - 2*T0.z*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) - 2*(Q.x - P0.x)*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) - 2*(Q.y - P0.y)*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) - 2*(Q.z - P0.z)*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z);
//            float c = 2*(2*T0.x + T1.x + 3*P0.x - 3*P1.x)*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x) + 2*(2*T0.y + T1.y + 3*P0.y - 3*P1.y)*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y) + 2*(2*T0.z + T1.z + 3*P0.z - 3*P1.z)*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z) + 2*T0.x*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) + 2*T0.y*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) + 2*T0.z*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z) + 2*T0.x*(T0.x + T1.x + 2*P0.x - 2*P1.x) + 2*T0.y*(T0.y + T1.y + 2*P0.y - 2*P1.y) + 2*T0.z*(T0.z + T1.z + 2*P0.z - 2*P1.z);
//            float b = - 2*(4*T0.x + 2*T1.x + 6*P0.x - 6*P1.x)*(T0.x + T1.x + 2*P0.x - 2*P1.x) - 2*(4*T0.y + 2*T1.y + 6*P0.y - 6*P1.y)*(T0.y + T1.y + 2*P0.y - 2*P1.y) - 2*(4*T0.z + 2*T1.z + 6*P0.z - 6*P1.z)*(T0.z + T1.z + 2*P0.z - 2*P1.z) - 2*(2*T0.x + T1.x + 3*P0.x - 3*P1.x)*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x) - 2*(2*T0.y + T1.y + 3*P0.y - 3*P1.y)*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y) - 2*(2*T0.z + T1.z + 3*P0.z - 3*P1.z)*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z);
//            float a = 2*(3*T0.x + 3*T1.x + 6*P0.x - 6*P1.x)*(T0.x + T1.x + 2*P0.x - 2*P1.x) + 2*(3*T0.y + 3*T1.y + 6*P0.y - 6*P1.y)*(T0.y + T1.y + 2*P0.y - 2*P1.y) + 2*(3*T0.z + 3*T1.z + 6*P0.z - 6*P1.z)*(T0.z + T1.z + 2*P0.z - 2*P1.z);
		float f = T0.x * (Q.x - P0.x) + T0.y * (Q.y - P0.y) + T0.z * (Q.z - P0.z);
		float e = -(Q.x - P0.x) * (4 * T0.x + 2 * T1.x + 6 * P0.x - 6 * P1.x) - (Q.y - P0.y) * (4 * T0.y + 2 * T1.y + 6 * P0.y - 6 * P1.y) - (Q.z - P0.z) * (4 * T0.z + 2 * T1.z + 6 * P0.z - 6 * P1.z) - T0.x * T0.x - T0.y * T0.y - T0.z * T0.z;
		float d = T0.x * (2 * T0.x + T1.x + 3 * P0.x - 3 * P1.x) + T0.y * (2 * T0.y + T1.y + 3 * P0.y - 3 * P1.y) + T0.z * (2 * T0.z + T1.z + 3 * P0.z - 3 * P1.z) + T0.x * (4 * T0.x + 2 * T1.x + 6 * P0.x - 6 * P1.x) + T0.y * (4 * T0.y + 2 * T1.y + 6 * P0.y - 6 * P1.y) + T0.z * (4 * T0.z + 2 * T1.z + 6 * P0.z - 6 * P1.z) + (Q.x - P0.x) * (3 * T0.x + 3 * T1.x + 6 * P0.x - 6 * P1.x) + (Q.y - P0.y) * (3 * T0.y + 3 * T1.y + 6 * P0.y - 6 * P1.y) + (Q.z - P0.z) * (3 * T0.z + 3 * T1.z + 6 * P0.z - 6 * P1.z);
		float c = -(2 * T0.x + T1.x + 3 * P0.x - 3 * P1.x) * (4 * T0.x + 2 * T1.x + 6 * P0.x - 6 * P1.x) - (2 * T0.y + T1.y + 3 * P0.y - 3 * P1.y) * (4 * T0.y + 2 * T1.y + 6 * P0.y - 6 * P1.y) - (2 * T0.z + T1.z + 3 * P0.z - 3 * P1.z) * (4 * T0.z + 2 * T1.z + 6 * P0.z - 6 * P1.z) - T0.x * (3 * T0.x + 3 * T1.x + 6 * P0.x - 6 * P1.x) - T0.y * (3 * T0.y + 3 * T1.y + 6 * P0.y - 6 * P1.y) - T0.z * (3 * T0.z + 3 * T1.z + 6 * P0.z - 6 * P1.z) - T0.x * (T0.x + T1.x + 2 * P0.x - 2 * P1.x) - T0.y * (T0.y + T1.y + 2 * P0.y - 2 * P1.y) - T0.z * (T0.z + T1.z + 2 * P0.z - 2 * P1.z);
		float b = (4 * T0.x + 2 * T1.x + 6 * P0.x - 6 * P1.x) * (T0.x + T1.x + 2 * P0.x - 2 * P1.x) + (4 * T0.y + 2 * T1.y + 6 * P0.y - 6 * P1.y) * (T0.y + T1.y + 2 * P0.y - 2 * P1.y) + (4 * T0.z + 2 * T1.z + 6 * P0.z - 6 * P1.z) * (T0.z + T1.z + 2 * P0.z - 2 * P1.z) + (2 * T0.x + T1.x + 3 * P0.x - 3 * P1.x) * (3 * T0.x + 3 * T1.x + 6 * P0.x - 6 * P1.x) + (2 * T0.y + T1.y + 3 * P0.y - 3 * P1.y) * (3 * T0.y + 3 * T1.y + 6 * P0.y - 6 * P1.y) + (2 * T0.z + T1.z + 3 * P0.z - 3 * P1.z) * (3 * T0.z + 3 * T1.z + 6 * P0.z - 6 * P1.z);
		float a = -(3 * T0.x + 3 * T1.x + 6 * P0.x - 6 * P1.x) * (T0.x + T1.x + 2 * P0.x - 2 * P1.x) - (3 * T0.y + 3 * T1.y + 6 * P0.y - 6 * P1.y) * (T0.y + T1.y + 2 * P0.y - 2 * P1.y) - (3 * T0.z + 3 * T1.z + 6 * P0.z - 6 * P1.z) * (T0.z + T1.z + 2 * P0.z - 2 * P1.z);

		// Get the real roots of the quintic
		SolverResult res = solveQuinticLaguerre(a / f, b / f, c / f, d / f, e / f, 1.0f);

		// Check which endpoint is closer
//		float dist0 = Q.distance(P0);
//		float dist1 = Q.distance(P1);
//		float minEndpointDist = Math.min(dist0, dist1);
//            System.out.println("Minimum endpoint dist: " + minEndpointDist);

            // For each nonimaginary root, with real part within [0,1], compute 
		// the distance to the input point and find the smallest value
		float bestDistSQ = 1e6f;
		float time = -1f;
		for (int i = 0; i < 6; i++) {
			if (Math.abs(res.im[i]) > 1e-3) {
				continue;
			}
			if (res.re[i] > 1 || res.re[i] < 0) {
				continue;
			}
			Vector3f Pt = Curve.cubicHermite(P0, T0, P1, T1, res.re[i]);
			float distSQ = Q.distanceSquared(Pt);
//                System.out.println("\troot t: " + res.re[i] + ", pt: " + Pt.toString() + ", dist:" + dist);
			if (distSQ < bestDistSQ) {
				time = res.re[i];
				bestDistSQ = distSQ;
			}
		}

		return time;
	}

	/**
	 * Uses the Laguerre algorithm iteratively to span the range [0,1] and find
	 * all the real roots.
	 */
	public static SolverResult solveQuinticLaguerre(float a, float b, float c, float d, float e, float f) {

		// Initialize the result
//            System.out.println("quintic params: " + a + ", "+ b + ", "+ c + ", "+ d + ", "+ e + ", "+ f);
		SolverResult res = new SolverResult();
		res.re = new float[6];
		res.im = new float[6];
		for (int i = 0; i < 6; i++) {
			res.im[i] = 1.0f;
		}

		// Create the quintic function and the solver
		double coefficients[] = {f, e, d, c, b, a};
		PolynomialFunction qf = new PolynomialFunction(coefficients);
		LaguerreSolver solver = new LaguerreSolver();

            // Iteratively call the NewtonRaphson solution until no solution 
		// exists
		double minBound = 0.0;
		double sol;
		int root_idx = 0;
		final double dt = 0.01;
		for (double t = -dt; t < 1.0; t += dt) {

			// Check if there is a sign-crossing between the two times
			double t2 = t + dt;
			double f1 = qf.value(t);
			double f2 = qf.value(t2);
			if (Math.signum(f1) == Math.signum(f2)) {
				continue;
			}

			// Attempt to get the solution
			try {
				sol = solver.solve(100, qf, t, t2);
			} catch (TooManyEvaluationsException | NumberIsTooLargeException exc) {
				break;
			}

			// Save the root and update the minimum bound
			res.re[root_idx] = (float) sol;
			res.im[root_idx] = 0.0f;
			minBound = sol + 1e-3;
			root_idx++;
		}

//            System.out.println("#quintic roots: " + root_idx);
		return res;
	}

	/**
	 * Uses the NewtonRaphson algorithm iteratively to span the range [0,1] and
	 * find all the real roots.
	 */
	public static SolverResult solveQuinticNewtonRaphson(float a, float b, float c, float d, float e, float f) {

		// Initialize the result
		System.out.println("quintic params: " + a + ", " + b + ", " + c + ", " + d + ", " + e + ", " + f);
		SolverResult res = new SolverResult();
		res.re = new float[6];
		res.im = new float[6];
		for (int i = 0; i < 6; i++) {
			res.im[i] = 1.0f;
		}

		// Create the quintic function and the solver
		QuinticFunction qf = new QuinticFunction(a, b, c, d, e, f);
		NewtonRaphsonSolver solver = new NewtonRaphsonSolver();

            // Iteratively call the NewtonRaphson solution until no solution 
		// exists
		double minBound = 0.0;
		double sol;
		int root_idx = 0;
		while (true) {

			// Attempt to get the solution
			try {
				sol = solver.solve(100, qf, minBound, 1.0);
			} catch (TooManyEvaluationsException | NumberIsTooLargeException exc) {
				break;
			}

                // Check if the solution is within the bounds (shouldn't be 
			// necessary but it is...)
			if (sol < minBound || sol > 1) {
				break;
			}

			// Save the root and update the minimum bound
			res.re[root_idx] = (float) sol;
			res.im[root_idx] = 0.0f;
			minBound = sol + 1e-3;
			root_idx++;
		}

		System.out.println("#quintic roots: " + root_idx);
		return res;
	}

	/**
	 * Solves a cubic equation using a closed-form approach.
	 * http://stackoverflow.com/questions/13328676/c-solving-cubic-equations
	 */
	public static SolverResult solveCubic(float a, float b, float c, float d) {

            
		SolverResult res = new SolverResult();
		res.re = new float[3];
		res.im = new float[3];
                
                // The problem is in the form of ax^3 + bx^2 + cx = 0
		// Solve quadratic
		if (Math.abs(d) < 1e-3) {
                    
                    // One of the roots is 0.
                    res.re[0] = 0.0f; 
                    res.im[0] = 0.0f;
                    
                    // Solve the quadratic equation
                    res.re[1] = res.re[2] = (-b / (2.0f * a));
                    float discr = b*b - 4.0f*a*c;
                    if(discr < 0) {
                        float temp = (float) (Math.sqrt(-discr) / (2.0f * a));
                        res.im[1] = temp;
                        res.im[2] = -temp;
                    }
                    else {
                        float temp = (float) (Math.sqrt(discr) / (2.0f * a));
                        res.im[1] = res.im[2] = 0.0f;
                        res.re[1] += temp;
                        res.re[2] -= temp;
                    }
                    System.out.println("a: " + a + ", b: " + b + ", c: " + c + ", d: " + d);
                    System.out.println("results: (" + res.re[0] + ", " + res.im[0] + "i), (" + res.re[1] + ", " + res.im[1] + "i), (" + res.re[2] + ", " + res.im[2] + "i)");
                    LOG.severe("Given system is not a general cubic - may cause problems");                    
                    return res;
                    // assert (false);
		}

		b /= a;
		c /= a;
		d /= a;
		float disc, q, r, dum1, s, t, term1, r13;
		q = (3.0f * c - (b * b)) / 9.0f;
		r = -(27.0f * d) + b * (9.0f * c - 2.0f * (b * b));
		r /= 54.0f;
		disc = q * q * q + r * r;
		term1 = (b / 3.0f);
		res.im[0] = 0.0f;
		if (disc > 0) { // one root real, two are complex
			s = r + (float) Math.sqrt(disc);
			s = (float) ((s < 0) ? -Math.pow(-s, (1.0 / 3.0)) : Math.pow(s, (1.0 / 3.0)));
			t = r - (float) Math.sqrt(disc);
			t = (float) ((t < 0) ? -Math.pow(-t, (1.0 / 3.0)) : Math.pow(t, (1.0 / 3.0)));
			res.re[0] = -term1 + s + t;
			term1 += (s + t) / 2.0;
			res.re[2] = res.re[1] = -term1;
			term1 = (float) Math.sqrt(3.0) * (-t + s) / 2;
			res.im[1] = term1;
			res.im[2] = -term1;
			return res;
		}

		// The remaining options are all real
		res.im[1] = res.im[2] = 0;
		if (disc == 0) { // All roots real, at least two are equal.
			r13 = (float) ((r < 0) ? -Math.pow(-r, (1.0 / 3.0)) : Math.pow(r, (1.0 / 3.0)));
			res.re[0] = -term1 + 2.0f * r13;
			res.re[0] = res.re[1] = -(r13 + term1);
			return res;
		}

		// Only option left is that all roots are real and unequal (to get here, q < 0)
		q = -q;
		dum1 = q * q * q;
		dum1 = (float) Math.acos(r / Math.sqrt(dum1));
		r13 = (float) (2.0f * Math.sqrt(q));
		res.re[0] = -term1 + (float) (r13 * Math.cos(dum1 / 3.0));
		res.re[1] = -term1 + (float) (r13 * Math.cos((dum1 + 2.0 * Math.PI) / 3.0));
		res.re[2] = -term1 + (float) (r13 * Math.cos((dum1 + 4.0 * Math.PI) / 3.0));
		return res;
	}

	/**
	 * Find the zero of the derivative wrt time of the perpendicularity
	 * constraint between a point and a quadratic hermite function in cubic
	 * closed-form. * The function is f(P(t), Q) = dot((Q-P(t)), P'(t));
	 */
	public static float perpPointOnQuadraticHermite(Vector3f P0, Vector3f T0, Vector3f P1, Vector3f Q) {

//            System.out.println("perpPoint call: P0: " + P0.toString() +",T0: " + T0.toString() +", P1: " + P1.toString() +", Q: " + Q.toString());
		// Get the parameters of the cubic function dfdt
//            float d = 2*(Q.x - P0.x)*(T0.x + 2*P0.x - 2*P1.x) + 2*(Q.y - P0.y)*(T0.y + 2*P0.y - 2*P1.y) + 2*(Q.z - P0.z)*(T0.z + 2*P0.z - 2*P1.z);
//            float c = (float) (2*Math.pow(T0.x + 2*P0.x - 2*P1.x,2) + 2*Math.pow(T0.y + 2*P0.y - 2*P1.y,2) + 2*Math.pow(T0.z + 2*P0.z - 2*P1.z,2) - 2*(Q.x - P0.x)*(2*T0.x + 2*P0.x - 2*P1.x) - 2*(Q.y - P0.y)*(2*T0.y + 2*P0.y - 2*P1.y) - 2*(Q.z - P0.z)*(2*T0.z + 2*P0.z - 2*P1.z));
//            float b = (float) (- 2*(T0.x + P0.x - P1.x)*(T0.x + 2*P0.x - 2*P1.x) - 2*(T0.y + P0.y - P1.y)*(T0.y + 2*P0.y - 2*P1.y) - 2*(T0.z + P0.z - P1.z)*(T0.z + 2*P0.z - 2*P1.z) - 2*(T0.x + 2*P0.x - 2*P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) - 2*(T0.y + 2*P0.y - 2*P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) - 2*(T0.z + 2*P0.z - 2*P1.z)*(2*T0.z + 2*P0.z - 2*P1.z));
//            float a = 2*(T0.x + P0.x - P1.x)*(2*T0.x + 2*P0.x - 2*P1.x) + 2*(T0.y + P0.y - P1.y)*(2*T0.y + 2*P0.y - 2*P1.y) + 2*(T0.z + P0.z - P1.z)*(2*T0.z + 2*P0.z - 2*P1.z);
		float d = -(Q.x - P0.x) * (T0.x + 2 * P0.x - 2 * P1.x) - (Q.y - P0.y) * (T0.y + 2 * P0.y - 2 * P1.y) - (Q.z - P0.z) * (T0.z + 2 * P0.z - 2 * P1.z);
		float c = (float) ((Q.x - P0.x) * (2 * T0.x + 2 * P0.x - 2 * P1.x) - Math.pow(T0.y + 2 * P0.y - 2 * P1.y, 2) - Math.pow(T0.z + 2 * P0.z - 2 * P1.z, 2) - Math.pow(T0.x + 2 * P0.x - 2 * P1.x, 2) + (Q.y - P0.y) * (2 * T0.y + 2 * P0.y - 2 * P1.y) + (Q.z - P0.z) * (2 * T0.z + 2 * P0.z - 2 * P1.z));
		float b = (float) ((T0.x + P0.x - P1.x) * (T0.x + 2 * P0.x - 2 * P1.x) + (T0.y + P0.y - P1.y) * (T0.y + 2 * P0.y - 2 * P1.y) + (T0.z + P0.z - P1.z) * (T0.z + 2 * P0.z - 2 * P1.z) + (T0.x + 2 * P0.x - 2 * P1.x) * (2 * T0.x + 2 * P0.x - 2 * P1.x) + (T0.y + 2 * P0.y - 2 * P1.y) * (2 * T0.y + 2 * P0.y - 2 * P1.y) + (T0.z + 2 * P0.z - 2 * P1.z) * (2 * T0.z + 2 * P0.z - 2 * P1.z));
		float a = -(T0.x + P0.x - P1.x) * (2 * T0.x + 2 * P0.x - 2 * P1.x) - (T0.y + P0.y - P1.y) * (2 * T0.y + 2 * P0.y - 2 * P1.y) - (T0.z + P0.z - P1.z) * (2 * T0.z + 2 * P0.z - 2 * P1.z);

		// Compute the roots of the cubic
		SolverResult res = solveCubic(a, b, c, d);

		// Check which endpoint is closer
		float dist0 = Q.distance(P0);
		float dist1 = Q.distance(P1);
		float minEndpointDist = Math.min(dist0, dist1);
            // System.out.println("Minimum endpoint dist: " + minEndpointDist);

            // For each nonimaginary root, with real part within [0,1], compute 
		// the distance to the input point and find the smallest value
		float bestDist = 1e6f;
		float time = -1f;
		for (int i = 0; i < 3; i++) {
//                System.out.println("Root " + i + ": " + res.re[i] + ", " + res.im[i]);
			if (Math.abs(res.im[i]) > 1e-3) {
				continue;
			}
			if (res.re[i] > 1 || res.re[i] < 0) {
				continue;
			}
			Vector3f Pt = Curve.quadraticHermite(P0, T0, P1, res.re[i]);
			float dist = Q.distance(Pt);
//                System.out.println("\troot time: " + res.re[i] + ", pt: " + Pt.toString() + ", dist:" + dist);
			if (dist <= (bestDist + 1e-3)) {
				time = res.re[i];
				bestDist = dist;
			}
		}

//            System.out.println("Final time: " + time);
		return time;
	}

	/**
	 * Given a point Q, finds the closest point on the given curve. Returns the
	 * point and the tangent to the curve at that point.
	 *
	 * @param curve
	 * @param Q
	 */
	public static ClosestInfo findClosest(Vector3f[] curve, Vector3f Q) {

//            System.out.println("\nfindClosest for " + Q.toString() + " -----------------");
		// Go through the curve segment by segment and treat quadratic and
		// cubic segments differently. If the point is found to belong
		// to one of the segments stop.
		int n = curve.length;
		float minDist = 1e6f;
		ClosestInfo info = new ClosestInfo();
		for (int i = 0; i < n - 1; i++) {

			// First segment
			if (i == 0) {
				Vector3f P0 = curve[0];
				Vector3f P1 = curve[1];
				Vector3f T0 = curve[2].subtract(curve[0]).multLocal(Curve.TANGENT_SCALE);
				float time = perpPointOnQuadraticHermite(P0, T0, P1, Q);
                    // System.out.println("Time for first segment: " + time + ", for Q: " + Q.toString());
				// System.out.println("\tP0: " + P0.toString() +", P1: " + P1.toString());
				if (time > -1e-3) {
//                        System.out.println("Returning the first segment.");
					Vector3f Pt = Curve.quadraticHermite(P0, T0, P1, time);
					float dist = Pt.distance(Q);
					if (dist < minDist) {
						info = new ClosestInfo(Pt, Curve.quadraticHermiteTangent(P0, T0, P1, time),
								Q.subtract(Pt).normalize(), time, i);
						minDist = dist;
					}
				}
			} // Last segment (had to switch p0 and p1 around)
			else if (i == n - 2) {
				Vector3f P0 = curve[n - 1];
				Vector3f P1 = curve[n - 2];
				Vector3f T0 = curve[n - 3].subtract(curve[n - 1]).multLocal(Curve.TANGENT_SCALE);
				float time = perpPointOnQuadraticHermite(P0, T0, P1, Q);
				if (time > -1e-3) {
					Vector3f Pt = Curve.quadraticHermite(P0, T0, P1, time);
					float dist = Pt.distance(Q);
					if (dist < minDist) {
						info = new ClosestInfo(Pt, Curve.quadraticHermiteTangent(P0, T0, P1, time),
								Q.subtract(Pt).normalize(), 1 - time, i);
						minDist = dist;
					}
				}
			} // Middle segment
			else {
//                    if(true) continue;
				Vector3f P0 = curve[i];
				Vector3f P1 = curve[i + 1];
				Vector3f T0 = curve[i + 1].subtract(curve[i - 1]).multLocal(Curve.TANGENT_SCALE);
				Vector3f T1 = curve[i + 2].subtract(curve[i]).multLocal(Curve.TANGENT_SCALE);
				float time = perpPointOnCubicHermite(P0, T0, P1, T1, Q);
				if (time > -1e-3) {
					Vector3f Pt = Curve.cubicHermite(P0, T0, P1, T1, time);
					float dist = Pt.distance(Q);
					if (dist < minDist) {
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
