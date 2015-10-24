/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import processing.core.*;
import processing.event.*;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static curveavg.Pv3D.*;

/**
 *
 * @author Sebastian Weiss
 */
public class CurveAverage extends AbstractPApplet {
	private static final Logger LOG = Logger.getLogger(CurveAverage.class.getName());

	public static final float SCALE = 50;
	private static final float PICK_TOLERANCE = 20;
	public static final float DEG_TO_RAD = PI / 180.0f;
	private static final float FPS = 60;
	private static final int CURVE_INTERPOLATION_SAMPLES = 32;
	private static final int CURVE_CYLINDER_SAMPLES = 8;
	private static final float CURVE_CYLINDER_RADIUS = 0.15f * SCALE;
	private static final int MEDIAL_AXIS_NET_COUNT = 10;

	private float dz = 0; // distance to camera. Manipulated with wheel or when 
//float rx=-0.06*TWO_PI, ry=-0.04*TWO_PI;    // view angles manipulated when space pressed but not mouse
	private float rx = 0, ry = 0;    // view angles manipulated when space pressed but not mouse
	private boolean animating = false, tracking = false, center = true, gouraud = true;
	private boolean interpolateControlCurve = true;
	private boolean equispacedInterpolation = true;
	private boolean viewpoint = false;
	private PImage myFace;
	private boolean filming = false, takePicture = false;
	private boolean change = false;
	private int frameCounter = 0;
	private int pictureCounter = 0;

	private long time1, time2;
	private float tpf;

	private int pickedPoint = 0;
	private Vector3f[] controlPoints;
	private Vector3f[] curveA, curveB;
	public Vector3f[] debugPoints;             ///< Some useful points that we can drag around 
	boolean recalculateCurve = false;
	private Vector3f[] samplesA, samplesB;
	List<MedialAxisTransform.TracePoint> ma = new ArrayList<MedialAxisTransform.TracePoint>();

	String title = "6491 P3 2015: Curve Average", name = "Sebastian Weiß, Can Erdogan",
			menu = "!:picture, ~:(start/stop)capture, space:rotate, "
			+ "s/wheel:closer, a:anim, i:interpolate control curve, e:equispaced interpolation, #:quit",
			guide = "click'n'drag the control points of the two curves green and blue"; // user's guide

	public static void main(String[] args) {
		PApplet.main(CurveAverage.class.getName());
	}

	public void settings() {
		size(900, 900, P3D); // p3D means that we will do 3D graphics
		noSmooth();
	}

	public void setup() {
		frameRate(FPS);

		myFace = loadImage("data/pic.jpg");  // load image from file pic.jpg in folder data *** replace that file with your pic of your own face
		textureMode(NORMAL);

		//example curve
		controlPoints = new Vector3f[]{
			new Vector3f(-50, -30, -70), //start
			new Vector3f(-60, 10, -40),
			new Vector3f(-70, 12, -6),
			new Vector3f(0, 50, 60),
			new Vector3f(80, 45, 140),
			new Vector3f(150, 80, 200), //end
			new Vector3f(140, 90, 160),
			new Vector3f(100, 37, 100),
			new Vector3f(50, 39, 40),
			//                        new Vector3f(17.307629f, 0.0f, -47.95851f)
			new Vector3f(0, 15, -20)
		};
//                for(int i = 0; i < controlPoints.length; i++) controlPoints[i].y = 0.0f;

		curveA = new Vector3f[controlPoints.length / 2 + 1];
		curveB = new Vector3f[controlPoints.length / 2 + 1];
		for (int i = 0; i < curveA.length; ++i) {
			curveA[i] = controlPoints[i];
		}
		curveB[0] = controlPoints[0];
		for (int i = 1; i < curveB.length; ++i) {
			curveB[i] = controlPoints[controlPoints.length - i];
		}
		recalculateCurve = true;

		rx += 0.0001f;
		ry += 0.0001f;

		time1 = System.currentTimeMillis();
		time2 = System.currentTimeMillis();

		// Add a debug point
		debugPoints = new Vector3f[]{
			//new Vector3f(34,35,56)};
			new Vector3f(-47.615837f, 0.0f, -38.66822f)};
		/*
		 new Vector3f(150, 90, 200),
		 new Vector3f(-70, -30, -70)
		 };
		 */
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
	public void trace(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {
		trace1(curveA, curveB, output);
//		trace2(curveA, curveB, output);
	}
	
	public void trace1(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {

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
		Vector3f Q1 = curveA[0].addScale(30.0f, line.v);

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
			Q1 = Q1.addScale(stepSize, line.v);
			if (dbg) {
				System.out.println("Q1 projected: " + Q1.toString());
			}

			// Debugging information
			if (dbg) {
				fill(red);
				showCylinder(Q1.addScale(-3 * SCALE, line.v), Q1.addScale(3 * SCALE, line.v), 2, 8);
				drawPoint(Q1, red);
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
				Q1 = Q1.addScaleLocal(-0.2f * err, dir);
				if (counter++ > 50) {
					break;
				}
			}

			// Visualize the medial axis
			if (dbg) {
				System.out.println("Q1 later: " + Q1.toString());
			}
			if (dbg) {
				drawPoint(Q1, orange);
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

			// Visualize stuff
			if (dbg) {
				drawPoint(infoA.Pt, blue);
				showCylinder(infoA.Pt.addScale(-1 * SCALE, infoA.dir), infoA.Pt.addScale(1 * SCALE, infoA.dir), 2, 8);
				drawPoint(infoB.Pt, green);
				showCylinder(infoB.Pt.addScale(-1 * SCALE, infoB.dir), infoB.Pt.addScale(1 * SCALE, infoB.dir), 2, 8);
			}

			// Find the medial axis of the tangent lines 
			line = MedialAxisTransform.medialAxisLine(infoA.Pt, infoA.tangent, infoB.Pt, infoB.tangent);
		}
	}
	
	public boolean trace2(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {
		//add start point
		output.add(new MedialAxisTransform.TracePoint(curveA[0], 0, 0, 0));
		
		Vector3f current = curveA[0].clone();
		float ta = 0;
		float tb = 0;
		//now run the tracing
		float stepSize = 1f;
		while (true) {
			//compute tangents numerically
			float tangentStepSize = 0.001f;
			Vector3f tA = Curve.interpolate(curveA, Math.min(1, ta+tangentStepSize))
					.subtract(Curve.interpolate(curveA, ta));
			Vector3f tB = Curve.interpolate(curveB, Math.min(1, tb+tangentStepSize))
					.subtract(Curve.interpolate(curveB, tb));
			tA.normalizeLocal();
			tB.normalizeLocal();
			Vector3f t = tA.add(tB).multLocal(stepSize);
			//move current position along this tangent
			current = current.add(t);
			//find closest projections
			//TODO: replace this by snap() and update current
//			MedialAxisTransform.ClosestInfo cA =
//					MedialAxisTransform.findClosest(curveA, current);
//			MedialAxisTransform.ClosestInfo cB =
//					MedialAxisTransform.findClosest(curveB, current);
//			if (cA==null || cB==null) {
//				LOG.log(Level.SEVERE, "unable to compute closest projection of {0} at time t={1}:{2}", 
//						new Object[]{current, ta, tb});
//				return false;
//			}
//			float nextTA = (cA.curveIndex + cA.time) / (curveA.length-1);
//			float nextTB = (cB.curveIndex + cB.time) / (curveB.length-1);
			SnapResult r = snap(curveA, curveB, current);
			if (r == null) {
				return false;
			}
			float nextTA = r.ta;
			float nextTB = r.tb;
			current = r.center;
			System.out.println("current="+current+", tA="+nextTA+", tB="+nextTB);
			if (nextTA >= 1 || nextTB >= 1) {
				return true; //end reached
			}
			if (nextTA < ta || nextTB < tb) {
				LOG.severe("going backwards!!!");
				return false;
			}
			//update ta, tb, add trace point
			ta = nextTA;
			tb = nextTB;
			output.add(new MedialAxisTransform.TracePoint(current, 0, ta, tb));
		}
	}
	private static class SnapResult {
		Vector3f center;
		float radius;
		float ta, tb;
	}
	//Snaps current on the medial axis
	private SnapResult snap(Vector3f[] curveA, Vector3f[] curveB, Vector3f current) {
		int maxSteps = 20;
		float stepSize = 0.2f;
		for (int i=0; i<maxSteps; ++i) {
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
			Vector3f A = cA.Pt;
			Vector3f B = cB.Pt;
			float distA = A.distance(current);
			float distB = B.distance(current);
			float delta = distA - distB;
			System.out.println(" snap current="+current+", distA="+distA+", distB="+distB+", delta="+delta);
			if (Math.abs(delta) < 1e-4) {
				break;
			}
			Vector3f dir = (cA.dir.subtract(cB.dir)).normalizeLocal().multLocal(-delta);
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

	public void drawPoint(Vector3f P, int color) {
		fill(color);
		pushMatrix();
		translate(P.x, P.y, P.z);
		sphere(0.1f * SCALE);
		popMatrix();
	}

	public void draw() {
		time1 = System.currentTimeMillis();
		tpf = (time1 - time2) / 1000f;
		time2 = time1;

		background(255);
		pushMatrix();   // to ensure that we can restore the standard view before writing on the canvas

		float fov = PI / 3.0f;
		float cameraZ = (height / 2.0f) / tan(fov / 2.0f);
		camera(0, 0, cameraZ, 0, 0, 0, 0, 1, 0);       // sets a standard perspective
		perspective(fov, 1.0f, 0.1f, 10000);

		translate(0, 0, dz); // puts origin of model at screen center and moves forward/away by dz
		lights();  // turns on view-dependent lighting
		rotateX(rx);
		rotateY(ry); // rotates the model around the new origin (center of screen)
		rotateX(PI / 2); // rotates frame around X to make X and Y basis vectors parallel to the floor
		noStroke(); // if you use stroke, the weight (width) of it will be scaled with you scaleing factor

		/// TODO: The visualization of the arrows do not respect cross product rules. 
		showFrame(-PI / 2, PI / 2, 0, 50); // X-red, Y-green, Z-blue arrows

		computeProjectedVectors(); // computes screen projections I, J, K of basis vectors (see bottom of pv3D): used for dragging in viewer's frame    

		calculateAndShowCurve();

		// Draw the debug points
		noStroke();

		//Show debug  points
		fill(red);
		for (Vector3f p : debugPoints) {
			pushMatrix();
			translate(p.x, p.y, p.z);
//                    sphere(0.1f*SCALE);
			popMatrix();
		}
		debugPoints[0].y = 0.0f;

		popMatrix(); // done with 3D drawing. Restore front view for writing text on canvas

		// for demos: shows the mouse and the key pressed (but it may be hidden by the 3D model)
		//  if(keyPressed) {stroke(red); fill(white); ellipse(mouseX,mouseY,26,26); fill(red); text(key,mouseX-5,mouseY+4);}
		fill(0xff000000);
		displayHeader(); // dispalys header on canvas, including my face
		if (!filming) {
			displayFooter(); // shows menu at bottom, only if not filming
		}
		if (filming && (animating || change)) {
			saveFrame("FRAMES/F" + nf(frameCounter++, 4) + ".tif");  // save next frame to make a movie
		}
		if (takePicture) {
			takePicture = false;
			saveFrame("FRAMES/P" + nf(pictureCounter++, 4) + ".tif");
		}
		change = false; // to avoid capturing frames when nothing happens (change is set uppn action)
	}

	protected void showClosestPointsAndTangents() {
		// Handle the first curve
		//            System.out.println("Input point: " + debugPoints[0].toString());
		/*
		 MedialAxisTransform.ClosestInfo info1 = MedialAxisTransform.findClosest(curveA, debugPoints[0]);
		 //            System.out.println("First curve: index: " + info1.curveIndex + ", time: " + info1.time + ", point: " + info1.Pt.toString());
		 if(info1.curveIndex > -1 && info1.curveIndex < curveA.length-1 && info1.time >= 0 && info1.time <= 1) {
		 // Visualize the closest point
		 pushMatrix();
		 translate(info1.Pt.x, info1.Pt.y, info1.Pt.z);
		 sphere(0.1f*SCALE);
		 popMatrix();
		 // Visualize the tangent
		 Vector3f ux = (debugPoints[0].subtract(info1.Pt)).normalize();
		 Vector3f uy = info1.tangent;
		 Vector3f px = info1.Pt.addScale(0.4f * SCALE, ux);
		 Vector3f py = info1.Pt.addScale(0.7f * SCALE, uy);
		 fill(blue);
		 showCylinder(info1.Pt, px, CURVE_CYLINDER_RADIUS/1.5f, CURVE_CYLINDER_SAMPLES);
		 showCylinder(info1.Pt, py, CURVE_CYLINDER_RADIUS/2f, CURVE_CYLINDER_SAMPLES);
		 //                System.out.println("px: " + px.toString() + ", py: " + py.toString());
		 }
		 */
		// Handle the second curve
		MedialAxisTransform.ClosestInfo info2 = MedialAxisTransform.findClosest(curveB, debugPoints[0]);
		//            System.out.println("First curve: index: " + info2.curveIndex + ", time: " + info2.time + ", point: " + info2.Pt.toString());
		if (info2.curveIndex > -1 && info2.curveIndex < curveA.length - 1 && info2.time >= 0 && info2.time <= 1) {
			// Visualize the closest point
			pushMatrix();
			translate(info2.Pt.x, info2.Pt.y, info2.Pt.z);
			sphere(0.1f * SCALE);
			popMatrix();
			// Visualize the tangent
			Vector3f ux = (debugPoints[0].subtract(info2.Pt)).normalize();
			Vector3f uy = info2.tangent;
			Vector3f px = info2.Pt.addScale(0.4f * SCALE, ux);
			Vector3f py = info2.Pt.addScale(0.7f * SCALE, uy);
			fill(green);
			showCylinder(info2.Pt, px, CURVE_CYLINDER_RADIUS / 1.5f, CURVE_CYLINDER_SAMPLES);
			showCylinder(info2.Pt, py, CURVE_CYLINDER_RADIUS / 2f, CURVE_CYLINDER_SAMPLES);
			//                System.out.println("px: " + px.toString() + ", py: " + py.toString());
		}
	}

	protected void calculateAndShowCurve() {
		noStroke();
		//Show control points
		fill(grey);
		for (Vector3f p : controlPoints) {
			pushMatrix();
			translate(p.x, p.y, p.z);
			sphere(0.1f * SCALE);
			popMatrix();
		}
		//calculate curves
		if (recalculateCurve) {
			recalculateCurve = false;
			//recalculate control curves
			if (interpolateControlCurve) {
				if (equispacedInterpolation) {
					samplesA = Curve.interpolateEquispacedArcLength(curveA, (curveA.length - 1) * CURVE_INTERPOLATION_SAMPLES);
					samplesB = Curve.interpolateEquispacedArcLength(curveB, (curveB.length - 1) * CURVE_INTERPOLATION_SAMPLES);
				} else {
					samplesA = Curve.interpolateUniformly(curveA, (curveA.length - 1) * CURVE_INTERPOLATION_SAMPLES);
					samplesB = Curve.interpolateUniformly(curveB, (curveB.length - 1) * CURVE_INTERPOLATION_SAMPLES);
				}
			}
			//trace medial axis
			ma.clear();
			trace(curveA, curveB, ma);
		}
		//show control curves
		if (interpolateControlCurve) {
			showQuads(samplesA, CURVE_CYLINDER_RADIUS, CURVE_CYLINDER_SAMPLES, blue);
			showQuads(samplesB, CURVE_CYLINDER_RADIUS, CURVE_CYLINDER_SAMPLES, green);
		} else {
			fill(blue);
			for (int i = 1; i < curveA.length; ++i) {
				showCylinder(curveA[i - 1], curveA[i], CURVE_CYLINDER_RADIUS / 3, CURVE_CYLINDER_SAMPLES);
			}
			fill(green);
			for (int i = 1; i < curveB.length; ++i) {
				showCylinder(curveB[i - 1], curveB[i], CURVE_CYLINDER_RADIUS / 3, CURVE_CYLINDER_SAMPLES);
			}
		}
		//show medial axis
		showMedialAxis();
	}
	
	private void showMedialAxis() {
		//interpolate center polyline
		Vector3f[] C = new Vector3f[ma.size()];
		for (int i=0; i<ma.size(); ++i) {
			C[i] = ma.get(i).center;
		}
		showQuads(C, CURVE_CYLINDER_RADIUS/2, CURVE_CYLINDER_SAMPLES, black);
		
		//show net to closest projections
		float step = (float) ma.size() / (float) MEDIAL_AXIS_NET_COUNT;
		for (float f=0; f<ma.size(); f+=step) {
			int i = (int) f;
			MedialAxisTransform.TracePoint p = ma.get(i);
			drawPoint(p.center, black);
			for (float t : p.projectionOnA) {
				Vector3f proj = Curve.interpolate(curveA, t);
				drawPoint(proj, black);
				showCylinder(p.center, proj, CURVE_CYLINDER_RADIUS / 4, CURVE_CYLINDER_SAMPLES);
			}
			for (float t : p.projectionOnB) {
				Vector3f proj = Curve.interpolate(curveB, t);
				drawPoint(proj, black);
				showCylinder(p.center, proj, CURVE_CYLINDER_RADIUS / 4, CURVE_CYLINDER_SAMPLES);
			}
		}
	}

	void showQuads(Vector3f[] C, float r, int ne, int col) {
		Vector3f[] L = new Vector3f[C.length];
		L[0] = C[1].subtract(C[0]).crossLocal(Vector3f.UNIT_Z).normalizeLocal();
		Vector3f[][] P = new Vector3f[2][ne];
		int p = 0;
		boolean dark = true;
		float[] c = new float[ne];
		float[] s = new float[ne];
		for (int j = 0; j < ne; j++) {
			c[j] = r * cos(TWO_PI * j / ne);
			s[j] = r * sin(TWO_PI * j / ne);
		}
		for (int j = 0; j < ne; j++) {
			P[p][j] = C[0].add(C[1]);
			P[p][j].addScaleLocal(c[j], L[0]);
			P[p][j].addScaleLocal(s[j], L[0].cross(C[1].subtract(C[0]).normalizeLocal()));
		}
		p = 1 - p;
		for (int i = 1; i < C.length - 1; i++) {
			dark = !dark;
			Vector3f I = C[i].subtract(C[i - 1]).normalizeLocal();
			Vector3f Ip = C[i + 1].subtract(C[i]).normalizeLocal();
			Vector3f IpmI = Ip.subtract(I);
			Vector3f N = I.cross(Ip);
			if (N.lengthSquared() < 0.001 * 0.001) {
				L[i] = L[i - 1];
			} else {
				float mixed = N.normalize().cross(I).dot(L[i - 1]);
				L[i] = L[i - 1].clone();
				L[i].addScaleLocal(mixed, N.normalize().cross(IpmI));
			}
			I = L[i].normalize();
			Vector3f J = I.cross(Ip).normalize();
			for (int j = 0; j < ne; j++) {
				P[p][j] = C[i].add(C[i + 1]);
				P[p][j].addScaleLocal(c[j], I);
				P[p][j].addScaleLocal(s[j], J);
			}
			p = 1 - p;
			if (i > 0) {
				for (int j = 0; j < ne; j++) {
					if (dark) {
						fill(200, 200, 200);
					} else {
						fill(col);
					}
					dark = !dark;
					int jp = (j + ne - 1) % ne;
					beginShape(QUADS);
					vertex(P[p][jp].mult(0.5f));
					vertex(P[p][j].mult(0.5f));
					vertex(P[1 - p][j].mult(0.5f));
					vertex(P[1 - p][jp].mult(0.5f));
					endShape(CLOSE);
				}
			}
			;
		}
	}

	void showCylinder(Vector3f A, Vector3f B, float r, int s) {
		Vector3f I = A.subtract(B).normalizeLocal(); // tangent
		Vector3f X = new Vector3f(1, 0, 0);
		if (Math.abs(X.dot(I)) > 0.9) {
			X = new Vector3f(0, 1, 0);
		}
		Vector3f J = X.cross(I).normalizeLocal();
		Vector3f K = I.cross(J).normalizeLocal();
		Vector3f P = A.addScale(r, J);
		Vector3f Q = B.addScale(r, J);
		beginShape(QUAD_STRIP);
		for (float t = 0; t <= TWO_PI; t += TWO_PI / s) {
			Vector3f v1 = A.addScale((float) (r * Math.cos(t)), J).addScale((float) (r * Math.sin(t)), K);
			Vector3f v2 = B.addScale((float) (r * Math.cos(t)), J).addScale((float) (r * Math.sin(t)), K);
			vertex(v1.x, v1.y, v1.z);
			vertex(v2.x, v2.y, v2.z);
		}
		endShape();
	}

	public void keyPressed() {

		if (key == '?') {
			scribeText = !scribeText;
		}
		if (key == '~') {
			filming = !filming;
		}
		if (key == '!') {
			takePicture = true;
		}
		if (key == 'G') {
			gouraud = !gouraud;
		}
		if (key == 'i') {
			interpolateControlCurve = !interpolateControlCurve;
			recalculateCurve = true;
		}
		if (key == 'e') {
			equispacedInterpolation = !equispacedInterpolation;
			recalculateCurve = true;
		}

		// if(key=='.') F=P.Picked(); // snaps focus F to the selected vertex of P (easier to rotate and zoom while keeping it in center)
		if (key == 'c') {
			center = !center; // snaps focus F to the selected vertex of P (easier to rotate and zoom while keeping it in center)
		}
		if (key == 't') {
			tracking = !tracking; // snaps focus F to the selected vertex of P (easier to rotate and zoom while keeping it in center)
		}
		if (key == 'a') {
			animating = !animating; // toggle animation
		}
		if (key == ',') {
			viewpoint = !viewpoint;
		}
		if (key == '#') {
			exit();
		}
		change = true;
	}

	public void mouseWheel(MouseEvent event) {
		dz += 20 * event.getAmount();
		change = true;
	}

	public void mousePressed() {
		pushMatrix();
		float fov = PI / 3.0f;
		float cameraZ = (height / 2.0f) / tan(fov / 2.0f);
		camera(0, 0, cameraZ, 0, 0, 0, 0, 1, 0);       // sets a standard perspective
		perspective(fov, 1.0f, 0.1f, 10000);
		translate(0, 0, dz); // puts origin of model at screen center and moves forward/away by dz
		rotateX(rx);
		rotateY(ry); // rotates the model around the new origin (center of screen)
		rotateX(PI / 2); // rotates frame around X to make X and Y basis vectors parallel to the floor

		float mx = mouseX;
		float my = mouseY;
//		System.out.println("Mouse X="+mx+" Y="+my);
		float dist = Float.MAX_VALUE;
		pickedPoint = 0;

		// See if the mouse is close to one of the control points
		for (int i = 0; i < controlPoints.length; ++i) {
			Vector3f p = controlPoints[i];
			float x = screenX(p.x, p.y, p.z);
			float y = screenY(p.x, p.y, p.z);
			float z = screenZ(p.x, p.y, p.z);
//			System.out.println("Point "+(i+1)+" X="+x+" Y="+y+" Z="+z);
			if (Math.abs(x - mx) > PICK_TOLERANCE || Math.abs(y - my) > PICK_TOLERANCE) {
				continue;
			}
			if (z < dist) {
				dist = z;
				pickedPoint = i + 1;
			}
		}

		// See if the mouse is close to one of the debug points
		for (int i = 0; i < debugPoints.length; ++i) {
			Vector3f p = debugPoints[i];
			float x = screenX(p.x, p.y, p.z);
			float y = screenY(p.x, p.y, p.z);
			float z = screenZ(p.x, p.y, p.z);
//			System.out.println("Point "+(i+1)+" X="+x+" Y="+y+" Z="+z);
			if (Math.abs(x - mx) > PICK_TOLERANCE || Math.abs(y - my) > PICK_TOLERANCE) {
				continue;
			}
			if (z < dist) {
				dist = z;
				pickedPoint = controlPoints.length + i + 1;
			}
		}
		System.out.println("Picked: " + pickedPoint);

		popMatrix();
	}

	@Override
	public void mouseReleased() {
		pickedPoint = 0;
	}

	public void mouseMoved() {
		if (keyPressed && key == ' ') {
			rx -= PI * (mouseY - pmouseY) / height;
			ry += PI * (mouseX - pmouseX) / width;
		};
		if (keyPressed && key == 's') {
			dz += (float) (mouseY - pmouseY); // approach view (same as wheel)
		}
	}

	public void mouseDragged() {
		if (pickedPoint == 0) {
			return;
		}
		Vector3f movement = new Vector3f();
		Pv3D.vec v = ToIJ(V((mouseX - pmouseX), 0, 0));
		movement.addLocal(v.x, v.y, v.z);
		v = ToK(V(0, (mouseY - pmouseY), 0));
		movement.addLocal(v.x, v.y, v.z);
		if (pickedPoint >= 1) {
			if (pickedPoint <= controlPoints.length) {
				controlPoints[pickedPoint - 1].addLocal(movement);
//                        System.out.println("pickedPoint: " + (pickedPoint - 1) + ", loc: " + controlPoints[pickedPoint-1].toString());
			} else {
//                        System.out.println("dbg loc: " + debugPoints[0].toString());
				debugPoints[pickedPoint - controlPoints.length - 1].addLocal(movement);
			}
		}
		recalculateCurve = true;
	}

// **** Header, footer, help text on canvas
	private void displayHeader() { // Displays title and authors face on screen
		scribeHeader(title, 0);
		scribeHeaderRight(name);
		fill(white);
		image(myFace, width - myFace.width / 2, 25, myFace.width / 2, myFace.height / 2);
	}

	private void displayFooter() { // Displays help text at the bottom
		scribeFooter(guide, 1);
		scribeFooter(menu, 0);
	}

	;
// ******************************** TEXT , TITLE, and USER's GUIDE
	Boolean scribeText = true; // toggle for displaying of help text
}
