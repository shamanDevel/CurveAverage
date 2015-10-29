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
import java.util.Random;
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
	private static final float MEDIAL_AXIS_RADIUS = 0.1f * SCALE;
	private static final float INTERPOLATED_AXIS_RADIUS = 0.07f * SCALE;
	private static final int MEDIAL_AXIS_GEODESIC_SAMPLE_COUNT = 50;
	private static final int MEDIAL_AXIS_CLOSEST_PROJECTIONS_COUNT = 20;
	private static final int MEDIAL_AXIS_ARC_COUNT = 8;
	private static final int MEDIAL_AXIS_ARC_RESOLUTION = 32;
	private static final int MEDIAL_AXIS_NET_COUNT = 7;
	private static final float MEDIAL_AXIS_NET_RADIUS = 0.02f * SCALE;
	private static final int MEDIAL_AXIS_ANIM_LENGTH = 4; //two cycles back and forth
	private static final float MEDIAL_AXIS_ANIM_SPEED = 1; //1/1 seconds per direction
	private static final int MEDIAL_AXIS_INFLATION_RESOLUTION = 32;

	private float dz = 0; // distance to camera. Manipulated with wheel or when 
//float rx=-0.06*TWO_PI, ry=-0.04*TWO_PI;    // view angles manipulated when space pressed but not mouse
	private float rx = 0, ry = 0;    // view angles manipulated when space pressed but not mouse
	private boolean animating = false, tracking = false, center = true, gouraud = true;
	private boolean showCoordinateAxes = false;
	
	private boolean interpolateControlCurve = true;
	private boolean equispacedInterpolation = true;
	private boolean showMedialAxis = true;
	private boolean geodesicMedialAxis = false;
	private boolean showClosestProjection = false;
	private boolean showCircularArcs = true;
	private boolean showNet = false;
	private boolean showInflation = false;
	private boolean showInflationWireframed = false;
	private boolean showHalfInflation = false;
	private int animDirection;
	private float animStep;
	private int animCounter;
	
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
			+ "s/wheel:closer, a:anim, x:show coordinate axes\n"
			//+ "i:interpolate control curve, e:equispaced interpolation, m:show medial axis, "
			+ "e:equispaced interpolation, m:show medial axis, g:geodesic sampling of the MA, "
			+ "p:show closest projections, c:show circular arc, n:show net (implies c)\n"
			+ "t:show inflation tube, w:wireframed inflation tube, h:show only half of the inflation tube",
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
			new Vector3f(-60, -40, -80), //start
			new Vector3f(-60, -10, -40),
			new Vector3f(-30, 12, -6),
			new Vector3f(0, 50, 60),
			new Vector3f(80, 45, 140),
			new Vector3f(170, 90, 200), //end
			new Vector3f(140, 70, 160),
			new Vector3f(100, 37, 100),
			new Vector3f(50, 19, 40),
			new Vector3f(0, 15, -20)
		};
//		for(int i = 0; i < controlPoints.length; i++) controlPoints[i].y = 0.0f;

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
		
		//test
		spiralControlPoints();

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
	private void spiralControlPoints() {
		curveA[0].set(0, 0, -50);
		curveA[5].set(0, 0, 50);
                curveB[0].set(0, 0, -50);
		curveB[5].set(0, 0, 50);
		curveA[1].set(-19, 0.01f, -30);
		curveA[2].set(-20, 0, -10);
		curveA[3].set(-20, 0.01f, 10);
		curveA[4].set(-19, 0, 30);
		curveB[1].set(19, 0.01f, -30);
		curveB[2].set(20, 0, -10);
		curveB[3].set(20, 0.01f, 10);
		curveB[4].set(19, 0, 30);
		//add noise
                
                // 
		Random rand = new Random();
                float randomness = 0.1f;
		for (int i=0; i<curveA.length; ++i) {
			curveA[i].addLocal(rand.nextFloat()*randomness, rand.nextFloat()*randomness, rand.nextFloat()*randomness);
		}
                for (int i=0; i<curveB.length; ++i) {
			curveB[i].addLocal(rand.nextFloat()*randomness, rand.nextFloat()*randomness, rand.nextFloat()*randomness);
		}
                curveB[0] = curveA[0];
                curveB[5] = curveA[5];
                for (int i = 0; i < curveA.length; ++i) {
                    controlPoints[i] = curveA[i];
		}
		for (int i = 1; i < curveB.length; ++i) {
                    controlPoints[controlPoints.length - i] = curveB[i];
		}
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
		if (showCoordinateAxes) {
			showFrame(-PI / 2, PI / 2, 0, 50); // X-red, Y-green, Z-blue arrows
		}

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
		 Vector3f px = info1.Pt.addScaled(0.4f * SCALE, ux);
		 Vector3f py = info1.Pt.addScaled(0.7f * SCALE, uy);
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
			Vector3f px = info2.Pt.addScaled(0.4f * SCALE, ux);
			Vector3f py = info2.Pt.addScaled(0.7f * SCALE, uy);
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
			if (geodesicMedialAxis) {
				MedialAxisTransform.geodesicTrace(curveA, curveB, MEDIAL_AXIS_GEODESIC_SAMPLE_COUNT, ma);
			} else {
				MedialAxisTransform.trace(curveA, curveB, ma);
			}
		}

		//show control curves
		if (interpolateControlCurve) {
			showQuads(samplesA, CURVE_CYLINDER_RADIUS, CURVE_CYLINDER_SAMPLES, blue, true);
			showQuads(samplesB, CURVE_CYLINDER_RADIUS, CURVE_CYLINDER_SAMPLES, green, true);
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
		if (showMedialAxis) {
			Vector3f[] medialAxis = new Vector3f[ma.size()];
			for (int i=0; i<ma.size(); ++i) {
				medialAxis[i] = ma.get(i).center;
			}
			showQuads(medialAxis, MEDIAL_AXIS_RADIUS, CURVE_CYLINDER_SAMPLES, black, true);
		}
	
		//show closest projections
		if (showClosestProjection) {
			int step = (int) (ma.size() / (float) MEDIAL_AXIS_CLOSEST_PROJECTIONS_COUNT);
			for (float f=step; f<ma.size(); f+=step) {
				int i = (int) f;
				MedialAxisTransform.TracePoint p = ma.get(i);
				drawPoint(p.center, black);
				for (float t : p.projectionOnA) {
					Vector3f proj = Curve.interpolate(curveA, t);
					drawPoint(proj, black);
					showCylinder(p.center, proj, MEDIAL_AXIS_NET_RADIUS, CURVE_CYLINDER_SAMPLES);
				}
				for (float t : p.projectionOnB) {
					Vector3f proj = Curve.interpolate(curveB, t);
					drawPoint(proj, black);
					showCylinder(p.center, proj, MEDIAL_AXIS_NET_RADIUS, CURVE_CYLINDER_SAMPLES);
				}
			}
		}
        
                //show circular arcs
		if (showCircularArcs || showNet) {
                        assert(ma.size() >= MEDIAL_AXIS_ARC_COUNT);     // to ensure the division below makes sense and step is not 0
			int step = (int) (ma.size() / (float) MEDIAL_AXIS_ARC_COUNT);
            
			for (float f=step; f<ma.size(); f+=step) {

				int i = (int) f;
				MedialAxisTransform.TracePoint p = ma.get(i);
				Vector3f P = p.center;
				Vector3f A = Curve.interpolate(curveA, p.projectionOnA[0]);
				Vector3f B = Curve.interpolate(curveB, p.projectionOnB[0]);

				//compute arc
				CircularArc ca = new CircularArc(P, A, B);

				//draw arc
				Vector3f[] arc = new Vector3f[MEDIAL_AXIS_ARC_RESOLUTION];
				for (int j=0; j<arc.length; ++j) {
					float t = j / (float) (arc.length-1);
					arc[j] = ca.getPointOnArc(t);
				}

				showQuads(arc, MEDIAL_AXIS_NET_RADIUS, 4, black, false);

//				drawPoint(ca.getCenter(), yellow);
			}
		}
            
		//show net
		if (showNet) {
			//compute circular arcs
			CircularArc[] arcs = new CircularArc[ma.size()];
			for (int i=0; i<ma.size(); ++i) {
				MedialAxisTransform.TracePoint p = ma.get(i);
				Vector3f P = p.center;
				Vector3f A = Curve.interpolate(curveA, p.projectionOnA[0]);
				Vector3f B = Curve.interpolate(curveB, p.projectionOnB[0]);
				arcs[i] = new CircularArc(P, A, B);
			}
			//sample net curve
			float step = 1f / MEDIAL_AXIS_NET_COUNT;
			for (float f=step; f<1; f+=step) {
				Vector3f[] points = new Vector3f[arcs.length];
				for (int i=0; i<arcs.length; ++i) {
					points[i] = arcs[i].getPointOnArc(f);
				}
				showQuads(points, MEDIAL_AXIS_NET_RADIUS, 4, black, false);
			}
		}
            
		//show animation
		if (animating) {
			animStep += tpf*animDirection*MEDIAL_AXIS_ANIM_SPEED;
			if (animStep>1) {
				animDirection = -1;
				animCounter++;
				animStep = 1;
			} else if (animStep<0) {
				animDirection = 1;
				animCounter++;
				animStep = 0;
			}
			if (animCounter==MEDIAL_AXIS_ANIM_LENGTH) {
				animating = false;
				animStep = 0;
				animDirection = 1;
				animCounter = 0;
			} else {
				//draw it
				Vector3f[] interpolatedAxis = new Vector3f[ma.size()];
				for (int i=0; i<ma.size(); ++i) {
					MedialAxisTransform.TracePoint p = ma.get(i);
					Vector3f P = p.center;
					Vector3f A = Curve.interpolate(curveA, p.projectionOnA[0]);
					Vector3f B = Curve.interpolate(curveB, p.projectionOnB[0]);
					CircularArc arc = new CircularArc(P, A, B);
					interpolatedAxis[i] = arc.getPointOnArc(animStep);
				}
				showQuads(interpolatedAxis, INTERPOLATED_AXIS_RADIUS, CURVE_CYLINDER_SAMPLES, red, true);
			}
		}
            
		//show inflation
		if (showInflation) {
			Vector3f[] medialAxis = new Vector3f[ma.size()];
			float[] radii = new float[ma.size()];
			Vector3f[] A = new Vector3f[ma.size()];
			Vector3f[] B = new Vector3f[ma.size()];
			for (int i=0; i<ma.size(); ++i) {
				MedialAxisTransform.TracePoint p = ma.get(i);
				medialAxis[i] = p.center;
				radii[i] = p.radius*2;
				A[i] = Curve.interpolate(curveA, p.projectionOnA[0]);
				B[i] = Curve.interpolate(curveB, p.projectionOnB[0]);
			}
			showInflation(A, B, medialAxis, radii, 
					showHalfInflation ? MEDIAL_AXIS_INFLATION_RESOLUTION/2 : MEDIAL_AXIS_INFLATION_RESOLUTION, 
					grey80, lightgrey80, showInflationWireframed, showHalfInflation, !showHalfInflation);
		}
            
	}

	/**
	 * Show a quad tube
	 * @param C the control points of that tube
	 * @param r the radius of the tube
	 * @param ne the resolution of the tube
	 * @param col the color
	 * @param checked if true, a checkered pattern is applied
	 */
	void showQuads(Vector3f[] C, float r, int ne, int col, boolean checked) {
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
			P[p][j].addScaledLocal(c[j], L[0]);
			P[p][j].addScaledLocal(s[j], L[0].cross(C[1].subtract(C[0]).normalizeLocal()));
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
				L[i].addScaledLocal(mixed, N.normalize().cross(IpmI));
			}
			I = L[i].normalize();
			Vector3f J = I.cross(Ip).normalize();
			for (int j = 0; j < ne; j++) {
				P[p][j] = C[i].add(C[i + 1]);
				P[p][j].addScaledLocal(c[j], I);
				P[p][j].addScaledLocal(s[j], J);
			}
			p = 1 - p;
			if (i > 0) {
				for (int j = 0; j < ne; j++) {
					if (dark && checked) {
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
		}
	}
	
	/**
	 * Show a quad tube with variable radius.
	 * the arrays C and r must be of equal length.
	 * @param C the control points of that tube
	 * @param r the radius of the tube at the same control point
	 * @param ne the resolution of the tube
	 * @param col1 the first color
	 * @param col2 the second color
	 */
	void showQuads(Vector3f[] C, float[] r, int ne, int col1, int col2, boolean wireframe) {
		Vector3f[] L = new Vector3f[C.length];
		L[0] = C[1].subtract(C[0]).crossLocal(Vector3f.UNIT_Z).normalizeLocal();
		Vector3f[][] P = new Vector3f[2][ne];
		int p = 0;
		boolean dark = true;
		float[] c = new float[ne];
		float[] s = new float[ne];
		for (int j = 0; j < ne; j++) {
			c[j] = cos(TWO_PI * j / ne);
			s[j] = sin(TWO_PI * j / ne);
		}
		for (int j = 0; j < ne; j++) {
			P[p][j] = C[0].add(C[1]);
			P[p][j].addScaledLocal(c[j]*(r[1]+r[0])/2, L[0]);
			P[p][j].addScaledLocal(s[j]*(r[1]+r[0])/2, L[0].cross(C[1].subtract(C[0]).normalizeLocal()));
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
				L[i].addScaledLocal(mixed, N.normalize().cross(IpmI));
			}
			I = L[i].normalize();
			Vector3f J = I.cross(Ip).normalize();
			for (int j = 0; j < ne; j++) {
				P[p][j] = C[i].add(C[i + 1]);
				P[p][j].addScaledLocal(c[j]*(r[i]+r[i+1])/2, I);
				P[p][j].addScaledLocal(s[j]*(r[i]+r[i+1])/2, J);
			}
			p = 1 - p;
			if (i > 0) {
				for (int j = 0; j < ne; j++) {
					int jp = (j + ne - 1) % ne;
					Vector3f P1 = P[p][jp].mult(0.5f);
					Vector3f P2 = P[p][j].mult(0.5f);
					Vector3f P3 = P[1 - p][j].mult(0.5f);
					Vector3f P4 = P[1 - p][jp].mult(0.5f);
					if (dark) {
						fill(col1);
					} else {
						fill(col2);
					}
					dark = !dark;
					if (wireframe) {
						noFill();
						stroke(black);
						beginShape(LINES);
						vertex(P1);
						vertex(P2);
						vertex(P2);
						vertex(P3);
						vertex(P3);
						vertex(P4);
						vertex(P4);
						vertex(P1);
						endShape();
					} else {
						beginShape(QUADS);
						vertex(P1);
						vertex(P2);
						vertex(P3);
						vertex(P4);
						endShape(CLOSE);
					}
				}
			}
		}
	}
	
	/**
	 * Show a half quad tube with variable radius.
	 * the arrays A,B,C and r must be of equal length.
	 * No parallel transport is used here
	 * @param A the points on the left side of the tube
	 * @param B the points on the right side of the tube
	 * @param C the control points / center points of that tube
	 * @param r the radius of the tube at the same control point
	 * @param ne the resolution of the tube
	 * @param col1 the first color
	 * @param col2 the second color
	 */
	void showInflation(Vector3f[] A, Vector3f[] B, Vector3f[] C, float[] r, int ne, int col1, int col2, 
			boolean wireframe, boolean half, boolean parallelTransport) {
		boolean dark = true;
		Vector3f[] CAs = new Vector3f[A.length]; //rotation axis
		Vector3f[] Ns = new Vector3f[A.length]; //rotation normals
		float[] angles = new float[A.length]; //start angles
		CAs[0] = Vector3f.ZERO;
		Ns[0] = Vector3f.ZERO;
		angles[0] = 0;
		for (int i = 1; i < C.length - 1; i++) {
			Vector3f N = C[i+1].subtract(C[i-1]).normalizeLocal();
			Vector3f CA = A[i].subtract(C[i]).normalizeLocal();
			float angle = 0;
			if (parallelTransport) {
				//set start angle according to parallel transport rules
				Vector3f CAp = CAs[i-1];
				if (CAp.equals(Vector3f.ZERO)) {
					//first real vector, I can choose any angle, so keep 0°
				} else {
					Vector3f reference = rotate(angles[i-1], CAp, Ns[i-1]);
					//compute the angle so that CA°angle lies in the plane N x reference
					Vector3f refPrime = reference.normalize();
					Vector3f NPrime = N.cross(refPrime);
					Vector3f CAPrime = CA.normalize();
					float x = CAPrime.dot(NPrime);
					float y = CAPrime.dot(refPrime);
					Vector3f CAproj = new Vector3f();
					CAproj.addScaledLocal(x, NPrime);
					CAproj.addScaledLocal(y, refPrime);
					CAproj.normalizeLocal();
					angle = Math.abs(refPrime.angleBetween(CAproj));
					if (x<0) {
						angle = -angle;
					}
				}
			}
			//set arrays
			CAs[i] = CA;
			Ns[i] = N;
			angles[i] = angle;
		}
		CAs[A.length-1] = Vector3f.ZERO;
		Ns[A.length-1] = Vector3f.ZERO;
		angles[A.length-1] = 0;
		//draw the tube
		float step = (float) ((half ? Math.PI : 2*Math.PI) / ne); //if the medial axis was traced correctly, the angle should always be 180°
		for (int i = 0; i < C.length - 1; i++) {
			dark = !dark;
			for (int j = 0; j < ne; j++) {
				Vector3f P1 = rotate(j*step + angles[i], CAs[i].mult(r[i]/2), Ns[i]).add(C[i]);
				Vector3f P2 = rotate(j*step + angles[i+1], CAs[i+1].mult(r[i+1]/2), Ns[i+1]).add(C[i+1]);
				Vector3f P3 = rotate((j+1)*step + angles[i+1], CAs[i+1].mult(r[i+1]/2), Ns[i+1]).add(C[i+1]);
				Vector3f P4 = rotate((j+1)*step + angles[i], CAs[i].mult(r[i]/2), Ns[i]).add(C[i]);
				if (dark) {
					fill(col1);
				} else {
					fill(col2);
				}
				dark = !dark;
				if (wireframe) {
					noFill();
					stroke(black);
					beginShape(LINES);
					vertex(P1);
					vertex(P2);
					vertex(P2);
					vertex(P3);
					vertex(P3);
					vertex(P4);
					vertex(P4);
					vertex(P1);
					endShape();
				} else {
					beginShape(QUADS);
					vertex(P1);
					vertex(P2);
					vertex(P3);
					vertex(P4);
					endShape(CLOSE);
				}
			}
		}
	}
	private static Vector3f rotate(float angle, Vector3f X, Vector3f N) {
		Vector3f W = N.mult(N.dot(X));
		Vector3f U = X.subtract(W);
		return W.add(U.mult((float) Math.cos(angle)))
				.subtract(N.cross(U).mult((float) Math.sin(angle)));
	}

	/**
	 * Shows a cyclinder from A to B with radius r and resolution s
	 * @param A
	 * @param B
	 * @param r
	 * @param s 
	 */
	void showCylinder(Vector3f A, Vector3f B, float r, int s) {
		Vector3f I = A.subtract(B).normalizeLocal(); // tangent
		Vector3f X = new Vector3f(1, 0, 0);
		if (Math.abs(X.dot(I)) > 0.9) {
			X = new Vector3f(0, 1, 0);
		}
		Vector3f J = X.cross(I).normalizeLocal();
		Vector3f K = I.cross(J).normalizeLocal();
		Vector3f P = A.addScaled(r, J);
		Vector3f Q = B.addScaled(r, J);
		beginShape(QUAD_STRIP);
		for (float t = 0; t <= TWO_PI; t += TWO_PI / s) {
			Vector3f v1 = A.addScaled((float) (r * Math.cos(t)), J).addScaled((float) (r * Math.sin(t)), K);
			Vector3f v2 = B.addScaled((float) (r * Math.cos(t)), J).addScaled((float) (r * Math.sin(t)), K);
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
		if (key == 'g') {
			geodesicMedialAxis = !geodesicMedialAxis;
			recalculateCurve = true;
		}
		if (key == 'm') {
			showMedialAxis = !showMedialAxis;
		}
		if (key == 'p') {
			showClosestProjection = !showClosestProjection;
		}
		if (key == 'c') {
			showCircularArcs = !showCircularArcs;
		}
		if (key == 'n') {
			showNet = !showNet;
		}
		if (key == 't') {
			showInflation = !showInflation;
		}
		if (key == 'w') {
			showInflationWireframed = !showInflationWireframed;
		}
		if (key == 'h') {
			showHalfInflation = !showHalfInflation;
		}
		if (key == 'x') {
			showCoordinateAxes = !showCoordinateAxes;
		}

		// if(key=='.') F=P.Picked(); // snaps focus F to the selected vertex of P (easier to rotate and zoom while keeping it in center)
		if (key == 'f') {
			center = !center; // snaps focus F to the selected vertex of P (easier to rotate and zoom while keeping it in center)
		}
		if (key == 't') {
			tracking = !tracking; // snaps focus F to the selected vertex of P (easier to rotate and zoom while keeping it in center)
		}
		if (key == 'a') {
			animating = !animating; // toggle animation
			animDirection = 1;
			animStep = 0;
			animCounter = 0;
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
		String menu2[] = menu.split("\\n");
		scribeFooter(guide, menu2.length);
		for (int i=0; i<menu2.length; ++i) {
			scribeFooter(menu2[i], menu2.length-i-1);
		}
	}

	;
// ******************************** TEXT , TITLE, and USER's GUIDE
	Boolean scribeText = true; // toggle for displaying of help text
}
