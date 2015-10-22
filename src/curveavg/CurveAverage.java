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
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import static curveavg.Pv3D.*;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

/**
 *
 * @author Sebastian Weiss
 */
public class CurveAverage extends PApplet {

	public static final float SCALE = 50;
	private static final float PICK_TOLERANCE = 20;
	public static final float DEG_TO_RAD = PI / 180.0f;
	private static final float FPS = 60;
	private static final int CURVE_INTERPOLATION_SAMPLES = 32;
	private static final int CURVE_CYLINDER_SAMPLES = 8;
	private static final float CURVE_CYLINDER_RADIUS = 0.15f*SCALE;

	private float dz = 0; // distance to camera. Manipulated with wheel or when 
//float rx=-0.06*TWO_PI, ry=-0.04*TWO_PI;    // view angles manipulated when space pressed but not mouse
	private float rx = 0, ry = 0;    // view angles manipulated when space pressed but not mouse
	private  boolean animating = false, tracking = false, center = true, gouraud = true;
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

	String title = "6491 P2 2015: 3D swirl", name = "Sebastian Weiß, Can Erdogan",
			menu = "!:picture, ~:(start/stop)capture, space:rotate, "
			+ "s/wheel:closer, a:anim, i:interpolate control curve, e:equispaced interpolation, #:quit",
			guide = "click'n'drag center of frames or arrow tips to change the start frame (green) and end frame (red)"; // user's guide

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
		controlPoints = new Vector3f[] {
			new Vector3f(-50, -30, -70), //start
			new Vector3f(-60, 10, -40),
			new Vector3f(-70, 12, -6),
			new Vector3f(0, 50, 60),
			new Vector3f(80, 45, 140),
			new Vector3f(150, 80, 200), //end
			new Vector3f(140, 90, 160),
			new Vector3f(100, 37, 100),
			new Vector3f(50, 39, 40),
                        new Vector3f(17.307629f, 0.0f, -47.95851f)
//			new Vector3f(0, 15, -20)
		};
                for(int i = 0; i < controlPoints.length; i++)
                    controlPoints[i].y = 0.0f;
                
		curveA = new Vector3f[controlPoints.length / 2 + 1];
		curveB = new Vector3f[controlPoints.length / 2 + 1];
		for (int i=0; i<curveA.length; ++i) {
			curveA[i] = controlPoints[i];
		}
		curveB[0] = controlPoints[0];
		for (int i=1; i<curveB.length; ++i) {
			curveB[i] = controlPoints[controlPoints.length - i];
		}
		recalculateCurve = true;

		rx+=0.0001f;
		ry+=0.0001f;
		
		time1 = System.currentTimeMillis();
		time2 = System.currentTimeMillis();
                
                // Add a debug point
                debugPoints = new Vector3f[] {
                    //new Vector3f(34,35,56)};
                    new Vector3f(-47.615837f, 0.0f, -38.66822f)};
                /*
                    new Vector3f(150, 90, 200),
                    new Vector3f(-70, -30, -70)
                };
                        */
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
	public void trace(Vector3f[] curveA, Vector3f[] curveB, List<MedialAxisTransform.TracePoint> output) {
		
            // Draw the initial lines
            /*
            Vector3f Q0 = (Curve.interpolate(curveA, 0.01f).add(Curve.interpolate(curveB, 0.1f))).mult(0.5f);
            drawPoint(Q0, cyan);

            System.out.println("\n\nQ0: " + Q0.toString());
            MedialAxisTransform.ClosestInfo info1a = MedialAxisTransform.findClosest(curveA, Q0);
            MedialAxisTransform.ClosestInfo info1b = MedialAxisTransform.findClosest(curveB, Q0);
            System.out.println("Tangent A: " + info1a.tangent.toString() + ", tangent B: " + info1b.tangent.toString());
            System.out.println("info1a index: " + info1a.curveIndex + ", info1b index: " + info1b.curveIndex);
                        drawPoint(info1a.Pt, orange);

//            fill(blue);
//            showCylinder(info1a.Pt.addScale(-3*SCALE, info1a.tangent), info1a.Pt.addScale(3*SCALE, info1a.tangent), 2, 8);
//            fill(green);
//            showCylinder(info1b.Pt.addScale(-3*SCALE, info1b.tangent), info1b.Pt.addScale(3*SCALE, info1b.tangent), 2, 8);
*/
            MedialAxisTransform.ClosestInfo infoA = MedialAxisTransform.findClosest(curveA, Curve.interpolate(curveA, 0.15f));
            MedialAxisTransform.ClosestInfo infoB = MedialAxisTransform.findClosest(curveB, Curve.interpolate(curveB, 0.15f));
            Vector3f Q1 = curveA[0];

            final float stepSize = 10.0f;
            for(int idx = 0; idx < 1; idx++) {

                System.out.println("Iteration " + idx + "---------------------");
                // Find the medial axis of the tangent lines
                MedialAxisTransform.Line line = MedialAxisTransform.medialAxisLine(infoA.Pt, infoA.tangent, 
                        infoB.Pt, infoB.tangent);
                System.out.println("Line p: " + line.p.toString() + ", line v: " + line.v.toString());
                fill(red);
                showCylinder(Q1.addScale(-3*SCALE, line.v), Q1.addScale(3*SCALE, line.v), 2, 8);
                Q1 = Q1.addScale(stepSize, line.v);
                drawPoint(Q1, red);

                // Find the closest point to the first estimate 
                System.out.println("Q1 initial: " + Q1.toString());

                // Move the estimate in the negative average direction of the projections
                // until the difference in their distances vanishes
                int counter = 0;
                while(true) {

                    // Find the projections and the distances
                    infoA = MedialAxisTransform.findClosest(curveA, Q1);
                    infoB = MedialAxisTransform.findClosest(curveB, Q1);
                    if(counter == 0) {
                        drawPoint(infoB.Pt, magenta);
                    }
                    assert(infoA.found);
                    assert(infoB.found);
                    float distA = Q1.distance(infoA.Pt);
                    float distB = Q1.distance(infoB.Pt);
                    float err = distA-distB;
                    System.out.println("distA: " + distA + ", distB: " + distB + ", |diff|: " + Math.abs(err) + ", dirA: " + infoA.dir.toString() + ", dirB: " + infoB.dir.toString());
                    if(Math.abs(err) < 1e-3) {
//                        System.out.println("Converged...");
                        break;
                    }

                    // Move the point in the average direction
//                    System.out.println("
                    Vector3f dir = (infoA.dir.subtract(infoB.dir)).normalize();
                    Q1 = Q1.addScaleLocal(-0.5f*err, dir);
                    if(counter++ > 10) break;
                }
                System.out.println("Q1 later: " + Q1.toString());
                drawPoint(Q1, orange);

                infoA = MedialAxisTransform.findClosest(curveA, Q1);
                infoB = MedialAxisTransform.findClosest(curveB, Q1);
                drawPoint(infoA.Pt, blue);
//                showCylinder(infoA.Pt.addScale(-1*SCALE, infoA.dir), infoA.Pt.addScale(1*SCALE, infoA.dir), 2, 8);
                drawPoint(infoB.Pt, green);
//                showCylinder(infoB.Pt.addScale(-1*SCALE, infoB.dir), infoB.Pt.addScale(1*SCALE, infoB.dir), 2, 8);
                
            }
            /*
            // Start with the common point 
            Vector3f point = curveA[0];
            
            while(true) {
                
                // Find which segment of the curves the point coincides to
                
            }
                    */
                
	}
        
        public void drawPoint (Vector3f P, int color) {
            fill(color);
            pushMatrix();
            translate(P.x, P.y, P.z);
            sphere(0.1f*SCALE);
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
		showFrame(-PI/2, PI/2, 0, 50); // X-red, Y-green, Z-blue arrows

		computeProjectedVectors(); // computes screen projections I, J, K of basis vectors (see bottom of pv3D): used for dragging in viewer's frame    
		
		calculateAndShowCurve();
		
                // Draw the debug points
                		noStroke();
                                
		//Show debug  points
		fill(red);
		for (Vector3f p : debugPoints) {
                    pushMatrix();
                    translate(p.x, p.y, p.z);
                    sphere(0.1f*SCALE);
                    popMatrix();
		}
                debugPoints[0].y = 0.0f;
                
                // Visualize the closest points and the tangents
                boolean visualizeClosest = true;
                if(visualizeClosest) {
                    showClosestPointsAndTangents();
                }
                
                
                
		List<MedialAxisTransform.TracePoint> ma = new ArrayList <MedialAxisTransform.TracePoint> ();
//                trace(curveA, curveB, ma);
//                exit();
                
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
	
        private void showClosestPointsAndTangents () {
            // Handle the first curve
//            System.out.println("Input point: " + debugPoints[0].toString());

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

            // Handle the second curve
            MedialAxisTransform.ClosestInfo info2 = MedialAxisTransform.findClosest(curveB, debugPoints[0]);
//            System.out.println("First curve: index: " + info2.curveIndex + ", time: " + info2.time + ", point: " + info2.Pt.toString());
            if(info2.curveIndex > -1 && info2.curveIndex < curveA.length-1 && info2.time >= 0 && info2.time <= 1) {

                // Visualize the closest point
                pushMatrix();
                translate(info2.Pt.x, info2.Pt.y, info2.Pt.z);
                sphere(0.1f*SCALE);
                popMatrix();

                // Visualize the tangent
                Vector3f ux = (debugPoints[0].subtract(info2.Pt)).normalize();
                Vector3f uy = info2.tangent;
                Vector3f px = info2.Pt.addScale(0.4f * SCALE, ux);
                Vector3f py = info2.Pt.addScale(0.7f * SCALE, uy);
                fill(green);
                showCylinder(info2.Pt, px, CURVE_CYLINDER_RADIUS/1.5f, CURVE_CYLINDER_SAMPLES);
                showCylinder(info2.Pt, py, CURVE_CYLINDER_RADIUS/2f, CURVE_CYLINDER_SAMPLES);
//                System.out.println("px: " + px.toString() + ", py: " + py.toString());

             }
        }
	private void calculateAndShowCurve() {
		noStroke();
		//Show control points
		fill(grey);
		for (Vector3f p : controlPoints) {
			pushMatrix();
			translate(p.x, p.y, p.z);
			sphere(0.1f*SCALE);
			popMatrix();
		}
		
		//calculate curves
		if (recalculateCurve) {
			recalculateCurve = false;
			if (interpolateControlCurve) {
				if (equispacedInterpolation) {
					samplesA = Curve.interpolateEquispacedArcLength(curveA, (curveA.length-1)*CURVE_INTERPOLATION_SAMPLES);
					samplesB = Curve.interpolateEquispacedArcLength(curveB, (curveB.length-1)*CURVE_INTERPOLATION_SAMPLES);
				} else {
					samplesA = Curve.interpolateUniformly(curveA, (curveA.length-1)*CURVE_INTERPOLATION_SAMPLES);
					samplesB = Curve.interpolateUniformly(curveB, (curveB.length-1)*CURVE_INTERPOLATION_SAMPLES);
				}
			}
		}
		
		//show curves
		if (interpolateControlCurve) {
			showQuads(samplesA, CURVE_CYLINDER_RADIUS, CURVE_CYLINDER_SAMPLES, blue);
			showQuads(samplesB, CURVE_CYLINDER_RADIUS, CURVE_CYLINDER_SAMPLES, green);
		} else {
			fill(blue);
			for (int i=1; i<curveA.length; ++i) {
				showCylinder(curveA[i-1], curveA[i], CURVE_CYLINDER_RADIUS/3, CURVE_CYLINDER_SAMPLES);
			}
			fill(green);
			for (int i=1; i<curveB.length; ++i) {
				showCylinder(curveB[i-1], curveB[i], CURVE_CYLINDER_RADIUS/3, CURVE_CYLINDER_SAMPLES);
			}
		}
	}
	
	void showQuads(Vector3f[] C, float r, int ne, int col) {
		Vector3f L[] = new Vector3f[C.length];
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
			Vector3f I = C[i].subtract(C[i-1]).normalizeLocal();
			Vector3f Ip = C[i+1].subtract(C[i]).normalizeLocal();
			Vector3f IpmI = Ip.subtract(I);
			Vector3f N = I.cross(Ip);
			if (N.lengthSquared() < 0.001*0.001) {
				L[i] =L[i - 1];
			} else {
				float mixed = N.normalize().cross(I).dot(L[i-1]);
				L[i] = L[i-1].clone();
				L[i].addScaleLocal(mixed, N.normalize().cross(IpmI));
			}
			I = L[i].normalize();
			Vector3f J = I.cross(Ip).normalize();
			for (int j = 0; j < ne; j++) {
				P[p][j] = C[i].add(C[i+1]);
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
			};
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
			Vector3f v1 = A.addScale((float) (r*Math.cos(t)), J)
					.addScale((float) (r*Math.sin(t)), K);
			Vector3f v2 = B.addScale((float) (r*Math.cos(t)), J)
					.addScale((float) (r*Math.sin(t)), K);
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
		for (int i=0; i<controlPoints.length; ++i) {
			Vector3f p = controlPoints[i];
			float x = screenX(p.x, p.y, p.z);
			float y = screenY(p.x, p.y, p.z);
			float z = screenZ(p.x, p.y, p.z);
//			System.out.println("Point "+(i+1)+" X="+x+" Y="+y+" Z="+z);
			if (Math.abs(x-mx) > PICK_TOLERANCE || Math.abs(y-my) > PICK_TOLERANCE)
				continue;
			if (z < dist) {
				dist = z;
				pickedPoint = i + 1;
			}
		}
                
                // See if the mouse is close to one of the debug points
                for (int i=0; i<debugPoints.length; ++i) {
                    Vector3f p = debugPoints[i];
                    float x = screenX(p.x, p.y, p.z);
                    float y = screenY(p.x, p.y, p.z);
                    float z = screenZ(p.x, p.y, p.z);
//			System.out.println("Point "+(i+1)+" X="+x+" Y="+y+" Z="+z);
                    if (Math.abs(x-mx) > PICK_TOLERANCE || Math.abs(y-my) > PICK_TOLERANCE)
                        continue;
                    if (z < dist) {
                        dist = z;
                        pickedPoint = controlPoints.length + i + 1;
                    }
		}
		System.out.println("Picked: "+pickedPoint);
		
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
		Pv3D.vec v = ToIJ(V((mouseX-pmouseX),0,0));
		movement.addLocal(v.x, v.y, v.z);
		v = ToK(V(0, (mouseY-pmouseY),0));
		movement.addLocal(v.x, v.y, v.z);
		if (pickedPoint >= 1) {
                    if(pickedPoint <= controlPoints.length) {
                        controlPoints[pickedPoint-1].addLocal(movement);
                        System.out.println("pickedPoint: " + (pickedPoint - 1) + ", loc: " + controlPoints[pickedPoint-1].toString());
                    }
                    else {
                        System.out.println("dbg loc: " + debugPoints[0].toString());
                        debugPoints[pickedPoint-controlPoints.length-1].addLocal(movement);
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

	Pv3D.pt ToScreen(Pv3D.pt P) {
		return new Pv3D.pt(screenX(P.x, P.y, P.z), screenY(P.x, P.y, P.z), 0);
	}  // O+xI+yJ+kZ

	Pv3D.pt ToModel(Pv3D.pt P) {
		return new Pv3D.pt(modelX(P.x, P.y, P.z), modelY(P.x, P.y, P.z), modelZ(P.x, P.y, P.z));
	}  // O+xI+yJ+kZ

// ===== mouse
	Pv3D.pt Mouse() {
		return new Pv3D.pt(mouseX, mouseY, 0);
	}

	;                                          // current mouse location
Pv3D.pt Pmouse() {
		return new Pv3D.pt(pmouseX, pmouseY, 0);
	}

	;
	Pv3D.vec MouseDrag() {
		return new Pv3D.vec(mouseX - pmouseX, mouseY - pmouseY, 0);
	}

	;                     // vector representing recent mouse displacement
Pv3D.pt ScreenCenter() {
		return new Pv3D.pt(width / 2, height / 2);
	}                                                        //  point in center of  canvas

// ===== render
	void normal(Pv3D.vec V) {
		normal(V.x, V.y, V.z);
	}

	;                                          // changes normal for smooth shading
void vertex(Pv3D.pt P) {
		vertex(P.x, P.y, P.z);
	}
void vertex(Vector3f P) {
	vertex(P.x, P.y, P.z);
}

	;                                           // vertex for shading or drawing
void v(Pv3D.pt P) {
		vertex(P.x, P.y, P.z);
	}

	;                                           // vertex for shading or drawing
void nv(Pv3D.vec N) {
		normal(N.x, N.y, N.z);
	}

	;                                           // vertex for shading or drawing
void vTextured(Pv3D.pt P, float u, float v) {
		vertex(P.x, P.y, P.z, u, v);
	}

	;                          // vertex with texture coordinates
void show(Pv3D.pt P, Pv3D.pt Q) {
		line(Q.x, Q.y, Q.z, P.x, P.y, P.z);
	}

	;                       // draws edge (P,Q)
void show(Pv3D.pt P, Pv3D.vec V) {
		line(P.x, P.y, P.z, P.x + V.x, P.y + V.y, P.z + V.z);
	}

	;          // shows edge from P to P+V
void show(Pv3D.pt P, float d, Pv3D.vec V) {
		line(P.x, P.y, P.z, P.x + d * V.x, P.y + d * V.y, P.z + d * V.z);
	}

	; // shows edge from P to P+dV
void show(Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		beginShape();
		vertex(A);
		vertex(B);
		vertex(C);
		endShape(CLOSE);
	}

	;                      // volume of tet 
void show(Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt D) {
		beginShape();
		vertex(A);
		vertex(B);
		vertex(C);
		vertex(D);
		endShape(CLOSE);
	}

	;                      // volume of tet 
void show(Pv3D.pt P, float r) {
		pushMatrix();
		translate(P.x, P.y, P.z);
		sphere(r);
		popMatrix();
	}

	; // render sphere of radius r and center P
void show(Pv3D.pt P, float s, Pv3D.vec I, Pv3D.vec J, Pv3D.vec K) {
		noStroke();
		fill(yellow);
		show(P, 5);
		stroke(red);
		show(P, s, I);
		stroke(green);
		show(P, s, J);
		stroke(blue);
		show(P, s, K);
	}

	; // render sphere of radius r and center P
void show(Pv3D.pt P, String s) {
		text(s, P.x, P.y, P.z);
	}

	; // prints string s in 3D at P
void show(Pv3D.pt P, String s, Pv3D.vec D) {
		text(s, P.x + D.x, P.y + D.y, P.z + D.z);
	}

	; // prints string s in 3D at P+D
void showShadow(Pv3D.pt P, float r) {
		pushMatrix();
		translate(P.x, P.y, 0);
		scale(1, 1, 0.01f);
		sphere(r);
		popMatrix();
	}

	String toText(Pv3D.vec V) {
		return "(" + nf(V.x, 1, 5) + "," + nf(V.y, 1, 5) + "," + nf(V.z, 1, 5) + ")";
	}
// ==== curve

	void bezier(Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt D) {
		bezier(A.x, A.y, A.z, B.x, B.y, B.z, C.x, C.y, C.z, D.x, D.y, D.z);
	} // draws a cubic Bezier curve with control points A, B, C, D

	void bezier(Pv3D.pt[] C) {
		bezier(C[0], C[1], C[2], C[3]);
	} // draws a cubic Bezier curve with control points A, B, C, D

	Pv3D.pt bezierPoint(Pv3D.pt[] C, float t) {
		return P(bezierPoint(C[0].x, C[1].x, C[2].x, C[3].x, t), bezierPoint(C[0].y, C[1].y, C[2].y, C[3].y, t), bezierPoint(C[0].z, C[1].z, C[2].z, C[3].z, t));
	}

	Pv3D.vec bezierTangent(Pv3D.pt[] C, float t) {
		return V(bezierTangent(C[0].x, C[1].x, C[2].x, C[3].x, t), bezierTangent(C[0].y, C[1].y, C[2].y, C[3].y, t), bezierTangent(C[0].z, C[1].z, C[2].z, C[3].z, t));
	}

	void PT(Pv3D.pt P0, Pv3D.vec T0, Pv3D.pt P1, Pv3D.vec T1) {
		float d = d(P0, P1) / 3;
		bezier(P0, P(P0, -d, U(T0)), P(P1, -d, U(T1)), P1);
	} // draws cubic Bezier interpolating  (P0,T0) and  (P1,T1) 

	void PTtoBezier(Pv3D.pt P0, Pv3D.vec T0, Pv3D.pt P1, Pv3D.vec T1, Pv3D.pt[] C) {
		float d = d(P0, P1) / 3;
		C[0].set(P0);
		C[1].set(P(P0, -d, U(T0)));
		C[2].set(P(P1, -d, U(T1)));
		C[3].set(P1);
	} // draws cubic Bezier interpolating  (P0,T0) and  (P1,T1) 

	Pv3D.vec vecToCubic(Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt D, Pv3D.pt E) {
		return V((-A.x + 4 * B.x - 6 * C.x + 4 * D.x - E.x) / 6, (-A.y + 4 * B.y - 6 * C.y + 4 * D.y - E.y) / 6, (-A.z + 4 * B.z - 6 * C.z + 4 * D.z - E.z) / 6);
	}

	Pv3D.vec vecToProp(Pv3D.pt B, Pv3D.pt C, Pv3D.pt D) {
		float cb = d(C, B);
		float cd = d(C, D);
		return V(C, P(B, cb / (cb + cd), D));
	}

	;  

// ==== perspective
Pv3D.pt Pers(Pv3D.pt P, float d) {
		return P(d * P.x / (d + P.z), d * P.y / (d + P.z), d * P.z / (d + P.z));
	}

	;
	Pv3D.pt InverserPers(Pv3D.pt P, float d) {
		return P(d * P.x / (d - P.z), d * P.y / (d - P.z), d * P.z / (d - P.z));
	}

	;

// ==== intersection
boolean intersect(Pv3D.pt P, Pv3D.pt Q, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt X) {
		return intersect(P, V(P, Q), A, B, C, X);
	} // if (P,Q) intersects (A,B,C), return true and set X to the intersection point

	boolean intersect(Pv3D.pt E, Pv3D.vec T, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt X) { // if ray from E along T intersects triangle (A,B,C), return true and set X to the intersection point
		Pv3D.vec EA = V(E, A), EB = V(E, B), EC = V(E, C), AB = V(A, B), AC = V(A, C);
		boolean s = cw(EA, EB, EC), sA = cw(T, EB, EC), sB = cw(EA, T, EC), sC = cw(EA, EB, T);
		if ((s == sA) && (s == sB) && (s == sC)) {
			return false;
		}
		float t = m(EA, AC, AB) / m(T, AC, AB);
		X.set(P(E, t, T));
		return true;
	}

	boolean rayIntersectsTriangle(Pv3D.pt E, Pv3D.vec T, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) { // true if ray from E with direction T hits triangle (A,B,C)
		Pv3D.vec EA = V(E, A), EB = V(E, B), EC = V(E, C);
		boolean s = cw(EA, EB, EC), sA = cw(T, EB, EC), sB = cw(EA, T, EC), sC = cw(EA, EB, T);
		return (s == sA) && (s == sB) && (s == sC);
	}

	;
	boolean edgeIntersectsTriangle(Pv3D.pt P, Pv3D.pt Q, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		Pv3D.vec PA = V(P, A), PQ = V(P, Q), PB = V(P, B), PC = V(P, C), QA = V(Q, A), QB = V(Q, B), QC = V(Q, C);
		boolean p = cw(PA, PB, PC), q = cw(QA, QB, QC), a = cw(PQ, PB, PC), b = cw(PA, PQ, PC), c = cw(PQ, PB, PQ);
		return (p != q) && (p == a) && (p == b) && (p == c);
	}

	float rayParameterToIntersection(Pv3D.pt E, Pv3D.vec T, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		Pv3D.vec AE = V(A, E), AB = V(A, B), AC = V(A, C);
		return -m(AE, AC, AB) / m(T, AC, AB);
	}

	float angleDraggedAround(Pv3D.pt G) {  // returns angle in 2D dragged by the mouse around the screen projection of G
		Pv3D.pt S = P(screenX(G.x, G.y, G.z), screenY(G.x, G.y, G.z), 0);
		Pv3D.vec T = V(S, Pmouse());
		Pv3D.vec U = V(S, Mouse());
		return atan2(d(R(U), T), d(U, T));
	}

	float scaleDraggedFrom(Pv3D.pt G) {
		Pv3D.pt S = P(screenX(G.x, G.y, G.z), screenY(G.x, G.y, G.z), 0);
		return d(S, Mouse()) / d(S, Pmouse());
	}

// FANS, CONES, AND ARROWS
	void disk(Pv3D.pt P, Pv3D.vec V, float r) {
		Pv3D.vec I = U(Normal(V));
		Pv3D.vec J = U(N(I, V));
		disk(P, I, J, r);
	}

	void disk(Pv3D.pt P, Pv3D.vec I, Pv3D.vec J, float r) {
		float da = TWO_PI / 36;
		beginShape(TRIANGLE_FAN);
		v(P);
		for (float a = 0; a <= TWO_PI + da; a += da) {
			v(P(P, r * cos(a), I, r * sin(a), J));
		}
		endShape();
	}

	void fan(Pv3D.pt P, Pv3D.vec V, float r) {
		Pv3D.vec I = U(Normal(V));
		Pv3D.vec J = U(N(I, V));
		fan(P, V, I, J, r);
	}

	void fan(Pv3D.pt P, Pv3D.vec V, Pv3D.vec I, Pv3D.vec J, float r) {
		float da = TWO_PI / 36;
		beginShape(TRIANGLE_FAN);
		v(P(P, V));
		for (float a = 0; a <= TWO_PI + da; a += da) {
			v(P(P, r * cos(a), I, r * sin(a), J));
		}
		endShape();
	}

	void collar(Pv3D.pt P, Pv3D.vec V, float r, float rd) {
		Pv3D.vec I = U(Normal(V));
		Pv3D.vec J = U(N(I, V));
		collar(P, V, I, J, r, rd);
	}

	void collar(Pv3D.pt P, Pv3D.vec V, Pv3D.vec I, Pv3D.vec J, float r, float rd) {
		float da = TWO_PI / 36;
		beginShape(QUAD_STRIP);
		for (float a = 0; a <= TWO_PI + da; a += da) {
			v(P(P, r * cos(a), I, r * sin(a), J, 0, V));
			v(P(P, rd * cos(a), I, rd * sin(a), J, 1, V));
		}
		endShape();
	}

	void cone(Pv3D.pt P, Pv3D.vec V, float r) {
		fan(P, V, r);
		disk(P, V, r);
	}

	void stub(Pv3D.pt P, Pv3D.vec V, float r, float rd) {
		collar(P, V, r, rd);
		disk(P, V, r);
		disk(P(P, V), V, rd);
	}

	void arrow(Pv3D.pt P, Pv3D.vec V, float r) {
		stub(P, V(.8f, V), r * 2 / 3, r / 3);
		cone(P(P, V(.8f, V)), V(.2f, V), r);
	}

// **************************** PRIMITIVE
	void showFrame(float angle_x, float angle_y, float angle_z, float d) {
		noStroke();
		fill(metal);
		sphere(d / 10);
		fill(blue);
                pushMatrix();
                rotateZ(angle_z);
		showArrow(d, d / 10);
                popMatrix();
		fill(red);
		pushMatrix();
		rotateY(angle_y);
		showArrow(d, d / 10);
		popMatrix();
		fill(green);
		pushMatrix();
		rotateX(angle_x);
		showArrow(d, d / 10);
		popMatrix();
	}

	void showFan(float d, float r) {
		float da = TWO_PI / 36;
		beginShape(TRIANGLE_FAN);
		vertex(0, 0, d);
		for (float a = 0; a <= TWO_PI + da; a += da) {
			vertex(r * cos(a), r * sin(a), 0);
		}
		endShape();
	}

	void showCollar(float d, float r, float rd) {
		float da = TWO_PI / 36;
		beginShape(QUAD_STRIP);
		for (float a = 0; a <= TWO_PI + da; a += da) {
			vertex(r * cos(a), r * sin(a), 0);
			vertex(rd * cos(a), rd * sin(a), d);
		}
		endShape();
	}

	void showCone(float d, float r) {
		showFan(d, r);
		showFan(0, r);
	}

	void showStub(float d, float r, float rd) {
		showCollar(d, r, rd);
		showFan(0, r);
		pushMatrix();
		translate(0, 0, d);
		showFan(0, rd);
		popMatrix();
	}

	void showArrow() {
		showArrow(1, 0.08f);
	}

	void showArrow(float d, float r) {
		float dd = d / 5;
		showStub(d - dd, r * 2 / 3, r / 3);
		pushMatrix();
		translate(0, 0, d - dd);
		showCone(dd, r);
		popMatrix();
	}

	void showBlock(float w, float d, float h, float x, float y, float z, float a) {
		pushMatrix();
		translate(x, y, h / 2);
		rotateZ(TWO_PI * a);
		box(w, d, h);
		popMatrix();
	}

//*********** PICK
	Pv3D.vec I = V(1, 0, 0), J = V(0, 1, 0), K = V(0, 0, 1); // screen projetions of global model frame

	void computeProjectedVectors() {
		Pv3D.pt O = ToScreen(P(0, 0, 0));
		Pv3D.pt A = ToScreen(P(1, 0, 0));
		Pv3D.pt B = ToScreen(P(0, 1, 0));
		Pv3D.pt C = ToScreen(P(0, 0, 1));
		I = V(O, A);
		J = V(O, B);
		K = V(O, C);
	}

	Pv3D.vec ToIJ(Pv3D.vec V) {
		float x = det2(V, J) / det2(I, J);
		float y = det2(V, I) / det2(J, I);
		return V(x, y, 0);
	}

	Pv3D.vec ToK(Pv3D.vec V) {
		float z = dot(V, K) / dot(K, K);
		return V(0, 0, z);
	}

// ******************************************COLORS 
	public static int black = 0xff000000, white = 0xffFFFFFF, // set more colors using Menu >  Tools > Color Selector
			red = 0xffFF0000, green = 0xff00FF01, blue = 0xff0300FF, yellow = 0xffFEFF00, cyan = 0xff00FDFF, magenta = 0xffFF00FB,
			grey = 0xff818181, orange = 0xffFFA600, brown = 0xffB46005, metal = 0xffB5CCDE, dgreen = 0xff157901,
			blue50 = 0x800300FF;

	void pen(int c, float w) {
		stroke(c);
		strokeWeight(w);
	}

// ******************************** TEXT , TITLE, and USER's GUIDE
	Boolean scribeText = true; // toggle for displaying of help text

	void scribe(String S, float x, float y) {
		fill(0);
		text(S, x, y);
		noFill();
	} // writes on screen at (x,y) with current fill color

	void scribeHeader(String S, int i) {
		fill(0);
		text(S, 10, 20 + i * 20);
		noFill();
	} // writes black at line i

	void scribeHeaderRight(String S) {
		fill(0);
		text(S, width - 7.5f * S.length(), 20);
		noFill();
	} // writes black on screen top, right-aligned

	void scribeFooter(String S, int i) {
		fill(0);
		text(S, 10, height - 10 - i * 20);
		noFill();
	} // writes black on screen at line i from bottom

	void scribeAtMouse(String S) {
		fill(0);
		text(S, mouseX, mouseY);
		noFill();
	} // writes on screen near mouse

	void scribeMouseCoordinates() {
		fill(black);
		text("(" + mouseX + "," + mouseY + ")", mouseX + 7, mouseY + 25);
		noFill();
	}
}
