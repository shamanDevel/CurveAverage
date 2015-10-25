/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import processing.core.PApplet;
import static curveavg.Pv3D.*;

/**
 *
 * @author Sebastian Weiss
 */
public abstract class AbstractPApplet extends PApplet {
	public static int black = 0xff000000;
	public static int white = 0xffFFFFFF; // set more colors using Menu >  Tools > Color Selector
	public static int red = 0xffFF0000;
	public static int green = 0xff00FF01;
	public static int blue = 0xff0300FF;
	public static int yellow = 0xffFEFF00;
	public static int cyan = 0xff00FDFF;
	public static int magenta = 0xffFF00FB;
	public static int grey = 0xff818181;
	public static int grey80 = 0xcc818181;
	public static int lightgrey = 0xffc8c8c8;
	public static int lightgrey80 = 0xccc8c8c8;
	public static int orange = 0xffFFA600;
	public static int brown = 0xffB46005;
	public static int metal = 0xffB5CCDE;
	public static int dgreen = 0xff157901;
	public static int blue50 = 0x800300FF;
	//*********** PICK
	Pv3D.vec I = V(1, 0, 0);
	Pv3D.vec J = V(0, 1, 0);
	Pv3D.vec K = V(0, 0, 1); // screen projetions of global model frame

	Pv3D.pt ToScreen(Pv3D.pt P) {
		return new Pv3D.pt(screenX(P.x, P.y, P.z), screenY(P.x, P.y, P.z), 0);
	} // O+xI+yJ+kZ

	Pv3D.pt ToModel(Pv3D.pt P) {
		return new Pv3D.pt(modelX(P.x, P.y, P.z), modelY(P.x, P.y, P.z), modelZ(P.x, P.y, P.z));
	} // O+xI+yJ+kZ

	// ===== render
	void normal(Pv3D.vec V) {
		normal(V.x, V.y, V.z);
	}

	// changes normal for smooth shading
	void vertex(Pv3D.pt P) {
		vertex(P.x, P.y, P.z);
	}

	void vertex(Vector3f P) {
		vertex(P.x, P.y, P.z);
	}

	// vertex for shading or drawing
	void v(Pv3D.pt P) {
		vertex(P.x, P.y, P.z);
	}

	// vertex for shading or drawing
	void nv(Pv3D.vec N) {
		normal(N.x, N.y, N.z);
	}

	// vertex for shading or drawing
	void vTextured(Pv3D.pt P, float u, float v) {
		vertex(P.x, P.y, P.z, u, v);
	}

	// draws edge (P,Q)
	void show(Pv3D.pt P, Pv3D.vec V) {
		line(P.x, P.y, P.z, P.x + V.x, P.y + V.y, P.z + V.z);
	}

	// shows edge from P to P+V
	void show(Pv3D.pt P, float d, Pv3D.vec V) {
		line(P.x, P.y, P.z, P.x + d * V.x, P.y + d * V.y, P.z + d * V.z);
	}

	// shows edge from P to P+dV
	void show(Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		beginShape();
		vertex(A);
		vertex(B);
		vertex(C);
		endShape(CLOSE);
	}

	// volume of tet
	void show(Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt D) {
		beginShape();
		vertex(A);
		vertex(B);
		vertex(C);
		vertex(D);
		endShape(CLOSE);
	}

	// volume of tet
	void show(Pv3D.pt P, float r) {
		pushMatrix();
		translate(P.x, P.y, P.z);
		sphere(r);
		popMatrix();
	}

	// render sphere of radius r and center P
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

	// render sphere of radius r and center P
	void show(Pv3D.pt P, String s) {
		text(s, P.x, P.y, P.z);
	}

	// prints string s in 3D at P
	void show(Pv3D.pt P, String s, Pv3D.vec D) {
		text(s, P.x + D.x, P.y + D.y, P.z + D.z);
	}

	// prints string s in 3D at P+D
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

	// ==== perspective
	Pv3D.pt Pers(Pv3D.pt P, float d) {
		return P(d * P.x / (d + P.z), d * P.y / (d + P.z), d * P.z / (d + P.z));
	}

	Pv3D.pt InverserPers(Pv3D.pt P, float d) {
		return P(d * P.x / (d - P.z), d * P.y / (d - P.z), d * P.z / (d - P.z));
	}

	// ==== intersection
	boolean intersect(Pv3D.pt P, Pv3D.pt Q, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt X) {
		return intersect(P, V(P, Q), A, B, C, X);
	} // if (P,Q) intersects (A,B,C), return true and set X to the intersection point

	boolean intersect(Pv3D.pt E, Pv3D.vec T, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C, Pv3D.pt X) {
		// if ray from E along T intersects triangle (A,B,C), return true and set X to the intersection point
		Pv3D.vec EA = V(E, A);
		Pv3D.vec EB = V(E, B);
		Pv3D.vec EC = V(E, C);
		Pv3D.vec AB = V(A, B);
		Pv3D.vec AC = V(A, C);
		boolean s = cw(EA, EB, EC);
		boolean sA = cw(T, EB, EC);
		boolean sB = cw(EA, T, EC);
		boolean sC = cw(EA, EB, T);
		if ((s == sA) && (s == sB) && (s == sC)) {
			return false;
		}
		float t = m(EA, AC, AB) / m(T, AC, AB);
		X.set(P(E, t, T));
		return true;
	}

	boolean rayIntersectsTriangle(Pv3D.pt E, Pv3D.vec T, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		// true if ray from E with direction T hits triangle (A,B,C)
		Pv3D.vec EA = V(E, A);
		Pv3D.vec EB = V(E, B);
		Pv3D.vec EC = V(E, C);
		boolean s = cw(EA, EB, EC);
		boolean sA = cw(T, EB, EC);
		boolean sB = cw(EA, T, EC);
		boolean sC = cw(EA, EB, T);
		return (s == sA) && (s == sB) && (s == sC);
	}

	boolean edgeIntersectsTriangle(Pv3D.pt P, Pv3D.pt Q, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		Pv3D.vec PA = V(P, A);
		Pv3D.vec PQ = V(P, Q);
		Pv3D.vec PB = V(P, B);
		Pv3D.vec PC = V(P, C);
		Pv3D.vec QA = V(Q, A);
		Pv3D.vec QB = V(Q, B);
		Pv3D.vec QC = V(Q, C);
		boolean p = cw(PA, PB, PC);
		boolean q = cw(QA, QB, QC);
		boolean a = cw(PQ, PB, PC);
		boolean b = cw(PA, PQ, PC);
		boolean c = cw(PQ, PB, PQ);
		return (p != q) && (p == a) && (p == b) && (p == c);
	}

	float rayParameterToIntersection(Pv3D.pt E, Pv3D.vec T, Pv3D.pt A, Pv3D.pt B, Pv3D.pt C) {
		Pv3D.vec AE = V(A, E);
		Pv3D.vec AB = V(A, B);
		Pv3D.vec AC = V(A, C);
		return -m(AE, AC, AB) / m(T, AC, AB);
	}

	float angleDraggedAround(Pv3D.pt G) {
		// returns angle in 2D dragged by the mouse around the screen projection of G
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

	void pen(int c, float w) {
		stroke(c);
		strokeWeight(w);
	}

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
                          // vertex with texture coordinates
void show(Pv3D.pt P, Pv3D.pt Q) {
		line(Q.x, Q.y, Q.z, P.x, P.y, P.z);
	}
	
}
