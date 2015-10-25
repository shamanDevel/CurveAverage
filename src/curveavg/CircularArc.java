/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

/**
 * Small helper class for computing the circular arc between two points.
 * @author Sebastian Weiss
 */
public class CircularArc {
	private static final float EPSILON = 1e-5f;
	
	private final Vector3f P;
	private final Vector3f A;
	private final Vector3f B;
	private Vector3f C;
	private float r;
	private Vector3f N;
	private Vector3f CA;
	private float angle;

	/**
	 * Computes the circular arc between the points A and B, with the point P on the
	 * medial axis (i.e. |A-N|==|B-N|).
	 * @param P the point on the medial axis, A and B are at the same distance from P
	 * @param A the closest point from P on the first control curve
	 * @param B the closest point from P on the second control curve
	 */
	public CircularArc(Vector3f P, Vector3f A, Vector3f B) {
		this.P = P;
		this.A = A;
		this.B = B;
		computeCenter();
	}
	
	private void computeCenter() {
		Vector3f PA = A.subtract(P);
		Vector3f PB = B.subtract(P);
		N = PA.cross(PB);
		//check for degenerated case
		if (N.lengthSquared() <= EPSILON*EPSILON) {
//			System.out.println("A="+A+", B="+B+" P="+P+" -> it is a line");
			r = 0;
			return; //Degenerated to a line
		}
		//compute directions of A,B to the center C
		N.normalizeLocal();
		PA.normalizeLocal();
		PB.normalizeLocal();
		Vector3f U = N.cross(PA);
		Vector3f V = N.cross(PB);
		//C is now the intersection of A+aU and B+bV
		Vector3f UxV = U.cross(V);
		Vector3f rhs = (B.subtract(A)).cross(V);
		float a1 = rhs.x / UxV.x;
		float a2 = rhs.y / UxV.y;
		float a3 = rhs.z / UxV.z; //all a1,a2,a3 have to be equal
		C = A.addScaled(a1, U);
		//compute radius and angle
		r = C.distance(A);
		CA = A.subtract(C);
		angle = (CA.normalize()).angleBetween((B.subtract(C).normalizeLocal()));
	}
	
	private float computeCenter_old() {
		//check for degenerated case
		Vector3f PA = A.subtract(P);
		Vector3f PB = B.subtract(P);
		N = PA.cross(PB);
		if (N.lengthSquared() <= EPSILON*EPSILON) {
//			System.out.println("A="+A+", B="+B+" P="+P+" -> it is a line");
			return 0; //Degenerated to a line
		}
		
		//define orthonormal basis I,J,K
		N.normalizeLocal();
		Vector3f I = PA.normalize();
		Vector3f J = N.cross(I);
		Vector3f K = N;
		Vector3f IxK = I.cross(K);
		Vector3f JxK = J.cross(K);
		//project points in the plane PAB
		Vector3f PAxN = PA.cross(N);
		Vector3f PBxN = PB.cross(N);
		Vector2f P2 = new Vector2f(0, 0);
		Vector2f A2 = new Vector2f(PA.dot(JxK)/I.dot(JxK), PA.dot(IxK)/J.dot(IxK));
		Vector2f B2 = new Vector2f(PB.dot(JxK)/I.dot(JxK), PB.dot(IxK)/J.dot(IxK));
		
		//compute time t of C=P+tPA°
		float t;
		if (B2.x == A2.x) {
			t = (B2.y - A2.y) / (A2.x - B2.x);
		} else {
			t = (B2.x - A2.x) / (A2.y - B2.y);
		}
		//compute C
		Vector2f PArot = new Vector2f(A.y-P.y, P.x-A.x);
		Vector2f C2 = P2.addLocal(PArot.multLocal(t));
		//project back
		C = new Vector3f(P);
		C.addScaledLocal(C2.x, I);
		C.addScaledLocal(C2.y, J);
		//Debug
//		System.out.println("A="+A+", B="+B+" P="+P+" -> I="+I+", J="+J+", K="+K+", P'="+P2+", A'="+A2+", B'="+B2+", t="+t+", C'="+C2+", C="+C);
		//set radius
		return C.distance(A);
	}
	
	/**
	 * Computes a point on the actual circular arc from A to B.
	 * The vectors C and the float r are the result from
	 * {@link #computeCenter(curveavg.Vector3f, curveavg.Vector3f, curveavg.Vector3f, curveavg.Vector3f) }.
	 * It interpolates from A (alpha=0) to B (alpha=1) in a circular arc.
	 * @param alpha
	 * @return 
	 */
	public Vector3f getPointOnArc(float alpha) {
		if (isDegenerated()) {
			Vector3f V = new Vector3f();
			return V.interpolateLocal(A, B, alpha); //degenerated case
		}
		//normal case, rotate
		Vector3f V = rotate(alpha*angle, CA, N);
		return V.addLocal(C);
	}
	
	private static Vector3f rotate(float angle, Vector3f X, Vector3f N) {
		Vector3f W = N.mult(N.dot(X));
		Vector3f U = X.subtract(W);
		return W.add(U.mult((float) Math.cos(angle)))
				.subtract(N.cross(U).mult((float) Math.sin(angle)));
	}

	public boolean isDegenerated() {
		return r==0;
	}
	
	public Vector3f getCenter() {
		return C;
	}

	public float getRadius() {
		return r;
	}

	public Vector3f getNormal() {
		return N;
	}

	public float getAngle() {
		return angle;
	}

	@Override
	public String toString() {
		return "CircularArc{" + "P=" + P + ", A=" + A + ", B=" + B + " -> C=" + C + ", r=" + r + ", N=" + N + ", angle=" + angle + '}';
	}
	
}
