/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import java.util.List;

/**
 * Implements the medial axis transformation between two curves.
 * @author Sebastian Weiss
 */
public class MedialAxisTransform {
	
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
		 * Normally, this array only contains one element, the unique closest projection.
		 * <br>
		 * However, if the two curves are not compatible, the medial axis branches
		 * and there are two or more closest projections. In that case, these times
		 * specify an interval of the curve in that they are not compatible. This
		 * is used in the UI to show the user that he has to change the curve.
		 * <br>
		 * The following must hold:
		 * {@code Curve.interpolate(curveA, projectionOnA[i]).distance(center) == radius} for each i.
		 */
		public float[] projectionOnA;
		/**
		 * The times on the second curve of the closest projection of this point.
		 * <br>
		 * Normally, this array only contains one element, the unique closest projection.
		 * <br>
		 * However, if the two curves are not compatible, the medial axis branches
		 * and there are two or more closest projections. In that case, these times
		 * specify an interval of the curve in that they are not compatible. This
		 * is used in the UI to show the user that he has to change the curve.
		 * <br>
		 * The following must hold:
		 * {@code Curve.interpolate(curveB, projectionOnB[i]).distance(center) == radius} for each i.
		 */
		public float[] projectionOnB;
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
	public static void trace(Vector3f[] curveA, Vector3f[] curveB, List<TracePoint> output) {
		//TODO
	}
}
