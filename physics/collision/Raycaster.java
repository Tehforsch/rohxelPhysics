package physics.collision;
import java.util.Vector;

import physics.Body;
import physics.World;
import physics.shapes.Circle;
import physics.shapes.Polygon;
import tools.Floatmath;
import tools.Point;
/**
 * Holds methods for casting rays against PShapes. Returns info about the intersecting points.
 * 
 * @author Tehforsch
 */
public class Raycaster {
	/**
	 * Casts a SRay against a SCircle.
	 * 
	 * @param ray
	 *            : The SRay that should be casted against the SCircle.
	 * @param circle
	 *            : The SCircle that the ray should be casted against.
	 */
	public static RayCastInfo rayCircle(Ray ray, Circle circle) {
		RayCastInfo col = new RayCastInfo();
		float af, bf, cf;
		af = 1;
		bf = 2 * (ray.pos.sub(circle.pos)).mul(ray.dir);
		cf = ray.pos.sub(circle.pos).square() - circle.radius * circle.radius;
		float disc = bf * bf - 4 * af * cf;
		if (disc < 0) {
			return null;
		}
		if (disc == 0) {
			return null;
		}
		float t1, t2;
		float sq = Floatmath.sqrt(disc);
		float divisor = 1.0f / (2 * af);
		t1 = (-bf + sq) * divisor;
		t2 = (-bf - sq) * divisor;
		// assert(t1 > t2);
		col.pos = ray.pos.add(ray.dir.mul(t1));
		col.pos2 = ray.pos.add(ray.dir.mul(t2));
		// t²+ 2t(P-C)·D + |P-C|² - R² = 0
		return col;
	}
	/**
	 * Casts a SRay against a SPolygon.
	 * 
	 * @param ray
	 *            : The SRay that should be casted against the SPolygon.
	 * @param polygon
	 *            : The SPolygon that the ray should be casted against.
	 * @return ColInfo about
	 */
	public static RayCastInfo rayPolygon(Ray ray, Polygon polygon) {
		RayCastInfo col = new RayCastInfo();
		Point pos;
		for (int i = 0, j = polygon.numPoints - 1; i < polygon.numPoints; i++, j = i - 1) {
			pos = rayLine(ray, polygon.points[i], polygon.points[j]);
			if (pos != null) {
				if (col.pos != null) {
					col.pos2 = pos;
				}
				else {
					col.pos = pos;
				}
			}
		}
		if (col.pos2 == null) {
		}
		else if (col.pos.squaredDistance(ray.pos) > col.pos2.squaredDistance(ray.pos)) {
			Point tmp = col.pos;
			col.pos = col.pos2;
			col.pos2 = tmp;
		}
		return col;
	}
	/**
	 * Casts a SRay against a Line given through two points.
	 * 
	 * @param ray
	 *            : The SRay that has to be casted against the Line.
	 * @param point1
	 *            : A point on the line.
	 * @param point2
	 *            : A second point on the line.
	 * @return ColInfo about
	 */
	public static Point rayLine(Ray ray, Point point1, Point point2) {
		float x1, y1, x2, y2;
		x1 = point1.x;
		x2 = point2.x;
		y1 = point1.y;
		y2 = point2.y;
		float x3, y3, x4, y4;
		x3 = ray.pos.x;
		x4 = ray.pos.x + ray.dir.x;
		y3 = ray.pos.y;
		y4 = ray.pos.y + ray.dir.y;
		float denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
		if (denom == 0) {
			return null;
		}
		denom = 1 / denom;
		float t1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) * denom;
		if (t1 > 0.0f && t1 < 1.0f)
			return point1.mul(1 - t1).add(point2.mul(t1));
		return null;
	}
	public static Vector<RayCastInfo> getAllIntersecting(World w, Point pos, Point line, float MINIMUMDIST) {
		Vector<RayCastInfo> cols = new Vector<RayCastInfo>();
		RayCastInfo k = null;
		Ray r = new Ray(pos, line);
		for (int i = 0; i < w.numBodies; i++) {
			Body b = w.getBody(i);
			if (b.shape instanceof Polygon) {
				k = rayPolygon(r, (Polygon) b.shape);
			}
			else if (b.shape instanceof Circle) {
				k = rayCircle(r, (Circle) b.shape);
			}
			if (k == null)
				continue;
			if (k.pos == null)
				continue;
			if (k.pos2 != null) { // Sort, pos should be the nearest, pos2 the one further away
				float d1 = k.pos.sub(pos).mul(line);
				float d2 = k.pos2.sub(pos).mul(line);
				if (d1 > d2) { // pos further away than pos2? -> Swap
					Point tmp = k.pos;
					k.pos = k.pos2;
					k.pos2 = tmp;
				}
			}
			float dist = k.pos.squaredDistance(r.pos);
			if (dist > MINIMUMDIST * MINIMUMDIST && k.pos.sub(pos).mul(r.dir) > 0) {
				k.body = b;
				cols.add(k);
			}
		}
		return cols;
	}
	public static RayCastInfo getFirstIntersecting(World w, Point pos, Point line, float MINIMUMDIST) {
		float mindist = Float.POSITIVE_INFINITY;
		RayCastInfo mincol = null;
		for (RayCastInfo c : getAllIntersecting(w, pos, line, MINIMUMDIST)) {
			float dist = c.pos.sub(pos).mul(line);
			if (c.pos2 != null) {
				float dist2 = c.pos2.sub(pos).mul(line);
				if (Math.signum(dist) != Math.signum(dist2))
					continue; // Point is in the body
			}
			if (dist < mindist && dist > MINIMUMDIST && c.pos.sub(pos).mul(line) > 0) {
				mincol = c;
				mindist = dist;
			}
		}
		return mincol;
	}
}