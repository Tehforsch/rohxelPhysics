package physics.collision;
import physics.Body;
import physics.PhysicalConstants;
import physics.shapes.Polygon;
import tools.Point;
/**
 * @author Tehforsch Provides methods that test intersection of two polygons, taking into account speeds and performing continuous collision detection.
 */
public class PolygonPolygon {
	static Point v = new Point(), interval1 = new Point(), interval2 = new Point(), line = new Point();
	public static ColInfo collision(Body body1, Body body2, Polygon polygon1, Polygon polygon2, float dt) {
		// BOUNDING CIRCLE TEST
		if (Math.pow(polygon1.radius + polygon2.radius, 2) + body1.vel.squaredDistance(body2.vel) < polygon1.pos.squaredDistance(polygon2.pos)) {
			return null;
		}
		// TODO was gutes hier ausdenken
		float depthDifference = Math.min(polygon1.radius, polygon2.radius) * 0.003f;
		ColInfo col = new ColInfo();
		int nLines = polygon1.numPoints + polygon2.numPoints + 1;
		Point[] lines = new Point[nLines];
		for (int i = 0; i < polygon1.numPoints; i++) {
			lines[i] = polygon1.lines[i].normal();
		}
		for (int i = 0; i < polygon2.numPoints; i++) {
			lines[i + polygon1.numPoints] = polygon2.lines[i].normal();
		}
		v.from2(body2.vel).sSub(body1.vel);
		lines[nLines - 1] = new Point(v.x, v.y);
		lines[nLines - 1].sNormal2().sNormalize();
		// col.lines = lines;
		float[] tcolls = new float[nLines];
		float mindepth = Float.POSITIVE_INFINITY;
		int mini = -1;
		float depth1, depth2;
		for (int i = 0; i < nLines; i++) {
			interval1 = setInterval(interval1, lines[i], polygon1);
			interval2 = setInterval(interval2, lines[i], polygon2);
			depth1 = interval1.x - interval2.y;
			depth2 = interval2.x - interval1.y;
			if (depth1 > 0 || depth2 > 0) { // Spaeter Ueberlappung
				float vn = lines[i].mul(v);
				if (Math.abs(vn) < 0.0001) {
					return null;
				}
				float t1 = depth1 / vn;
				float t2 = -depth2 / vn;
				if (t1 > t2) {
					float temp = t1;
					t1 = t2;
					t2 = temp;
				}
				float tcoll = (t1 > 0.0) ? t1 : t2;
				if (tcoll > dt || tcoll < 0.0) {
					return null;
				}
				tcolls[i] = tcoll;
			}
			else {
				tcolls[i] = 0.0f;
				depth1 = Math.min(Math.abs(depth1), Math.abs(depth2));
				if (depth1 < mindepth) {
					mindepth = depth1;
					mini = i;
				}
			}
		}
		for (int i = 0; i < nLines; i++) {
			if (tcolls[i] != 0.0) {
				if (tcolls[i] > col.time || col.time == dt) {
					col.time = tcolls[i];
					mini = i;
				}
			}
		}
		if (mini == -1) {
			return null;
		}
		col.normal = new Point(lines[mini].x, lines[mini].y);
		if (polygon1.pos.distOnLineDir(polygon2.pos, col.normal) > 0) {
			col.normal.sNeg();
		}
		findColPos(polygon1, polygon2, col.normal, col, depthDifference);
		col.depth = mindepth;
		if (col.time != dt && col.time != 0.0f) {
			col.depth = PhysicalConstants.ALLOWEDPENETRATION;
		}
		return col;
	}
	private static Point setInterval(Point inter, Point normal, Polygon p) {
		float projected;
		projected = p.points[0].mul(normal);
		inter.x = projected;
		inter.y = projected;
		for (int i = 1; i < p.numPoints; i++) {
			projected = p.points[i].mul(normal);
			if (projected < inter.x) {
				inter.x = projected;
			}
			if (projected > inter.y) {
				inter.y = projected;
			}
		}
		return inter;
	}
	static private Point[] findManifoldPoints(Point normal, Point[] points1, Point[] points2) {
		line = normal.normal();
		interval1.x = points1[0].mul(line);
		interval1.y = points1[1].mul(line);
		interval2.x = points2[0].mul(line);
		interval2.y = points2[1].mul(line);
		boolean swap1 = false, swap2 = false;
		if (interval1.x > interval1.y) {
			interval1 = swap(interval1);
			swap1 = true;
		}
		if (interval2.x > interval2.y) {
			interval2 = swap(interval2);
			swap2 = true;
		}
		Point pcoll1, pcoll2;
		if (interval1.x < interval2.x) {
			if (!swap2) {
				pcoll1 = points2[0];
			}
			else {
				pcoll1 = points2[1];
			}
		}
		else {
			if (!swap1) {
				pcoll1 = points1[0];
			}
			else {
				pcoll1 = points1[1];
			}
		}
		if (interval1.y > interval2.y) {
			if (!swap2) {
				pcoll2 = points2[1];
			}
			else {
				pcoll2 = points2[0];
			}
		}
		else {
			if (!swap1) {
				pcoll2 = points1[1];
			}
			else {
				pcoll2 = points1[0];
			}
		}
		return new Point[]{pcoll1, pcoll2};
	}
	private static Point swap(Point p) {
		float tmp = p.x;
		p.x = p.y;
		p.y = tmp;
		return p;
	}
	private static void findColPos(Polygon p1, Polygon p2, Point normal, ColInfo col, float depthDifference) {
		Point[] minpoints1 = getMinPoints(p1, p2, normal, depthDifference);
		Point[] minpoints2 = getMinPoints(p2, p1, normal, depthDifference);
		if (minpoints1[1] == null && minpoints2[1] == null) { // Punkt vs
			// Punkt
			col.pos = new Point((minpoints1[0].x + minpoints2[0].x) * 0.5f, (minpoints1[0].y + minpoints2[0].y) * 0.5f);
			assert (p1 != p2);
			return;
		}
		else if (minpoints1[1] != null && minpoints2[1] != null) { // Kante vs
			// Kante
			Point[] colPoints = findManifoldPoints(normal, minpoints1, minpoints2);
			col.pos = colPoints[0];
			col.pos2 = colPoints[1];
			return;
		}
		Point linepoint1, linepoint2, p;
		if (minpoints1[1] == null) {
			linepoint1 = minpoints2[0];
			linepoint2 = minpoints2[1];
			p = minpoints1[0];
		}
		else {
			linepoint1 = minpoints1[0];
			linepoint2 = minpoints1[1];
			p = minpoints2[0];
		}
		normal = new Point(linepoint2.x, linepoint2.y);
		normal.sSub2(linepoint1).sNormalize();
		// normal.from2(linepoint2).sSub2(linepoint1).sNormalize();
		// normal = linepoint2.sub(linepoint1).normalize();
		float projected1 = linepoint1.mul(normal);
		float projected2 = p.mul(normal);
		// col.pos = linepoint1.add(normal.mul(projected2 - projected1));
		col.pos = new Point(linepoint1.x + normal.x * (projected2 - projected1), linepoint1.y + normal.y * (projected2 - projected1));
	}
	private static Point[] getMinPoints(Polygon p1, Polygon p2, Point normal, float depthDifference) {
		Point[] minpoints;
		minpoints = new Point[2];
		float mindepth = Float.POSITIVE_INFINITY;
		float subdepth = p2.pos.mul(normal);
		float depth;
		for (int i = 0; i < p1.numPoints; i++) {
			depth = Math.abs(p1.points[i].mul(normal) - subdepth);
			if (Math.abs(depth - mindepth) < depthDifference) {
				minpoints[1] = p1.points[i];
			}
			else if (depth < mindepth) {
				minpoints[0] = p1.points[i];
				minpoints[1] = null;
				mindepth = depth;
			}
		}
		return minpoints;
	}
}