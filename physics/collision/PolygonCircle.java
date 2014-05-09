package physics.collision;
import tools.Point;
import physics.Body;
import physics.shapes.Circle;
import physics.shapes.Polygon;
/**
 * Provides methods that test intersection of a polygon and a circle, featuring the typical CCD (continous collision detection, meaning that collisions in future time will be recognized and treated especially), returning a depth, normal and position of the collision
 * 
 * @author Tehforsch
 */
public class PolygonCircle {
	static Point intervalP = new Point(), intervalC = new Point();
	static Point v = new Point();
	public static ColInfo collision(Body body1, Body body2, Polygon p, Circle c, float dt) {
		ColInfo col = new ColInfo();
		int nLines = p.numPoints * 2;
		Point[] lines = new Point[nLines];
		for (int i = 0; i < p.numPoints; i++) {
			lines[i] = p.lines[i].normal();
			lines[i + p.numPoints] = new Point(p.points[i].x - c.pos.x, p.points[i].y - c.pos.y);
			lines[i + p.numPoints].sNormalize();
		}
		float minDepth = Float.POSITIVE_INFINITY;
		Point minLine = null;
		for (int i = 0; i < lines.length; i++) {
			intervalP = getInterval(lines[i], p);
			intervalC = getInterval(lines[i], c);
			if (intervalP.y < intervalC.x || intervalC.y < intervalP.x) {
				return null;
			}
			float depth1 = intervalP.y - intervalC.x;
			float depth2 = intervalC.y - intervalP.x;
			float depth = Math.min(Math.abs(depth1), Math.abs(depth2));
			if (depth < minDepth) {
				minDepth = depth;
				minLine = lines[i];
			}
		}
		col.pos = findColPos(p, c, minLine);
		col.depth = minDepth;
		col.time = 0.0f;
		col.normal = minLine;
		// Make sure the normal always points in the right direction
		if (c.pos.distOnLineDir(col.pos, col.normal) < 0) {
			col.normal.sNeg();
		}
		return col;
	}
	public static Point getInterval(Point normal, Polygon p) {
		float projected;
		projected = p.points[0].mul(normal);
		intervalP.x = projected;
		intervalP.y = projected;
		for (int i = 1; i < p.numPoints; i++) {
			projected = p.points[i].mul(normal);
			if (projected < intervalP.x) {
				intervalP.x = projected;
			}
			if (projected > intervalP.y) {
				intervalP.y = projected;
			}
		}
		return intervalP;
	}
	public static Point getInterval(Point normal, Circle c) {
		float h = c.pos.mul(normal);
		intervalC.x = h - c.radius;
		intervalC.y = h + c.radius;
		return intervalC;
	}
	public static Point findColPos(Polygon p, Circle c, Point normal) {
		float minProjected = p.points[0].mul(normal);
		float projected = 0.0f;
		Point colPoint = p.points[0];
		for (int i = 1; i < p.numPoints; i++) {
			projected = p.points[i].mul(normal);
			if (projected < minProjected) {
				minProjected = projected;
				colPoint = p.points[i];
			}
		}
		normal = normal.normal();
		return colPoint.add(normal.mul(c.pos.mul(normal) - colPoint.mul(normal)));
	}
}
