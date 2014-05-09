package physics;
import java.util.Vector;
import physics.shapes.Circle;
import physics.shapes.Polygon;
import tools.Log;
import tools.Point;
/**
 * Calculates wind forces. Takes into account that objects can protect others from wind. Can become quite slow for large object counts.
 * 
 * @author toni
 * 
 */
public class Wind {
	public World world;
	float vel;
	Point dir;
	public Wind(World world, Point windVel) {
		this.world = world;
		if (windVel == null)
			return;
		this.vel = windVel.length();
		this.dir = windVel.sNormalize2();
	}
	public void handle() {
		// dir == null? Wind deactivated.
		if (dir == null)
			return;
		if (world.numBodies > 300) {
			Log.p("Wind is activated. NumBodies is " + world.numBodies + ". Wind calculation can slow the game down");
		}
		Vector<Body> sorted = getSortedBodies();
		applyWind(sorted);
	}
	private void applyWind(Vector<Body> sorted) {
		Vector<Point> intervals = new Vector<Point>();
		Point normal = dir.normal();
		for (Body b : sorted) {
			Point i1 = getInterval(b, normal);
			if (b.isStatic()) {
				intervals.add(i1);
				continue;
			}
			Point i1Copy = new Point(i1);
			for (Point i2 : intervals) {
				i1 = subtractInterval(i1, i2);
			}
			if (i1.x == 0.0f && i1.y == 0.0f) { // Empty intervall, no force and
												// irrelevant for wind
												// calculation
				continue;
			}
			intervals.add(i1);
			Point newForceCenter;
			if (i1Copy.x != i1.x || i1Copy.y != i1.y) {
				float centerDiff = -((i1.y - i1.x) - (i1Copy.y - i1Copy.x)) * 0.5f;
				centerDiff *= 1f;
				newForceCenter = b.pos.add(normal.mul(centerDiff));
			}
			else {
				newForceCenter = b.pos;
			}
			Point force = dir.mul(vel * getArea(i1));
			b.applyConstForce(force, newForceCenter);
		}
	}
	private Point subtractInterval(Point i1, Point i2) {
		// !!!
		// Does not cover the case where the interval is split in half
		// x2<-------->y2
		// x1<---------------->y1
		// <---> <--->
		// !!!
		// I2 completely covers I1
		// x2<-------------------->y2
		// x1<------->y1
		if (i1.y <= i2.y && i1.x >= i2.x) {
			i1.x = 0;
			i1.y = 0;
			return i1;
		}
		// I1 is completely to the left of I2
		// x2<--->y2
		// x1<--->y1
		else if (i1.y <= i2.x) {
			return i1;
		}
		// I1 is completely to the right of I2
		// x2<--->y2
		// x1<---->y1
		else if (i1.x >= i2.y) {
			return i1;
		}
		// I1 has its right side covered by I2
		// x2<---->y2
		// x1<---->y1
		else if (i1.x <= i2.x && i1.y >= i2.x) {
			i1.y = i2.x;
			return i1;
		}
		// I1 has its left t side covered by I2
		// x2<---->y2
		// x1<---->y1
		else if (i1.x <= i2.y && i1.y >= i2.y) {
			i1.x = i2.y;
			return i1;
		}
		return null;
	}
	private float getArea(Point interval) {
		return interval.y - interval.x;
	}
	private Point getInterval(Body b, Point normal) {
		if (b.shape instanceof Polygon) {
			Polygon poly = ((Polygon) b.shape);
			float maxP = Float.NEGATIVE_INFINITY, minP = Float.POSITIVE_INFINITY;
			for (Point p : poly.points) {
				float k = p.mul(normal);
				if (k > maxP)
					maxP = k;
				if (k < minP)
					minP = k;
			}
			return new Point(minP, maxP);
		}
		else if (b.shape instanceof Circle) {
			Circle circle = (Circle) b.shape;
			float k = circle.pos.mul(normal);
			return new Point(k - circle.radius, k + circle.radius);
		}
		assert (false);
		return new Point();
	}
	private Vector<Body> getSortedBodies() {
		Vector<Body> unsorted = new Vector<Body>();
		for (int i = 0; i < world.numBodies; i++) {
			unsorted.add(world.getBody(i));
		}
		Vector<Body> sorted = new Vector<Body>();
		for (int i = 0; i < world.numBodies; i++) {
			float minProjected = Float.POSITIVE_INFINITY;
			Body min = null;
			for (Body b2 : unsorted) {
				float projected = b2.pos.mul(dir);
				if (projected < minProjected) {
					min = b2;
					minProjected = projected;
				}
			}
			sorted.add(min);
			unsorted.remove(min);
		}
		return sorted;
	}
}
