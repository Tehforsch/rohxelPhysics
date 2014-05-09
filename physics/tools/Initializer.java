package physics.tools;
import java.util.Vector;

import physics.Body;
import physics.World;
import physics.joints.JDistanceJoint;
import physics.shapes.Circle;
import physics.shapes.Polygon;
import physics.shapes.Shape;

import tools.Floatmath;
import tools.Point;
public class Initializer {
	public static void addLineOfRectanglesX(World w, Point start, int number, float width, float height, float mass, float distance) {
		for (int i = 0; i < number; i++) {
			Point pos = start.add(new Point(i * distance, 0));
			Point[] points = new Point[]{new Point(0, 0), new Point(width, 0), new Point(width, height), new Point(0, height)};
			w.addBody(new Body(new Polygon(pos, points), mass));
		}
	}
	public static void addLineOfRectanglesY(World w, Point start, int number, float width, float height, float mass, float distance) {
		for (int i = 0; i < number; i++) {
			Point pos = start.add(new Point(0, i * distance));
			Point[] points = new Point[]{new Point(0, 0), new Point(width, 0), new Point(width, height), new Point(0, height)};
			w.addBody(new Body(new Polygon(pos, points), mass));
		}
	}
	public static Vector<Body> getLineOfRectanglesX(Point start, int number, float width, float height, float mass, float distance) {
		Vector<Body> bodies = new Vector<Body>();
		for (int i = 0; i < number; i++) {
			Point pos = start.add(new Point(i * distance, 0));
			Point[] points = new Point[]{new Point(0, 0), new Point(width, 0), new Point(width, height), new Point(0, height)};
			bodies.add(new Body(new Polygon(pos, points), mass));
		}
		return bodies;
	}
	public static Vector<Body> getLineOfRectanglesY(Point start, int number, float width, float height, float mass, float distance) {
		Vector<Body> bodies = new Vector<Body>();
		for (int i = 0; i < number; i++) {
			Point pos = start.add(new Point(0, i * distance));
			Point[] points = new Point[]{new Point(0, 0), new Point(width, 0), new Point(width, height), new Point(0, height)};
			bodies.add(new Body(new Polygon(pos, points), mass));
		}
		return bodies;
	}
	public static Body getRectangle(Point pos, float a, float b, float mass, float alpha) {
		Point[] points = new Point[]{new Point(0, 0), new Point(a, 0), new Point(a, b), new Point(0, b)};
		for (int i = 0; i < points.length; i++) {
			points[i] = points[i].rotate(alpha);
		}
		return new Body(new Polygon(pos, points), mass);
	}
	public static Body getRectangle(Point pos, float a, float b, float mass) {
		return getRectangle(pos, a, b, mass, 0.0f);
	}
	public static JDistanceJoint dJoint(Body b1, Body b2) {
		return new JDistanceJoint(b1, b2, b1.pos, b2.pos);
	}
	public static Body getCircle(Point p, float r, float mass) {
		return new Body(new Circle(p, r), mass);
	}
	/**
	 * Not extremely valuable in terms of physical accuracy but delivers plausible results for the simulation. m = m0 * sqrt(A)
	 * 
	 * @author toni
	 * 
	 * @param shape
	 * @param mass
	 * @return
	 */
	public static float calcMass(Shape shape, float mass) {
		return mass * Floatmath.sqrt(shape.calcArea());
	}
}
