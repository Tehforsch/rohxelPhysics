package physics.shapes;
import java.io.Serializable;

import physics.Body;

import tools.Point;
/**
 * Represents an abstract physical shape (in particular : polygons, circles, maybe infinite planes or convex polygons for later). Each subclass must provide a method to calculate the area and the moment of inertia of the shape.
 * 
 * @author toni
 * 
 */
public abstract class Shape implements Serializable {
	public Point pos;
	public Body parent;
	public float radius;
	public abstract float calcArea();
	public abstract float calcInertia(float mass);
	public abstract void update();
	public abstract float getWidth();
	public abstract float getHeight();
}
