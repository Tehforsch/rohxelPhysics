package physics.shapes;
import tools.Floatmath;
import tools.Point;
public class Circle extends Shape {
	public Circle(Point pos, float radius) {
		this.pos = pos;
		this.radius = radius;
	}
	@Override
	public float calcArea() {
		return Floatmath.PI * radius * radius;
	}
	@Override
	public float calcInertia(float mass) {
		return 0.2f * mass * radius * radius;
	}
	@Override
	public float getHeight() {
		return radius;
	}
	@Override
	public float getWidth() {
		return radius;
	}
	@Override
	public void update() {
	}
}
