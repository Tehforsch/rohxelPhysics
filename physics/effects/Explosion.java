package physics.effects;
import physics.Body;
import tools.Point;
public class Explosion extends Effect {
	private Point p;
	private float radius;
	private float strength;
	public Explosion(Point p, float rad, float strength) {
		this.p = p;
		this.radius = rad;
		this.strength = strength;
	}
	@Override
	public void apply(Body b) {
		float d = p.distance(b.pos);
		if (d < radius && d > 0) {
			Point i = b.pos.sub(p).mul(strength / (0.1f * d));
			b.applyImpulse(i);
		}
	}
}
