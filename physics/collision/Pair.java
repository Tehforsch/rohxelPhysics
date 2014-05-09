package physics.collision;
import java.io.Serializable;

import physics.Body;
import tools.Point;
public class Pair implements Serializable {
	public Body b1, b2;
	public Point pos, pos2, normal;
	public float depth;
	public Arbiter arbiter;
	public Pair(Body b1, Body b2) {
		super();
		this.b1 = b1;
		this.b2 = b2;
	}
	public void update(ColInfo c) {
		pos = c.pos;
		pos2 = c.pos2;
		normal = c.normal;
		depth = c.depth;
		if (arbiter != null) {
			arbiter.update(this);
		}
	}
	// returns the body in the pair that is not the given body
	public Body otherThan(Body b) {
		if (b1 == b)
			return b2;
		if (b2 == b)
			return b1;
		return null;
	}
	@Override
	public String toString() {
		return "(" + String.valueOf(b1.id) + "," + String.valueOf(b2.id) + ")";
	}
}
