package physics;
import java.io.Serializable;

import tools.Point;
/**
 * Handles gravity. Will get added options for directional gravity ( not just downwards )
 * 
 * @author toni
 * 
 */
public class Gravity implements Serializable {
	private World world;
	public Gravity(World w) {
		world = w;
	}
	public void handle() {
		Body b;
		for (int i = 0; i < world.numBodies; i++) {
			b = world.getBody(i);
			if (b.isStatic())
				continue;
			Point force = new Point(0, PhysicalConstants.GRAVITY * PhysicalConstants.UNITSPERMETER * b.mass);
			b.applyConstForce(force);
		}
	}
}
