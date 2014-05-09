package physics.sleeping;
import java.io.Serializable;
import java.util.Vector;

import physics.Body;
import physics.World;
/**
 * Handles the subdivision of bodies in collision groups. Two bodies are in the same collision group if they have a connection that means an arbitrary constraint that connects them. Arbiters and joints both count as connections.
 * 
 * The algorithm to determine the collision groups uses body.flag. It iterates through all bodies in the world and for every body that does not have its flag set, the walk routine is called upon the body and a newly created collision group. walk will just set the flag for the body and add the body to the collision group and afterwards iterate through all bodies that have connections to the considered body and call the walk method upon them aswell.
 * 
 * @author toni
 * 
 */
public class CollisionGrouping implements Serializable {
	private World world;
	public Vector<ColGroup> groups;
	public CollisionGrouping(World w) {
		world = w;
		groups = new Vector<ColGroup>();
	}
	public void update() {
		groups.clear();
		Body b;
		// Set all flags to false
		for (int i = 0; i < world.numBodies; i++) {
			b = world.getBody(i);
			b.flag = false;
		}
		for (int i = 0; i < world.numBodies; i++) {
			b = world.getBody(i);
			if (!b.flag) {
				ColGroup group = new ColGroup();
				walk(b, group);
				groups.add(group);
			}
		}
	}
	public void walk(Body b, ColGroup group) {
		if (b.isStatic()) {
			if (group.size() == 0) {
				group.add(b);
			}
			return;
		}
		b.flag = true;
		group.add(b);
		for (Connection c : b.connections) {
			Body other = c.otherThan(b);
			if (!other.flag)
				walk(other, group);
		}
	}
}
