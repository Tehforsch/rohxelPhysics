package physics.sleeping;
import java.io.Serializable;
import java.util.Vector;

import physics.Body;
import physics.World;
import physics.collision.Pair;
import physics.joints.Joint;
/**
 * Handles the sleeping system. Bodies begin to sleep whenever the whole collision group they are connected to is in rest for a period of time. Bodies are in rest whenever their velocity and angular velocity as well as position correction impulses (bias vel) are below a certain threshold. Collision groups are determined by CollisionGrouping.java. Collisions group will only be set to sleep if every body within them is in rest.
 * 
 * When two bodies connected by a contact are asleep (The case where only one body is asleep will not occur since collision groups are treated as a whole) their contact-arbiter will not need to perform calculations. It still needs to be saved so that warmstarting impulses are kept alive until the bodies are awakened. When bodies are put asleep its important to not set the velocities to zero but instead not integrate positions/velocities for those bodies. This is because the non-zero velocities that the bodies had before they were asleep solved the contact system (for the corresponding warmstarting impulses) better than exact zero velocities. If velocities are set to zero, the system will fall asleep but as soon as it is awakened it will be in an unstable state.
 * 
 * Whenever a non-constant force/impulse is applied to a body the body will be awakened. Every force that is meant to be game-dependent like explosions etc. must be non-constant whereas most physical forces like gravity and air friction are constant since they are applied all the time.
 * 
 * 
 * @author toni
 * 
 */
public class SleepingSystem implements Serializable {
	private static final float SLEEPTIME = 0.5f;
	private static final float SLEEPVEL = 0.05f;
	private static final float SLEEPAVEL = 0.005f;
	private World world;
	private CollisionGrouping colGrouping;
	public SleepingSystem(World w) {
		world = w;
		colGrouping = new CollisionGrouping(world);
	}
	public void handle(float dt) {
		colGrouping.update();
		for (ColGroup c : colGrouping.groups) {
			handleGroup(c, dt);
		}
	}
	private void handleGroup(ColGroup c, float dt) {
		boolean isSleeping = true;
		for (Body b : c) {
			// Body is idle? Update the timer
			if (isIdle(b)) {
				b.idleTime += dt;
			}
			// Otherwise reset it
			else {
				b.idleTime = 0;
			}
			// If there is one body that isn't sleeping the whole collision
			// group isn't sleeping aswell.
			if (b.idleTime < SLEEPTIME) {
				isSleeping = false;
			}
		}
		for (Body b : c) {
			// Body was awake but the collision group is now sleeping -> Put
			// body asleep.
			if (isSleeping && !b.sleeps) {
				b.sleep();
			}
			// Body was asleep but the collision group is now awake -> Put body
			// awake.
			if (!isSleeping && b.sleeps) {
				b.wakeUp();
			}
		}
	}
	private boolean isIdle(Body b) {
		return b.vel.square() < SLEEPVEL * SLEEPVEL && b.avel * b.avel < SLEEPAVEL * SLEEPAVEL;
	}
	public void handleRemovedBody(Body body, Vector<Pair> pairsWithThisBody) {
		body.wakeUp(); // Hopefully not needed. I don't care.
		for (Connection c : body.connections) {
			c.otherThan(body).wakeUp();
		}
		// The collision detection will ignore two sleeping bodies and not add
		// them to arbiters. The result is
		// that sometimes bodies won't wake up because there's no arbiter
		// between them, although they are in contact.
		// This can be resolved by waking up every possible collision pair. Not
		// very beautiful but simple and shouldn't
		// make the system too sloppy
		for (Pair p : pairsWithThisBody) {
			p.otherThan(body).wakeUp();
		}
	}
	public void handleAddedJoint(Joint joint) {
		joint.wakeUp();
	}
	public void handleRemovedJoint(Joint joint) {
		joint.wakeUp();
	}
}
