package physics;
import java.io.Serializable;
import java.util.Vector;

import physics.collision.Arbiter;
import physics.collision.CollisionHandler;
import physics.collision.Pair;
import physics.effects.Effect;
import physics.joints.JDistanceJoint;
import physics.joints.Joint;
import physics.joints.TwoBodyJoint;
import physics.sleeping.Connection;
import physics.sleeping.SleepingSystem;
import tools.Point;
import tools.Statistics;
import tools.id.IdSystem;
/**
 * Represents the physical world. Holds a list of bodies and joints that connect those bodies. Each call of handle() proceeds simulation time by a fixed deltaT.
 * 
 * Order of calculations:
 * 
 * - Apply external forces (gravity, air friction, ... ) and directly calculate resulting accelerations - Integrate velocities ( v' = v + a * dt ) - Check for intersections of shapes - - Broad phase : sweep and prune algorithm - - Mid phase : dynamic bounding boxes are checked for intersection (not really a performance boost for circles/rectangles, but the more complex the shapes become the more gain is to be expected) - - Narrow phase : Circle <-> Polygon and Polygon <-> Polygon collisions are non-trivial. The time of future collisions is also calculated, resulting in more precise simulation. This is quite difficult for Circle <-> Polygon checks - - Optional : Divide bodies in collision groups for sleeping system calculations. (Important for large simulations) - Solve constraints (including contacts/collisions) simultaneously - - Linear solver (solve each constraint in some given order for a number of iterations => system is approximately solved) - Integrate positions ( x' = x + v * dt )
 * 
 * @author toni
 */
public class World implements Serializable {
	// Number of iterations done in the constraint solver. This should be around
	// 10.
	// Lower numbers speed the calculations up but make them less accurate,
	// higher numbers increase accuracy and stability but will lead to slower
	// calculations.
	private static final int SOLVEITERATIONS = 25;
	// The position-correction factor. Should be around [0.05, 0.2].
	public static final float BIAS = 0.1f;
	// The distance that two bodies can penetrate each other without the contact
	// solver pushing them apart.
	public static final float INVDT = 1.0f / PhysicalConstants.DT;
	public static final float ALLOWEDPENETRATION = 0.09f;
	// A little margin that is added to the bodies in the broadphase so they are
	// not cut up exactly where they end.
	// Not entirely sure if this is actually needed but it doesn't slow down
	// calculations very much and will probably prevent some occuring bugs
	// later.
	public static final float BOUNDINGMARGIN = 0.1f;
	// The two fundamental types of physical objects
	private IdSystem<Body> bodies;
	private IdSystem<Joint> joints;
	public int numBodies;
	public int numJoints;
	private transient CollisionHandler collisions;
	private transient SleepingSystem sleepingSystem;
	private transient Gravity gravity;
	private transient AirFriction airfriction;
	private transient Wind wind;
	// A list of arbiters, that is contacts between to bodies that need to be
	// solved.
	private Vector<Arbiter> arbiters;
	// The time that has passed so far
	public float time = 0.0f;
	private Point windStrength = PhysicalConstants.WIND;
	private transient Statistics stats;
    // If this is true a few methods are run to see if newly added bodies are too large/small/fast/...
	public static final boolean SAFETY = true;
	public World() {
		this.bodies = new IdSystem<Body>();
		joints = new IdSystem<Joint>();
		numBodies = bodies.size();
		numJoints = 0;
		collisions = new CollisionHandler(this);
		gravity = new Gravity(this);
		airfriction = new AirFriction(this);
		wind = new Wind(this, windStrength);
		arbiters = new Vector<Arbiter>();
		sleepingSystem = new SleepingSystem(this);
		stats = new Statistics();
	}
	public void handle() {
		externalForces();
		integrateVelocities();
		cleanConnections();
		checkCollisions();
		solve();
		integratePositions();
		refresh();
		handleSleeping();
		// phyicalStats();
		time += PhysicalConstants.DT;
	}
	private void checkCollisions() {
		Vector<Pair> contacts = collisions.handle(PhysicalConstants.DT);
		arbiters.clear();
		for (Pair p : contacts) {
			if (p.arbiter == null) {
				p.arbiter = new Arbiter(p);
			}
			addConnection(p.arbiter); // Important : This way there will be a
										// connection even if the two bodies are
										// sleeping.
			arbiters.add(p.arbiter);
		}
	}
	private void handleSleeping() {
		sleepingSystem.handle(PhysicalConstants.DT);
	}
	/**
	 * Sets up the system for the next frame
	 */
	private void refresh() {
		// Update bodies (move and rotate shapes, set acceleration to zero, ...)
		Body b;
		for (int i = 0; i < numBodies; i++) {
			b = getBody(i);
			b.update(PhysicalConstants.DT);
		}
	}
	private void cleanConnections() {
		Body b;
		for (int i = 0; i < numBodies; i++) {
			b = getBody(i);
			Vector<Connection> toRemove = new Vector<Connection>();
			for (Connection c : b.connections) {
				if (c.joint instanceof Arbiter)
					toRemove.add(c);
			}
			for (Connection c : toRemove) {
				b.connections.remove(c);
			}
		}
	}
	private void integratePositions() {
		Body b;
		for (int i = 0; i < numBodies; i++) {
			b = getBody(i);
			if (b.sleeps)
				continue;
			b.integratePositions(PhysicalConstants.DT);
		}
	}
	private void solve() {
		for (Arbiter a : arbiters) {
			a.preSolve();
		}
		for (int i = 0; i < numJoints; i++) {
			Joint j = getJoint(i);
			if (j.sleeps())
				continue;
			j.preSolve();
		}
		for (int i = 0; i < SOLVEITERATIONS; i++) {
			for (Arbiter a : arbiters) {
				a.solve();
			}
			for (int j = 0; j < numJoints; j++) {
				Joint jo = getJoint(j);
				jo.sleeps();
				jo.solve();
			}
		}
	}
	private void integrateVelocities() {
		Body b;
		for (int i = 0; i < numBodies; i++) {
			b = getBody(i);
			if (b.sleeps)
				continue;
			b.integrateVelocities(PhysicalConstants.DT);
		}
	}
	private void externalForces() {
		gravity.handle();
		airfriction.handle();
		wind.handle();
	}
	/**
	 * Connections are needed to keep track of collision groups. This adds a connection between to bodies and should only be called when a joint is added to the world or a new arbiter is created.
	 * 
	 * @param joint
	 */
	private void addConnection(Joint joint) {
		if (joint instanceof TwoBodyJoint) {
			TwoBodyJoint j = (TwoBodyJoint) joint;
			Connection c = new Connection(j);
			j.b1.connections.add(c);
			j.b2.connections.add(c);
		}
	}
	private void removeConnection(Joint j) {
		if (j instanceof TwoBodyJoint) {
			TwoBodyJoint joint = (TwoBodyJoint) j;
			Connection toRemoveB1 = null, toRemoveB2 = null;
			for (Connection c : joint.b1.connections) {
				if (joint.b1 == c.b1 && joint.b2 == c.b2) {
					toRemoveB1 = c;
					break;
				}
			}
			for (Connection c : joint.b2.connections) {
				if (joint.b1 == c.b1 && joint.b2 == c.b2) {
					toRemoveB2 = c;
					break;
				}
			}
			// assert (toRemoveB1 != null && toRemoveB2 != null);
			if (toRemoveB1 != null)
				joint.b1.connections.remove(toRemoveB1);
			if (toRemoveB2 != null)
				joint.b2.connections.remove(toRemoveB2);
		}
	}
	/**
	 * When a body is added with empty pair vector (happens in network synchronization) this is used to calculate all the possible pairs so the collision detection works properly
	 * 
	 * @author toni
	 * 
	 * @param b
	 * @return
	 */
	public void setUpPairs(Body b) {
		collisions.setUpPairs(b);
	}
	public void setUp() {
		collisions = new CollisionHandler(this);
		sleepingSystem = new SleepingSystem(this);
		gravity = new Gravity(this);
		airfriction = new AirFriction(this);
		wind = new Wind(this, windStrength);
	}
	public void addBody(Body body) {
		bodies.add(body);
		numBodies++;
		collisions.addBody(body);
		if (World.SAFETY) {
			Safety.check(body);
		}
	}
	public void remBody(Body body) {
		bodies.remove(body);
		numBodies--;
		Vector<Pair> pairs = collisions.remBody(body);
		sleepingSystem.handleRemovedBody(body, pairs);
	}
	public void addJoint(Joint joint) {
		joints.add(joint);
		joint.setWorld(this);
		joint.updateBodies();
		numJoints++;
		addConnection(joint);
		sleepingSystem.handleAddedJoint(joint);
	}
	public void remJoint(Joint joint) {
		joints.remove(joint);
		numJoints--;
		removeConnection(joint);
		sleepingSystem.handleRemovedJoint(joint);
	}
	public boolean contains(Body b2) {
		return bodies.contains(b2);
	}
	public boolean contains(Joint j2) {
		return joints.contains(j2);
	}
	public Body getBodyById(int id) {
		return bodies.getById(id);
	}
	public Joint getJointById(int id) {
		return joints.getById(id);
	}
	public Body getBody(int i) {
		return bodies.get(i);
	}
	public Joint getJoint(int i) {
		return joints.get(i);
	}
	public boolean containsBodyId(int id) {
		return bodies.containsId(id);
	}
	public boolean containsJointId(int id) {
		return joints.containsId(id);
	}
	public void applyEffect(Effect e) {
		for (int i = 0; i < this.numBodies; i++) {
			Body b = this.getBody(i);
			if (!b.isStatic()) {
				e.apply(b);
			}
		}
	}
	private void phyicalStats() {
		float energy = 0.0f;
		for (int i = 0; i < numBodies; i++) {
			Body b1 = getBody(i);
			if (b1.isStatic())
				continue;
			energy += b1.vel.square() * 0.5 * b1.mass + b1.avel * b1.avel * 0.5 * b1.inertia + b1.mass * b1.pos.y * PhysicalConstants.GRAVITY;
		}
		float error = 0.0f;
		for (int i = 0; i < numJoints; i++) {
			Joint j1 = getJoint(i);
			if (j1 instanceof JDistanceJoint) {
				JDistanceJoint j = ((JDistanceJoint) j1);
				error += Math.abs(j.getLength() - j.getP1().distance(j.getP2()));
			}
		}
		if (stats.isEmpty()) {
			stats.set("energy", 100);
			stats.set("error", 100);
		}
		stats.put("energy", energy);
		stats.put("error", error);
	}
}
