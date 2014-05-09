package physics;
import java.io.Serializable;
import java.util.Vector;
import physics.collision.Arbiter;
import physics.collision.Pair;
import physics.shapes.Shape;
import physics.sleeping.Connection;
import tools.Floatmath;
import tools.Point;
import tools.id.IdObject;
/**
 * Represents a physical object consisting of a set of shapes (circles, polygons ... ) with a specific mass, a velocity, a position in space, a rotation, rotational velocity etc.
 * 
 * @see World
 * @author Tehforsch
 */
public class Body implements Serializable, IdObject {
	// Physical variables
	public Point pos, vel, acc;
	public float mass, invmass;
	public float apos, avel, aacc;
	public float inertia, invinertia;
	// Shape dependent variables
	public Shape shape;
	public float width, height;
	// Cosinus and sinus for faster calculation of rotation
	public float cosa, sina;
	// Helping variables for position correction in the contact solver.
	public Point biasVel;
	// Fast pair finding for broad phase algorithms:
	public transient Vector<Pair> pairs;
	public int indexInWorld;
	public int id = IdObject.DEFAULTID;
	public int delIdStartX, delIdEndX, delIdStartY, delIdEndY;
	// List of body-ids with whom this body doesn't collide.
	public Vector<Integer> nonCollidingBodies;
	// This is a flag. Some physics-relevant methods use it for faster
	// calculations and they share the same flag so it doesn't mess up the body
	// class too much.
	// (See CollisionGrouping, Broadphase)
	public boolean flag;
	// Collision grouping and some other parts of the physics engine need to
	// keep track of all bodies that this body is connected to in some ways
	// (joints, contacts)
	public transient Vector<Connection> connections;
	// Variables for sleeping system.
	public boolean sleeps;
	public float idleTime;
	public float unAcceleratedTime;
	public float syncTime;
	public float friction = PhysicalConstants.FRICTION;
	public float restitution;
	public Body(Shape s, float m) {
		shape = s;
		shape.parent = this;
		pos = shape.pos;
		vel = new Point();
		acc = new Point();
		apos = 0.0f;
		avel = 0.0f;
		aacc = 0.0f;
		if (m == 0) { // Body was meant to be static
			mass = Float.POSITIVE_INFINITY;
			inertia = Float.POSITIVE_INFINITY;
			invmass = 0.0f;
			invinertia = 0.0f;
		}
		else {
			mass = m;
			invmass = 1.0f / mass;
			inertia = s.calcInertia(mass);
			invinertia = 1.0f / inertia;
		}
		cosa = Floatmath.cos(apos);
		sina = Floatmath.sin(apos);
		biasVel = new Point();
		width = s.getWidth();
		height = s.getHeight();
		pairs = new Vector<Pair>();
		connections = new Vector<Connection>();
		nonCollidingBodies = new Vector<Integer>();
	}
	public void applyBiasImpulse(Point impulse, Point p) {
		biasVel.x += impulse.x * invmass;
		biasVel.y += impulse.y * invmass;
	}
	public void applyConstImpulse(Point impulse, Point p) {
		vel.x += impulse.x * invmass;
		vel.y += impulse.y * invmass;
		avel += invinertia * p.normal().mul(impulse);
	}
	public void applyImpulse(Point impulse, Point p) {
		vel.x += impulse.x * invmass;
		vel.y += impulse.y * invmass;
		avel += invinertia * p.normal().mul(impulse);
		wakeUp();
	}
	public void applyImpulse(Point impulse) {
		vel.x += impulse.x * invmass;
		vel.y += impulse.y * invmass;
		wakeUp();
	}
	// Does not wake the body up because the force is constant (gravity, ...)
	public void applyConstForce(Point force, Point p) {
		acc.sAdd(force);
		aacc += p.sub(pos).normal().mul(force);
	}
	// Does not wake the body up because the force is constant (gravity, ...)
	public void applyConstForce(Point force) {
		acc.sAdd(force);
	}
	public void applyForce(Point force) {
		acc.sAdd(force);
		wakeUp();
	}
	public void applyForce(Point force, Point p) {
		acc.sAdd(force);
		aacc += p.sub(pos).normal().mul(force);
		wakeUp();
	}
	public void applyConstAngularImpulse(float l) {
		avel += l * invinertia;
	}
	public void integrateVelocities(float dt) {
		vel.x += acc.x * dt * invmass;
		vel.y += acc.y * dt * invmass;
		avel += aacc * dt * invinertia;
	}
	public void integratePositions(float dt) {
		pos.x += (vel.x + biasVel.x) * dt;
		pos.y += (vel.y + biasVel.y) * dt;
		apos += avel * dt;
	}
	public void update(float dt) {
		shape.pos = pos; // GOD. DAMN.
		cosa = Floatmath.cos(apos);
		sina = Floatmath.sin(apos);
		acc = new Point();
		aacc = 0.0f;
		shape.update();
		biasVel.x = 0;
		biasVel.y = 0;
		width = shape.getWidth();
		height = shape.getHeight();
		unAcceleratedTime += PhysicalConstants.DT;
	}
	public Point getVel(Point p) {
		return new Point((pos.y - p.y) * avel + vel.x, -(pos.x - p.x) * avel + vel.y);
	}
	public void unsetStatic(float mass) {
		this.mass = mass;
		invmass = 1.0f / mass;
		inertia = shape.calcInertia(mass);
		invinertia = 1.0f / inertia;
	}
	public void setStatic() {
		mass = Float.POSITIVE_INFINITY;
		invmass = 0.0f;
		inertia = Float.POSITIVE_INFINITY;
		invinertia = 0.0f;
		vel = new Point();
		avel = 0.0f;
		aacc = 0.0f;
		acc = new Point();
	}
	public boolean isStatic() {
		return (invmass == 0.0);
	}
	@Override
	public String toString() {
		return "b" + id;
	}
	public void sleep() {
		sleeps = true;
	}
	public void stop() {
		vel.x = 0;
		vel.y = 0;
		acc.x = 0;
		acc.y = 0;
		avel = 0;
		aacc = 0;
	}
	public void wakeUp() {
		sleeps = false;
		idleTime = 0.0f;
		unAcceleratedTime = 0.0f;
	}
	/**
	 * Converts a given point p into a local anchor-vector by shifting and rotating backwards.
	 * 
	 * @return
	 */
	public Point toLocalSpace(Point p) {
		return p.sub(pos).rotate(cosa, -sina); // subtract the current position, rotate backwards.
	}
	/**
	 * Converts a given anchor-vector r into a global point by rotating and adding the position.
	 * 
	 * @return
	 */
	public Point toWorldSpace(Point r) {
		return r.rotate(cosa, sina).add(pos); // rotate forwards, add the current position
	}
	@Override
	public int getId() {
		return id;
	}
	@Override
	public void setId(int id) {
		this.id = id;
	}
	@Override
	public void setIndexInList(int i) {
		indexInWorld = i;
	}
	public boolean collidesWith(Body b2) {
		return !nonCollidingBodies.contains(b2.id);
	}
	public void setNonColliding(Body b2) {
		addNonCollidingBody(b2.id);
		b2.addNonCollidingBody(id);
	}
	public void setColliding(Body b2) {
		nonCollidingBodies.remove(b2.id);
		b2.nonCollidingBodies.remove(id);
	}
	private void addNonCollidingBody(int ncid) {
		if (!this.nonCollidingBodies.contains(ncid)) {
			nonCollidingBodies.add(ncid);
		}
	}
	public int numCollidingWith() {
		int k = 0;
		for (Connection c : connections) {
			if (c.joint instanceof Arbiter) {
				k++;
			}
		}
		return k;
	}
}