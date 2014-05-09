package physics.joints;
import java.io.Serializable;

import physics.Body;
import physics.World;
import tools.Point;
import tools.StringTools;
import tools.id.IdObject;
public abstract class Joint implements Serializable, IdObject {
	protected transient World world;
	public int id = IdObject.DEFAULTID;
	public transient Body b1;
	public int b1Id = IdObject.DEFAULTID;
	public float impulse;
	public float relVel;
	public float energy;
	protected Point staticr1;
	protected Point p1;
	protected Point r1;
	protected float strength = 1;
	public abstract void preSolve();
	public abstract void solve();
	public Joint(Body b1, Point anchor1) {
		this.b1 = b1;
		b1Id = b1.id;
		p1 = new Point(anchor1);
		staticr1 = b1.toLocalSpace(anchor1);
		strength = 1;
	}
	public Joint(Body b1) {
		this.b1 = b1;
		b1Id = b1.id;
	}
	public Point getP1() {
		return p1;
	}
	public void calcPositionOffsets() {
		p1 = b1.toWorldSpace(staticr1);
		r1 = p1.sub(b1.pos);
	}
	public boolean sleeps() {
		return b1.sleeps;
	}
	public void wakeUp() {
		b1.wakeUp();
	}
	public void setWorld(World world) {
		this.world = world;
	}
	public void updateBodies() {
		b1 = world.getBodyById(b1Id);
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
	}
	public String toFileString() {
		return b1.id + "/" + staticr1.toFileString();
	}
	public String fromFileString(String s) {
		String[] splitted = s.split("/");
		b1 = world.getBodyById(Integer.valueOf(splitted[0]));
		staticr1 = Point.valueOf(splitted[1]);
		calcPositionOffsets();
		return StringTools.stringFromSubArray(splitted, 2, splitted.length);
	}
	public void setStrength(float strength) {
		this.strength = strength;
	}
	public float getStrength() {
		return strength;
	}
}
