package physics.joints;
import physics.Body;
import tools.Point;
import tools.StringTools;
public abstract class TwoBodyJoint extends Joint {
	public transient Body b2;
	public int b2Id = -1;
	protected Point staticr2, p2;
	protected Point r2;
	public TwoBodyJoint(Body b1, Body b2) {
		this(b1, b2, b1.pos, b2.pos);
	};
	public TwoBodyJoint(Body b1, Body b2, Point anchor1, Point anchor2) {
		super(b1, anchor1);
		assert (!b1.isStatic() || !b2.isStatic()) : "Cannot create a joint between two static bodies.";
		this.b2 = b2;
		b2Id = b2.id;
		p2 = new Point(anchor2);
		staticr2 = b2.toLocalSpace(anchor2);
	}
	/**
	 * This calculates the current position-offsets of the bodies.
	 * 
	 */
	@Override
	public void calcPositionOffsets() {
		p1 = b1.toWorldSpace(staticr1);
		p2 = b2.toWorldSpace(staticr2);
		r1 = p1.sub(b1.pos);
		r2 = p2.sub(b2.pos);
	}
	public Point getP2() {
		return p2;
	}
	public void wakeUp() {
		super.wakeUp();
		b2.wakeUp();
	}
	@Override
	public void updateBodies() {
		b1 = world.getBodyById(b1Id);
		b2 = world.getBodyById(b2Id);
	}
	@Override
	public boolean sleeps() {
		return b2.sleeps && super.sleeps();
	}
	public String toFileString() {
		return super.toFileString() + "/" + b2.id + "/" + staticr2.toFileString();
	}
	@Override
	public String fromFileString(String s) {
		s = super.fromFileString(s);
		String[] splitted = s.split("/");
		b2 = world.getBodyById(Integer.valueOf(splitted[0]));
		staticr2 = Point.valueOf(splitted[1]);
		calcPositionOffsets();
		return StringTools.stringFromSubArray(splitted, 2, splitted.length);
	}
}
