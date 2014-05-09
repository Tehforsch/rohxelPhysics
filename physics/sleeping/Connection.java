package physics.sleeping;
import physics.Body;
import physics.joints.Joint;
import physics.joints.TwoBodyJoint;
public class Connection {
	public Body b1, b2;
	public Joint joint;
	public Connection(TwoBodyJoint j) {
		super();
		this.joint = j;
		this.b1 = j.b1;
		this.b2 = j.b2;
	}
	public Body otherThan(Body b) {
		if (b == b1)
			return b2;
		if (b == b2)
			return b1;
		return null;
	}
	@Override
	public String toString() {
		return "[" + b1.toString() + " <-> " + b2.toString() + "]";
	}
}
