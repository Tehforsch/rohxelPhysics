package physics.joints;
import physics.Body;
import tools.Floatmath;
import tools.Log;
import tools.Point;
public class JBackgroundDistanceJoint extends Joint {
	static private Point dv = new Point();
	private float k;
	private Point normal;
	private float length;
	private float bias;
	private float accP;
	private Point p2;
	public JBackgroundDistanceJoint(Body b1, Point anchor1, Point anchor2) {
		this(b1, anchor1, anchor2, anchor1.distance(anchor2));
	}
	public JBackgroundDistanceJoint(Body b1, Point anchor1, Point anchor2, float dist) {
		super(b1, anchor1);
		if (b1.isStatic()) {
			Log.p("Connecting a static body with the background is dangerous and useless!");
		}
		p2 = new Point(anchor2);
		length = dist;
	}
	public void preSolve() {
		// n = (x1 - x2) / |x1 - x2| = the direction (normal) of the joint.
		// We're looking for an impulse nλ to solve
		// (v1 - v2) + (ω1*r1*n) + (ω1*r2*n) + λ/m1 + λ/m2 + λ*r1n^2/I1 +
		// λ*r2n^2/I2 = 0
		// pre-calculate the velocity independent part for faster calculations:
		// k = 1 / (1/m1 + 1/m2 + r1n^2/I1 + r2n^2/I2)
		super.calcPositionOffsets();
		normal = p1.sub(p2).sNormalize2();
		k = 1 / (b1.invmass + b1.invinertia * Floatmath.pow(r1.cross(normal), 2));
		bias = p1.distance(p2) - length;
		b1.applyConstImpulse(normal.mul(accP), r1);
	}
	// private void getRelVel() {
	// dv.x = (b2.pos.y - p2.y) * b2.avel + b2.vel.x - (b1.pos.y - p1.y) *
	// b1.avel - b1.vel.x;
	// dv.y = -(b2.pos.x - p2.x) * b2.avel + b2.vel.y + (b1.pos.x - p1.x) *
	// b1.avel - b1.vel.y;
	// }
	public void getRelVel() { // Copies relvel to dv
		dv.x = -(r1.y * b1.avel + b1.vel.x);
		dv.y = r1.x * b1.avel - b1.vel.y;
	}
	@Override
	public void solve() {
		getRelVel();
		float lambda = (dv.mul(normal) - 3f * bias) * k;
		accP = accP + lambda;
		Point P = normal.mul(lambda);
		b1.applyConstImpulse(P, r1);
	}
	public Point getP2() {
		return p2;
	}
	@Override
	public String toFileString() {
		return super.toFileString() + "/" + length;
	}
	@Override
	public String fromFileString(String s) {
		s = super.fromFileString(s);
		length = Float.valueOf(s);
		return s;
	}
}
