package physics.joints;
import physics.Body;
import physics.World;
import tools.Floatmath;
import tools.Log;
import tools.Point;
public class JDistanceJoint extends TwoBodyJoint {
	static private Point dv = new Point();
	private static Point dvb = new Point();
	private float k;
	private Point normal;
	private float length;
	private float bias;
	private float accP;
	private float accBP;
	public JDistanceJoint(Body b1, Body b2, Point anchor1, Point anchor2) {
		this(b1, b2, anchor1, anchor2, anchor1.distance(anchor2));
	}
	public JDistanceJoint(Body b1, Body b2, Point anchor1, Point anchor2, float length) {
		super(b1, b2, anchor1, anchor2);
		if (b1.isStatic() && b2.isStatic()) {
			Log.p("Connecting two static bodies with a joint is dangerous!");
		}
		this.length = length;
	}
	public void preSolve() {
		// n = (x1 - x2) / |x1 - x2| = the direction (normal) of the joint.
		// We're looking for an impulse nλ to solve
		// (v1 - v2) + (ω1*r1*n) + (ω1*r2*n) + λ/m1 + λ/m2 + λ*r1n^2/I1 +
		// λ*r2n^2/I2 = 0
		// pre-calculate the velocity independent part for faster calculations:
		// k = 1 / (1/m1 + 1/m2 + r1n^2/I1 + r2n^2/I2)
		calcPositionOffsets();
		normal = p1.sub(p2).sNormalize2();
		k = 1 / (b1.invmass + b2.invmass + b1.invinertia * Floatmath.pow(r1.cross(normal), 2) + b2.invinertia * Floatmath.pow(r2.cross(normal), 2));
		bias = (p1.distance(p2) - length) * World.INVDT * World.BIAS;
		b1.applyConstImpulse(normal.mul(accP), r1);
		b2.applyConstImpulse(normal.mul(-accP), r2);
		b1.applyBiasImpulse(normal.mul(accBP), r1);
		b2.applyBiasImpulse(normal.mul(-accBP), r2);
	}
	// private void getRelVel() {
	// dv.x = (b2.pos.y - p2.y) * b2.avel + b2.vel.x - (b1.pos.y - p1.y) *
	// b1.avel - b1.vel.x;
	// dv.y = -(b2.pos.x - p2.x) * b2.avel + b2.vel.y + (b1.pos.x - p1.x) *
	// b1.avel - b1.vel.y;
	// }
	public void getRelVel() { // Copies relvel to dv
		dv.x = -r2.y * b2.avel + b2.vel.x - (-r1.y * b1.avel + b1.vel.x);
		dv.y = r2.x * b2.avel + b2.vel.y - r1.x * b1.avel - b1.vel.y;
	}
	public void getRelBVel() {
		dvb.x = b2.biasVel.x - b1.biasVel.x;
		dvb.y = b2.biasVel.y - b1.biasVel.y;
	}
	@Override
	public void solve() {
		getRelVel();
		getRelBVel();
		float lambda = (dv.mul(normal)) * k;
		float bLambda = (-bias + dvb.mul(normal)) * k;
		accP = accP + lambda;
		// accBP = accBP + bLambda;
		Point P = normal.mul(lambda);
		Point bP = normal.mul(bLambda);
		b1.applyConstImpulse(P, r1);
		b2.applyConstImpulse(P.mul(-1), r2);
		b1.applyBiasImpulse(bP, r1);
		b2.applyBiasImpulse(bP.mul(-1), r2);
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
	public void setLength(float f) {
		length = f;
	}
	public float getLength() {
		return length;
	}
}
