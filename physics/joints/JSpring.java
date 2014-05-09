package physics.joints;
import physics.Body;
import tools.Floatmath;
import tools.Point;
public class JSpring extends TwoBodyJoint {
	private static final float MAXF = 500;
	private float strength;
	private float damp;
	private float rlength;
	private Point normal;
	private float bias;
	public JSpring(Body b1, Body b2, Point anchor1, Point anchor2, float length, float strength, float damp) {
		super(b1, b2, anchor1, anchor2);
		this.strength = strength;
		this.damp = damp;
		this.rlength = length;
	}
	@Override
	public void preSolve() {
		this.calcPositionOffsets();
		normal = p1.sub(p2);
		bias = normal.length();
		normal = normal.mul(1.0f / bias);
	}
	@Override
	public void solve() {
		float f = Floatmath.max(-MAXF, Floatmath.min(strength * (rlength - bias) * 0.5f, MAXF));
		float v1 = b1.vel.mul(normal);
		float v2 = b2.vel.mul(normal);
		if (bias != 0) {
			float d = Floatmath.max(-MAXF, Floatmath.min((v2 - v1) * damp, MAXF));
			f += d;
			b1.applyConstImpulse(normal.mul(f / b1.mass), r1);
			b2.applyConstImpulse(normal.mul(-f / b2.mass), r2);
		}
	}
	@Override
	public String toFileString() {
		return super.toFileString() + "/" + strength + "/" + damp;
	}
	@Override
	public String fromFileString(String s) {
		s = super.fromFileString(s);
		String l[] = s.split("/");
		rlength = Float.valueOf(l[0]);
		strength = Float.valueOf(l[1]);
		damp = Float.valueOf(l[2]);
		return s;
	}
}
