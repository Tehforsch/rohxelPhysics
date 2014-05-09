package physics.joints;
import physics.Body;
public class JBackgroundAngleJoint extends Joint {
	private static final float BIASFACTOR = 1;
	private float k;
	private float accL;
	private float alpha;
	private float bias;
	public JBackgroundAngleJoint(Body b1, float alpha) {
		super(b1);
		alpha = b1.apos;
	}
	@Override
	// Target : alphaDiff = const
	// C = alphaDiff
	// dC/dt = 0
	// L = (w2 - w1) / (1 / I1 + 1 / I2)
	// But I2 = infinity and w2 = 0
	// L = -w1 / I1
	public void preSolve() {
		k = b1.inertia;
		b1.applyConstAngularImpulse(accL);
		bias = b1.apos - alpha;
	}
	@Override
	public void solve() {
		float L;
		L = k * -b1.avel - BIASFACTOR * bias;
		b1.applyConstAngularImpulse(L * strength);
		accL += L * strength;
	}
	@Override
	public String toFileString() {
		return super.toFileString() + "/" + alpha;
	}
	@Override
	public String fromFileString(String s) {
		s = super.fromFileString(s);
		String[] splitted = s.split("/");
		alpha = Float.valueOf(splitted[0]);
		return s;
	}
}
