package physics.joints;
import physics.Body;
public class JAngleJoint extends TwoBodyJoint {
	private static final float BIASFACTOR = 1;
	private float k;
	private float accL;
	private float alpha;
	private float bias;
	public JAngleJoint(Body b1, Body b2, float alpha) {
		super(b1, b2);
		alpha = b1.apos - b2.apos;
	}
	@Override
	// Target : alphaDiff = const
	// C = alphaDiff
	// dC/dt = 0
	// L = (w2 - w1) / (1 / I1 + 1 / I2)
	public void preSolve() {
		k = 1 / (b1.invinertia + b2.invinertia);
		b1.applyConstAngularImpulse(accL);
		b2.applyConstAngularImpulse(-accL);
		bias = (b1.apos - b2.apos) - getAngle();
	}
	@Override
	public void solve() {
		float L;
		L = k * ((b2.avel - b1.avel) - BIASFACTOR * bias);
		b1.applyConstAngularImpulse(L);
		b2.applyConstAngularImpulse(-L);
		accL += L;
	}
	@Override
	public String toFileString() {
		return super.toFileString() + "/" + getAngle();
	}
	@Override
	public String fromFileString(String s) {
		s = super.fromFileString(s);
		String[] splitted = s.split("/");
		setAngle(Float.valueOf(splitted[0]));
		return s;
	}
	public void setAngle(float alpha) {
		this.alpha = alpha;
	}
	public float getAngle() {
		return alpha;
	}
}
