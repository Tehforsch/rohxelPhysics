package physics.collision;
import physics.Body;
import physics.World;
import physics.joints.TwoBodyJoint;
import tools.Floatmath;
import tools.Point;
/**
 * Solves occuring collisions. preSolve is called once before solve is called n times. This solver will solve vor deltaV = 0, no restitution (bounciness) exists.
 * 
 * @author toni
 * 
 */
public class Arbiter extends TwoBodyJoint {
	float energyFactor;
	public Contact[] contacts;
	private float friction;
	public int num;
	private Point tangent = new Point();
	float relevance;
	// Calculation variables - sad but true
	static Point P = new Point(), Pn = new Point(), Pb = new Point(), Pt = new Point();
	static Point dv = new Point(), dvb = new Point();
	static float dPn, Pn0, dPnb, pnb0, dPt, maxPt, oldTangentImpulse;
	public Arbiter(Pair p) {
		super(p.b1, p.b2);
		assert (p.b1.id < p.b2.id);
		num = (p.pos2 != null) ? 2 : 1;
		contacts = new Contact[num];
		friction = Floatmath.sqrt(b1.friction * b2.friction);
		contacts[0] = new Contact(p.normal, p.pos, -p.depth);
		if (num == 2) {
			contacts[1] = new Contact(p.normal, p.pos2, -p.depth);
		}
		energyFactor = (b1.mass * b2.mass) / (b1.mass + b2.mass);
	}
	public float getRelVel(Body body1, Body body2, Point n) {
		return n.x * (body1.vel.x - body2.vel.x) + n.y * (body1.vel.y - body2.vel.y);
	}
	public float getImpulse(Body body1, Body body2, Point n) {
		return n.x * body1.mass * (body1.vel.x - body2.vel.x) + n.y * body2.mass * (body1.vel.y - body2.vel.y);
	}
	public void getRelVelAtP(Body body1, Body body2, Point p) { // Copies relvel
																// to dv
		dv.x = (body2.pos.y - p.y) * body2.avel + body2.vel.x - ((body1.pos.y - p.y) * body1.avel + body1.vel.x);
		dv.y = -(body2.pos.x - p.x) * body2.avel + body2.vel.y - (-(body1.pos.x - p.x) * body1.avel + body1.vel.y);
	}
	public void getRelBVelAtP(Body body1, Body body2, Point p) {
		dvb.x = body2.biasVel.x - body1.biasVel.x;
		dvb.y = body2.biasVel.y - body1.biasVel.y;
	}
	public void setAccImp(Contact c) {
		P.x = c.normal.x * c.accPn - c.normal.y * c.accPt;
		P.y = c.normal.y * c.accPn + c.normal.x * c.accPt;
	}
	public void preSolve() {
		Contact c;
		impulse = 0.0f;
		relVel = 0.0f;
		for (int i = 0; i < num; i++) {
			c = contacts[i];
			c.r1 = c.pos.sub(b1.pos);
			c.r2 = c.pos.sub(b2.pos);
			if (b1.invmass == 0.0f) {
				impulse += -b2.vel.mul(c.normal) * b2.mass;
				relVel += -b2.vel.mul(c.normal);
				energy = 0.5f * relVel;
			}
			else if (b2.invmass == 0.0f) {
				impulse += b1.vel.mul(c.normal) * b1.mass;
				relVel += b1.vel.mul(c.normal);
				energy = 0.5f * relVel;
			}
			else {
				relVel += getRelVel(b1, b2, c.normal);
				impulse += getImpulse(b1, b2, c.normal);
				energy = relVel * 0.5f * energyFactor;
			}
			// normal-mass, tangent-mass und bias errechnen
			float rn1 = c.r1.mul(c.normal);
			float rn2 = c.r2.mul(c.normal);
			float kNormal = b1.invmass + b2.invmass;
			kNormal += b1.invinertia * (c.r1.square() - Math.pow(rn1, 2)) + b2.invinertia * (c.r2.square() - Math.pow(rn2, 2));
			c.massNormal = 1.0f / kNormal;
			// t = normal(n);
			tangent.x = -c.normal.y;
			tangent.y = c.normal.x;
			float rt1 = c.r1.mul(tangent);
			float rt2 = c.r2.mul(tangent);
			float kTangent = b1.invmass + b2.invmass;
			kTangent += b1.invinertia * (c.r1.square() - Math.pow(rt1, 2)) + b2.invinertia * (c.r2.square() - Math.pow(rt2, 2));
			c.massTangent = 1.0f / kTangent;
			c.bias = World.BIAS * World.INVDT * Floatmath.max(-(c.separation + World.ALLOWEDPENETRATION), 0.0f);
			// Apply normal + friction impulse
			setAccImp(c);
			P = c.normal.mul(c.accPn).add(c.normal.normal().mul(c.accPt));
			b2.applyConstImpulse(P, c.r2);
			b1.applyConstImpulse(P.sMul2(-1), c.r1);
			Pb.from2(c.normal).sMul(c.accPnb);
			b2.applyBiasImpulse(Pb, c.r2);
			b1.applyBiasImpulse(Pb.sMul2(-1), c.r2);
		}
	}
	public void solve() {
		Contact c;
		for (int i = 0; i < num; i++) {
			c = contacts[i];
			getRelVelAtP(b1, b2, c.pos);
			// dv.x += c.normal.x * c.separation * World.BIAS;
			// dv.y += c.normal.y * c.separation * World.BIAS;
			// --- Normale --- //
			dPn = c.massNormal * (-dv.mul(c.normal));
			Pn0 = c.accPn;
			c.accPn = Floatmath.max(Pn0 + dPn, 0.0f);
			dPn = c.accPn - Pn0;
			// Impuls wirken lassen
			Pn.from2(c.normal).sMul(dPn);
			b2.applyConstImpulse(Pn, c.r2);
			b1.applyConstImpulse(Pn.sMul2(-1f), c.r1);
			// --- Position Correction --- //
			getRelBVelAtP(b1, b2, c.pos);
			// float dPnb = c.massNormal *
			// (-b2.getBiasVel(c.pos).sub(b1.getBiasVel(c.pos)).mul(c.normal) +
			// c.bias);
			dPnb = c.massNormal * (-dvb.mul(c.normal) + c.bias);
			pnb0 = c.accPnb;
			c.accPnb = Floatmath.max(pnb0 + dPnb, 0.0f);
			dPnb = c.accPnb - pnb0;
			// Impuls wirken lassen
			Pb.from2(c.normal).sMul(dPnb);
			b2.applyBiasImpulse(Pb, c.r2);
			b1.applyBiasImpulse(Pb.sMul2(-1), c.r1);
			// --- Reibung --- //
			dPt = c.massTangent * -dv.mul(tangent);
			maxPt = friction * c.accPn;
			oldTangentImpulse = c.accPt;
			c.accPt = clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			dPt = c.accPt - oldTangentImpulse;
			// Impuls wirken lassen //
			Pt.from2(tangent).sMul(dPt);
			b2.applyConstImpulse(Pt, c.r2);
			b1.applyConstImpulse(Pt.sMul2(-1), c.r1);
		}
	}
	public float clamp(float toClamp, float min, float max) {
		return Math.max(Math.min(toClamp, max), min);
	}
	public void update(Pair p) {
		int size = (p.pos2 != null) ? 2 : 1;
		if (size != num) {
			contacts = new Contact[size];
			contacts[0] = new Contact(p.normal, p.pos, -p.depth);
			if (size == 2) {
				contacts[1] = new Contact(p.normal, p.pos2, -p.depth);
			}
			num = size;
		}
		else {
			contacts[0].normal = p.normal;
			contacts[0].separation = -p.depth;
			contacts[0].pos = p.pos;
			if (p.pos2 != null) {
				contacts[1].normal = p.normal;
				contacts[1].separation = -p.depth;
				contacts[1].pos = p.pos2;
			}
		}
	}
	public Point getP1() {
		return contacts[0].pos;
	}
	public Point getP2() {
		return contacts[0].pos;
	}
}