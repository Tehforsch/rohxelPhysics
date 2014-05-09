package physics;
import physics.shapes.Polygon;
import tools.Point;
public class AirFriction implements java.io.Serializable {
	World world;
	public AirFriction(World world) {
		this.world = world;
	}
	public void handle() {
		Body b;
		for (int i = 0; i < world.numBodies; i++) {
			b = world.getBody(i);
			Point vDiff = b.vel;
			Point normal = vDiff.normal().sNormalize2();
			float area = getProjectedArea(b, normal);
			Point direction = vDiff.normalize();
			float velSquared = vDiff.square();
			Point force = direction.mul(-PhysicalConstants.AIRFRICTION * PhysicalConstants.UNITSPERMETER * area * velSquared);
			b.applyConstForce(force);
		}
	}
	private float getProjectedArea(Body b, Point normal) {
		if (b.shape instanceof Polygon) {
			float maxP = Float.NEGATIVE_INFINITY, minP = Float.POSITIVE_INFINITY;
			for (Point p : ((Polygon) b.shape).points) {
				float k = p.mul(normal);
				if (k > maxP)
					maxP = k;
				if (k < minP)
					minP = k;
			}
			return maxP - minP;
		}
		return 0.0f;
	}
}
