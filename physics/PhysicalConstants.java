package physics;
import tools.Point;
public class PhysicalConstants {
	public static final float GRAVITY = -9.81f;
	public static final float UNITSPERMETER = 10;
	public static final float FRICTION = 0.8f;
	public static final float RESTITUTION = 0;
	public static final float DT = 1.0f / 40.0f;
	public static final float ALLOWEDPENETRATION = 0.1f;
	public static final float AIRFRICTION = 0.000001f;
	public static final Point WIND = null; // If null, no wind calculation will
}
