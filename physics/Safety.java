package physics;
import tools.Log;
public class Safety {
	private static final float MAXALLOWEDMASS = 100f;
	private static final float MINALLOWEDMASS = 0.01f;
	private static final float MAXALLOWEDAREA = 100000;
	private static final float MINALLOWEDAREA = 0;
	public static void check(Body body) {
		if (body.isStatic())
			return;
		if (body.mass > MAXALLOWEDMASS) {
			Log.p("Body with id " + body.id + " is too heavy. Mass : " + body.mass);
		}
		if (body.mass < MINALLOWEDMASS) {
			Log.p("Body with id " + body.id + " is too light. Mass : " + body.mass);
		}
		float area = body.shape.calcArea();
		if (area > MAXALLOWEDAREA) {
			Log.p("Body with id " + body.id + " is too large. Area : " + area);
		}
		if (area < MINALLOWEDAREA) {
			Log.p("Body with id " + body.id + " is too small. Area : " + area);
		}
	}
}
