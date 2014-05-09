package tools;
import java.util.Vector;
public class DebugDraw {
	public static Vector<Point> points = new Vector<Point>();
	public static void draw(Point pos) {
		points.add(pos);
	}
}
