package physics.collision;
import tools.Point;
public class Ray {
	public Ray(Point pos, Point dir) {
		super();
		this.pos = pos;
		this.dir = dir;
	}
	public Point pos, dir;
}
