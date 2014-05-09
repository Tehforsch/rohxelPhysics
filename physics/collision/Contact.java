package physics.collision;
import java.io.Serializable;

import tools.Point;
public class Contact implements Serializable {
	public Point pos, normal;
	public float separation, massNormal, massTangent, bias;
	public float accPn, accPnb, accPt;
	public Point r1, r2;
	public Contact(Point n, Point p, float s) {
		normal = n;
		pos = p;
		separation = s;
	}
}
