package physics.shapes;
import tools.Floatmath;
import tools.Point;
// TODO: check ob COM = pos ist, immer?!
public class Polygon extends Shape {
	public Point[] points, lines;
	public Point[] pointsOffset, linesOffset;
	public int numPoints;
	public Polygon(Point pos, Point[] ps) {
		this.pos = pos;
		numPoints = ps.length;
		// Schwerpunkt berechnen
		Point com = calcCOM(ps);
		// Punkte die evtl. falsch vom "User" angegeben wurden richtig ruecken
		// (Zentrum : pos)
		pointsOffset = new Point[numPoints];
		for (int i = 0; i < numPoints; i++) {
			pointsOffset[i] = ps[i].sub(com);
		}
		// Position entsprechend verschieben
		pos = pos.add(com);
		// Polygon fertig berechnen (Linien, Punkte, etc.)
		construct(0.0f);
	}
	private void construct(float a) {
		lines = new Point[numPoints];
		linesOffset = new Point[numPoints];
		calcPoints(a);
		calcLines();
		calcRadius();
	}
	private void calcLines() {
		for (int i = 0, j = numPoints - 1; i < numPoints; i++, j = i - 1) {
			linesOffset[i] = pointsOffset[i].sub(pointsOffset[j]).normalize();
			lines[i] = linesOffset[i];
		}
	}
	private void calcRadius() {
		radius = 0.0f;
		for (int i = 0; i < numPoints; i++) {
			radius = Math.max(radius, pointsOffset[i].length());
		}
	}
	private void calcPoints(float a) {
		points = new Point[numPoints];
		float cosa, sina;
		cosa = Floatmath.cos(a);
		sina = Floatmath.sin(a);
		for (int i = 0; i < numPoints; i++) {
			points[i] = pointsOffset[i].rotate(cosa, sina).add(pos);
		}
	}
	public float calcInertia(float mass) {
		float enumerator = 0.0f;
		float denominator = 0.0f;
		for (int i = 0; i < numPoints; i++) {
			int i2 = (i + 1) % numPoints;
			enumerator += Math.abs(pointsOffset[i2].cross(pointsOffset[i])) * (pointsOffset[i2].square() + pointsOffset[i].square() + pointsOffset[i].mul(pointsOffset[i2]));
			denominator += Math.abs(pointsOffset[i2].cross(pointsOffset[i]));
		}
		return enumerator / denominator * mass / 6.0f;
	}
	public float calcArea() {
		return Math.abs(calcAreaWithSign(pointsOffset));
	}
	private static float calcAreaWithSign(Point[] po) {
		int n = po.length;
		float a = 0.0f;
		for (int i = 0, j = n - 1; i < n; i++, j = i - 1) {
			a += po[j].cross(po[i]);
		}
		a *= 0.5f;
		return a;
	}
	@Override
	public void update() {
		for (int i = 0; i < numPoints; i++) {
			points[i].from2(pointsOffset[i]).sRotate2(parent.cosa, parent.sina).sAdd(parent.pos);
			lines[i] = linesOffset[i].rotate(parent.cosa, parent.sina);
		}
	}
	public static Point calcCOM(Point[] ps) {
		int n = ps.length;
		float a = 0.0f;
		Point p = new Point();
		for (int i = 0, j = n - 1; i < n; i++, j = i - 1) {
			a += ps[j].cross(ps[i]);
			p.x += (ps[i].x + ps[j].x) * ps[j].cross(ps[i]);
			p.y += (ps[i].y + ps[j].y) * ps[j].cross(ps[i]);
		}
		a *= 0.5f;
		return p.mul(1.0f / 6.0f / a);
	}
	public float getWidth() {
		float width = 0;
		for (int i = 0; i < numPoints; i++) {
			width = Math.max(width, Math.abs(points[i].x - pos.x));
		}
		return width;
	}
	public float getHeight() {
		float height = 0;
		for (int i = 0; i < numPoints; i++) {
			height = Math.max(height, Math.abs(points[i].y - pos.y));
		}
		return height;
	}
}
