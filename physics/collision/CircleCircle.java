package physics.collision;
import physics.*;
import physics.shapes.Circle;
import tools.Floatmath;
import tools.Point;
/**
 * Provides methods that test intersection of two circles, featuring the typical CCD (continous collision detection, meaning that collisions in future time will be recognized and treated especially), returning a depth, normal and position of the collision
 * 
 * @author Tehforsch
 */
public class CircleCircle {
	static Point dv = new Point(), dx = new Point();
	public static ColInfo collision(Body body1, Body body2, Circle circle1, Circle circle2, float dt) {
		ColInfo col = new ColInfo();
		float sumRad = Floatmath.pow((circle1.radius + circle2.radius), 2f);
		float dist = circle1.pos.squaredDistance(circle2.pos) - sumRad;
		if (dist < 0) {
			col.normal = new Point(circle2.pos.x, circle2.pos.y);
			col.normal.sSub2(circle1.pos).sNormalize2();
			col.pos = new Point(circle1.pos.x + col.normal.x * circle1.radius, circle1.pos.y + col.normal.y * circle1.radius);
			// col.depth =
			// -(Floatmath.sqrt(circle1.pos.sub(circle2.pos).square()) -
			// (circle1.radius + circle2.radius));
			col.depth = -(circle1.pos.distance(circle2.pos) - (circle1.radius + circle2.radius));
			return col;
		}
		dv.from2(body1.vel).sSub(body2.vel);
		if (dv.square() < 0.0001f)
			return null;
		dx.from2(circle1.pos).sSub(circle2.pos);
		col.time = (sumRad - dx.x * dx.x) * dv.y * dv.y;
		col.time += dx.x * dx.y * dv.x * dv.y;
		col.time += (sumRad - dx.y * dx.y) * dv.x * dv.x;
		if (col.time < 0)
			return null;
		col.time = Floatmath.sqrt(col.time);
		col.time += dx.mul(dv);
		col.time /= -dv.square();
		if (col.time > dt || col.time <= 0) {
			return null;
		}
		// NACHSCHAUN
		// col.pos = circle1.pos.add(body1.vel.mul(col.time));
		col.pos = new Point(circle1.pos.x + body1.vel.x * col.time, circle1.pos.y + body1.vel.y * col.time);
		// col.normal =
		// circle2.pos.add(body2.vel.mul(col.time)).sub(col.pos).normalize();
		col.normal = new Point(circle2.pos.x + body2.vel.x * col.time - col.pos.x, circle2.pos.y + body2.vel.y * col.time - col.pos.y);
		col.normal.sNormalize();
		col.depth = 0.0f;
		return col;
	}
}