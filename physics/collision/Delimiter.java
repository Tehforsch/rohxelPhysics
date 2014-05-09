package physics.collision;
import java.io.Serializable;

import physics.Body;
public class Delimiter implements Serializable {
	public Body body;
	public float pos;
	public boolean isStart;
	public Delimiter(Body b, float p, boolean s) {
		body = b;
		pos = p;
		isStart = s;
	}
	@Override
	public String toString() {
		if (isStart)
			return "<(" + body.id + ")" + pos;
		return pos + "(" + body.id + ")>";
	}
}
