package tools;
import java.util.Vector;
public class Average {
	Vector<Float> values;
	private int size;
	Average(int size) {
		values = new Vector<Float>();
		this.size = size;
	}
	public void feed(float value) {
		values.addElement(value);
		while (values.size() > size) {
			values.remove(0);
		}
	}
	public float getAverage() {
		float k = 0;
		for (Float f : values) {
			k += f;
		}
		return k / values.size();
	}
}
