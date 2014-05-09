package tools;
import java.util.Vector;
public class Statistics {
	public Vector<String> names = new Vector<String>();
	public Vector<Integer> maxNums = new Vector<Integer>();
	public Vector<Vector<Float>> data = new Vector<Vector<Float>>();
	public void set(String name, int maxNum) {
		names.add(name);
		maxNums.add(maxNum);
		data.add(new Vector<Float>());
	}
	public void put(String name, float v) {
		int index;
		for (index = 0; index < names.size(); index++) {
			if (names.get(index).equals(name)) {
				break;
			}
		}
		data.get(index).add(v);
		if (data.get(index).size() > maxNums.get(index)) {
			data.get(index).remove(0);
		}
	}
	public float mean(String name) {
		int index;
		for (index = 0; index < names.size(); index++) {
			if (names.get(index).equals(name)) {
				break;
			}
		}
		float k = 0;
		for (int i = 0; i < data.get(index).size(); i++) {
			k += data.get(index).get(i);
		}
		return k / (float) maxNums.get(index);
	}
	public boolean isEmpty() {
		return names.size() == 0;
	}
}
