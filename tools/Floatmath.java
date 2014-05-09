package tools;
public class Floatmath {
	public static final float PI = (float) Math.PI;
	public static final float PIHALF = PI * 0.5f;
	public static final float PIQUARTER = PI * 0.25f;
	public static final float E = (float) Math.E;
	public static float sqrt(float x) {
		return (float) Math.sqrt(x);
	}
	public static float random() {
		return (float) Math.random();
	}
	public static int randInt(int min, int max) {
		int diff = max - min;
		return (int) (min + (random() * diff));
	}
	public static float cos(float x) {
		return (float) Math.cos(x);
	}
	public static float sin(float x) {
		return (float) Math.sin(x);
	}
	public static float atan(float x) {
		return (float) Math.atan(x);
	}
	public static float tanh(float x) {
		return (float) Math.tanh(x);
	}
	public static float pow(float x, float y) {
		return (float) Math.pow(x, y);
	}
	public static float max(float x, float y) {
		if (x > y) {
			return x;
		}
		return y;
	}
	public static float gauss(float x, float deviation, float expvalue) {
		return pow(E, -0.5f * pow((x - expvalue) / deviation, 2)); // e ^ (x^2)
	}
	public static float min(float x, float y) {
		if (y > x) {
			return x;
		}
		return y;
	}
	public static float atan2(float y, float x) {
		return (float) Math.atan2(y, x);
	}
	public static float square(float f) {
		return f * f;
	}
	public static float asin(float b) {
		return (float) Math.asin(b);
	}
	public static float log(float k1) {
		return (float) Math.log(k1);
	}
	public static float degToRad(float angle) {
		return angle * (PI / 180.0f);
	}
	public static float radToDeg(float x) {
		return (float) (x * (180 / Math.PI));
	}
}