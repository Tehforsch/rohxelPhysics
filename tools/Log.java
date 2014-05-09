package tools;
public class Log {
	private static final boolean PHYSICS = false;
	public static void p(String k) {
		if (PHYSICS) {
			Help.p(k);
		}
		Help.p(k);
	}
}
