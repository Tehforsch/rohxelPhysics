package tools;
public class ProgramParameters {
	public static boolean contains(String[] args, String parameter) {
		for (String s : args) {
			if (s.contains("-")) {
				s = strip(s);
				if (s.equals(parameter)) {
					return true;
				}
			}
		}
		return false;
	}
	private static String strip(String s) {
		s = s.replace("-", "");
		s = s.replace(" ", "");
		return s;
	}
	/**
	 * @author Tehforsch
	 * @returns The value of a given program parameter.
	 * @example Calling the method with args = ["-l", "Test1" "-p" "Test2" "-c"] and parameter = "p" would result in "Test2".
	 **/
	public static String value(String[] args, String parameter) {
		for (int i = 0; i < args.length; i++) {
			String s = args[i];
			if (s.contains("-")) {
				s = strip(s);
				if (s.equals(parameter)) {
					if (args.length > i + 1) {
						String value = args[i + 1];
						return value;
					}
				}
			}
		}
		return "";
	}
}
