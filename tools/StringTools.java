package tools;
public class StringTools {
	/**
	 * @author Tehforsch Returns the concatenation of the strings in the subarray strings[start:end], including start and excluding end.
	 * @return
	 */
	public static String stringFromSubArray(String[] strings, int start, int end) {
		String s = "";
		for (int i = start; i < end; i++) {
			s = s + strings[i];
		}
		return s;
	}
}
