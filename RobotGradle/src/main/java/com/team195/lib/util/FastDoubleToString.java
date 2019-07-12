package com.team195.lib.util;

public class FastDoubleToString {
	private static final StringBuilder sb = new StringBuilder();

	private static final long[] POW10 = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

	public static String format(double val) {
		return format(val, 2);
	}

	public static synchronized String format(double val, int precision) {
		if (Double.isNaN(val) || Double.isInfinite(val))
			return "0";
		sb.setLength(0);

		if (val < 0) {
			sb.append('-');
			val = -val;
		}

		long lval = (long) (val * POW10[precision] + 0.5);
		sb.append(lval / POW10[precision]).append('.');
		long fval = lval % POW10[precision];
		for (int p = precision - 1; p > 0 && fval < POW10[p]; p--) {
			sb.append('0');
		}
		sb.append(fval);
		return sb.toString();
	}
}
