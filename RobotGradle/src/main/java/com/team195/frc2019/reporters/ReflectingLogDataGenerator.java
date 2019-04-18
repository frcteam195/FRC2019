package com.team195.frc2019.reporters;

import java.lang.reflect.Field;

public class ReflectingLogDataGenerator<T> {

	private final Field[] mFields;

	public ReflectingLogDataGenerator(Class<T> typeClass) {
		mFields = typeClass.getFields();
	}

	public String generateData(T data) {
		StringBuilder sb = new StringBuilder();

		for (Field f : mFields) {
			try {
				String s = f.getName();
				s += ":";
				s += f.get(data).toString();
				s += ";";

				sb.append(s);
			} catch (Exception ex) {
				ConsoleReporter.report(ex);
			}
		}

		return sb.toString();
	}
}
