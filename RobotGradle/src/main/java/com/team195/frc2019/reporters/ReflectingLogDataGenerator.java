package com.team195.frc2019.reporters;

import com.team195.lib.util.ElapsedTimer;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class ReflectingLogDataGenerator<T> {

	private final Field[] mFields;
	private final List<Object> mObjList;

	public ReflectingLogDataGenerator(Class<T> typeClass) {
		mFields = typeClass.getFields();
		mObjList = new ArrayList<>(mFields.length);
	}
	public List<Object> generateData(T data) {
		mObjList.clear();

		for (Field f : mFields) {
			try {
				mObjList.add(f.getName());
				Object o = f.get(data);
				if (o != null)
					mObjList.add(o instanceof Double ? o : o.toString());
				else
					mObjList.add("null");
			} catch (Exception ex) {
				ConsoleReporter.report(ex);
			}
		}

		return mObjList;
	}
}
