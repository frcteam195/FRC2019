package com.team195.lib.util;

import java.util.concurrent.atomic.AtomicLong;

import static java.lang.Double.doubleToLongBits;
import static java.lang.Double.longBitsToDouble;

public class AtomicDouble extends Number {

	private AtomicLong bits;

	public AtomicDouble() {
		this(0.0);
	}

	public AtomicDouble(double initialValue) {
		bits = new AtomicLong(doubleToLongBits(initialValue));
	}

	public final boolean compareAndSet(double expect, double update) {
		return bits.compareAndSet(doubleToLongBits(expect), doubleToLongBits(update));
	}

	public final void set(double newValue) {
		bits.set(doubleToLongBits(newValue));
	}

	public final double get() {
		return longBitsToDouble(bits.get());
	}

	public final double getAndSet(double newValue) {
		return longBitsToDouble(bits.getAndSet(doubleToLongBits(newValue)));
	}

	public final boolean weakCompareAndSetPlain(double expect, double update) {
		return bits.weakCompareAndSetPlain(doubleToLongBits(expect),
				doubleToLongBits(update));
	}

	@Override
	public double doubleValue() {
		return get();
	}

	@Override
	public float floatValue() {
		return (float) get();
	}

	@Override
	public int intValue() {
		return (int) get();
	}

	@Override
	public long longValue() {
		return (long) get();
	}

	private static final long serialVersionUID = -8742359924652065765L;
}