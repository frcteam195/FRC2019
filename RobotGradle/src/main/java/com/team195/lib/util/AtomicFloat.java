package com.team195.lib.util;

import java.util.concurrent.atomic.AtomicInteger;

import static java.lang.Float.floatToIntBits;
import static java.lang.Float.intBitsToFloat;

public class AtomicFloat extends Number {
	private static final long serialVersionUID = -7741346627852362165L;
	private AtomicInteger bits;

	public AtomicFloat() {
		this(0f);
	}

	public AtomicFloat(float initialValue) {
		bits = new AtomicInteger(floatToIntBits(initialValue));
	}

	public final boolean compareAndSet(float expect, float update) {
		return bits.compareAndSet(floatToIntBits(expect),
				floatToIntBits(update));
	}

	public final void set(float newValue) {
		bits.set(floatToIntBits(newValue));
	}

	public final float get() {
		return intBitsToFloat(bits.get());
	}

	public final float getAndSet(float newValue) {
		return intBitsToFloat(bits.getAndSet(floatToIntBits(newValue)));
	}

	public final boolean weakCompareAndSetPlain(float expect, float update) {
		return bits.weakCompareAndSetPlain(floatToIntBits(expect),
				floatToIntBits(update));
	}

	@Override
	public float floatValue() {
		return get();
	}

	@Override
	public double doubleValue() {
		return (double) floatValue();
	}

	@Override
	public int intValue() {
		return (int) get();
	}

	@Override
	public long longValue() {
		return (long) get();
	}

	@Override
	public String toString() {
		return Float.toString(get());
	}
}