package com.team195.lib.util.statefuser;

public class Quaternion {
	private double w;
	private double x;
	private double y;
	private double z;

	public Quaternion(double w, double x, double y, double z) {
		this.w = w;
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public double getW() {
		return w;
	}

	public synchronized void setW(double w) {
		this.w = w;
	}

	public double getX() {
		return x;
	}

	public synchronized void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public synchronized void setY(double y) {
		this.y = y;
	}

	public double getZ() {
		return z;
	}

	public synchronized void setZ(double z) {
		this.z = z;
	}

	public synchronized void set(double w, double x, double y, double z) {
		this.w = w;
		this.x = x;
		this.y = y;
		this.z = z;
	}
}
