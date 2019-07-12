package com.team195.lib.util.statefuser;

public class Velocity3d {
	private double x;
	private double y;
	private double z;

	public Velocity3d(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public double x() {
		return x;
	}

	public synchronized void setX(double x) {
		this.x = x;
	}

	public double y() {
		return y;
	}

	public synchronized void setY(double y) {
		this.y = y;
	}

	public double z() {
		return z;
	}

	public synchronized void setZ(double z) {
		this.z = z;
	}

	public synchronized void set(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public synchronized void set(Velocity3d velocity3d) {
		this.x = velocity3d.x;
		this.y = velocity3d.y;
		this.z = velocity3d.z;
	}
}
