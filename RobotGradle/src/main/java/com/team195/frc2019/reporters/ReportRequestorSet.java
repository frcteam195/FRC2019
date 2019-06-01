package com.team195.frc2019.reporters;

import java.net.InetAddress;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.Consumer;

public class ReportRequestorSet {
	private final ConcurrentMap<ReportRequestor, ReportRequestor> requestorSet;

	public ReportRequestorSet() {
		requestorSet = new ConcurrentHashMap<>();
	}

	@SuppressWarnings({"SuspiciousMethodCalls", "UnusedReturnValue"})
	public synchronized ReportRequestor add(InetAddress value) {
		if (value != null) {
			try {
				if (!requestorSet.containsKey(value))
					return add(new ReportRequestor(value));
				else {
					ReportRequestor r = requestorSet.get(value);
					r.pumpHeartbeat();
					return r;
				}
			} catch (Exception ignored) {

			}
		}
		return null;
	}

	public synchronized ReportRequestor add(ReportRequestor value) {
		if (value != null) {
			try {
				requestorSet.putIfAbsent(value, value);
				ReportRequestor r = requestorSet.get(value);
				r.pumpHeartbeat();
				return r;
			} catch (Exception ignored) {

			}
		}
		return null;
	}

	public synchronized void removeExpiredEntries() {
		requestorSet.entrySet().removeIf((k) -> k.getKey().isExpired());
	}

	public synchronized void forEach(Consumer<? super ReportRequestor> action) {
		requestorSet.forEach((K, V) -> {
			action.accept(K);
		});
	}
}
