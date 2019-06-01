package com.team254.lib.util;

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverage {

    private final ArrayList<Double> numbers;
    private final int maxSize;

    public MovingAverage(int maxSize) {
        this.maxSize = maxSize;
        numbers = new ArrayList<>(maxSize);
    }

    public void addNumber(double newNumber) {
        numbers.add(newNumber);
        if (numbers.size() > maxSize) {
            numbers.remove(0);
        }
    }

    public double getAverage() {
        double total = 0;

        if (numbers.size() > 0) {
            for (double number : numbers) {
                total += number;
            }

            return total / numbers.size();
        }
        else {
            return 0;
        }
    }

    public double getLastSample() {
        return numbers.get(numbers.size() - 1);
    }

    public int getSize() {
        return numbers.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        numbers.clear();
    }

    @Override
    public String toString() {
        return Double.toString(getAverage());
    }
}
