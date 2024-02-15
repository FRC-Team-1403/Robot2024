package team1403.robot.Datables;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Tables {
    private HashMap<Double, ShooterValues> table = new HashMap<Double, ShooterValues>();
    private final double increment = 0.1;
    private HashMap<Double, ShooterValues> computeTable = new HashMap<Double, ShooterValues>();

    public Tables(HashMap<Double, ShooterValues> table) {
        this.table = table;
        this.computeTable.putAll(table);
        init();
    }

    public double roundToTenths(double num) {
        return Math.round(num * 10.0) / 10.0;
    }

    public ShooterValues get(double location) {
        return table.get(roundToTenths(location));
    }

    public void init() {
        ArrayList<Double> sortedKeys = new ArrayList<Double>(table.keySet());
        Collections.sort(sortedKeys);
        for (int x = 1; x < sortedKeys.size(); x++) {
            double low = roundToTenths(sortedKeys.get(x - 1));
            double high = roundToTenths(sortedKeys.get(x));
            while (low < high) {
                table.put(low, compute(low));
                low += increment;
                low = roundToTenths(low);
            }
        }
    }

    public ShooterValues compute(double location) {
        ShooterValues lowData = null;
        ShooterValues highData = null;
        double lowDataDistance = Double.NEGATIVE_INFINITY;
        double highDataDistance = Double.POSITIVE_INFINITY;

        for (Map.Entry<Double, ShooterValues> entry : computeTable.entrySet()) {
            double key = entry.getKey();
            ShooterValues value = entry.getValue();
            double check = key - location;
            if (check < 0 && check > lowDataDistance) {
                lowDataDistance = check;
                lowData = value;
            } else if (check > 0 && check < highDataDistance) {
                highDataDistance = check;
                highData = value;
            } else if (check == 0) {
                lowData = value;
                highData = value;
                break;
            }
        }
        return new ShooterValues(interpolate(highData.angle, highDataDistance, lowData.angle, lowDataDistance, location),
                interpolate(highData.rpm, highDataDistance, lowData.rpm, lowDataDistance, location),
                interpolate(highData.robotAngle, highDataDistance, lowData.robotAngle, lowDataDistance, location));
    }

    private double interpolate(double highData, double highDataDistance, double lowData, double lowDataDistance, double location) {
        return lowData + ((location - lowDataDistance) / (highDataDistance - lowDataDistance)) * (highData - lowData);
    }
}

class RoundOff {
    public int value;
    public int offset;

    public RoundOff(int count, int num, int increment) {
        value = num;
        while (value % count != 0) {
            value += increment;
            offset += Math.abs(increment);
        }
    }
}

class SimpleRegression {
    public static double calc(double first, double last, int distanceFirst, int distanceLast) {
        double l = (first - last) / (distanceFirst - distanceLast);
        if (l == 0) {
            l = 1;
        }
        return (last - l * distanceLast) / l;
    }
}
