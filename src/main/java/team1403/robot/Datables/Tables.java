package team1403.robot.Datables;

import java.util.HashMap;
import java.util.Map;

public class Tables {
    private HashMap<Double, ShooterValues> table = new HashMap<Double, ShooterValues>();
    private final double increment = 0.1;
    private HashMap<Double, ShooterValues> computeTable = new HashMap<Double, ShooterValues>();

    public Tables(HashMap<Double, ShooterValues> table) {
        this.table = table;
        this.computeTable = table;
    }

    public double roundToTenths(double num) {
        return Math.round(num * 10.0) / 10.0;
    }

    public ShooterValues get(double location) {
        return table.get(roundToTenths(location));
    }

    public void init() {
        boolean evenIteration = false;
        double currentLocation = 0;
        for (Map.Entry<Double, ShooterValues> entry : table.entrySet()) {
            double key = entry.getKey();
            if (!evenIteration) {
                currentLocation = key;
                continue;
            }
            while (key != currentLocation) {
                currentLocation += increment;
                table.put(currentLocation, compute(currentLocation));
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
            // if key negative then it lower
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

        System.out.println("Low Data: " + lowData + ", High Data: " + highData +
                ", Low Data Dist: " + lowDataDistance + ", High Data Dist: " + highDataDistance);
        return new ShooterValues(interpolate(highData.angle, highDataDistance, lowData.angle, lowDataDistance),
                interpolate(highData.rpm, highDataDistance, lowData.rpm, lowDataDistance),
                interpolate(highData.robotAngle, highDataDistance, lowData.robotAngle, lowDataDistance));
    }

    private double interpolate(double highData, double highDataDistance, double lowData, double lowDataDistance) {
        return ((highData / highDataDistance) + Math.abs(lowData / lowDataDistance)) *
                Math.abs(highDataDistance * lowDataDistance);
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