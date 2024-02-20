package team1403.robot.Datables;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Tables {
    //key = radius (m), value is whatever we want to store
    //in this case we cannot store thetaz here, and we will init it later
    private HashMap<Double, ShooterValues> data = new HashMap<Double, ShooterValues>();

    //key = theta (radians), value is the distance x offset
    private HashMap<Double, Double> offsets;

    //all interpolation is precomputed
    public static final double data_precision = 0.5;
    //theta is in radians
    public static final double theta_precision = 0.05;
    
    private final double increment = 0.1;
    private HashMap<Double, ShooterValues> computeTable = new HashMap<Double, ShooterValues>();

    public Tables(HashMap<Double, ShooterValues> table,HashMap<Double, Double> offsets ) {
        this.data = table;
        this.offsets = offsets;
        this.computeTable.putAll(table);
        init();
    }

    public double roundToTenths(double num) {
        return Math.round(num * 10.0) / 10.0;
    }

    private static double round_theta(double num)
    {
        return Math.round(num / theta_precision) * theta_precision;
    }

    public ShooterValues get(double location) {
        return data.get(roundToTenths(location));
    }

    public void init() {
        ArrayList<Double> sortedKeys = new ArrayList<Double>(data.keySet());
        Collections.sort(sortedKeys);
        for (int x = 1; x < sortedKeys.size(); x++) {
            double low = roundToTenths(sortedKeys.get(x - 1));
            double high = roundToTenths(sortedKeys.get(x));
            while (low < high) {
                data.put(low, compute(low));
                low += increment;
                low = roundToTenths(low);
            }
        }
    }

    //this function assumes the target is at 0,0
    public ShooterValues getValues(double x, double y)
    {
        double nonoffsettheta = Math.atan2(x,y);

        Double offset = offsets.get(round_theta(nonoffsettheta));

        if(offset == null)
            return null;

        double theta = Math.atan2(x + offset.doubleValue(), y);
        double r = Math.hypot(x + offset.doubleValue(), y);

        ShooterValues ret = data.get(roundToTenths(r));

        if(ret == null)
            return null;

        ret.thetaz = theta;
                
        return ret;
    }
    public ShooterValues compute(double location) {
        ShooterValues lowData = null;
        ShooterValues highData = null;
        double lowDataDistance = Double.NEGATIVE_INFINITY;
        double lowDataX = 0;
        double highDataX = 0;
        double highDataDistance = Double.POSITIVE_INFINITY;

        for (Map.Entry<Double, ShooterValues> entry : computeTable.entrySet()) {
            double key = entry.getKey();
            ShooterValues value = entry.getValue();
            double check = key - location;
            if (check < 0 && check > lowDataDistance) {
                lowDataDistance = check;
                lowData = value;
                lowDataX = location;
            } else if (check > 0 && check < highDataDistance) {
                highDataDistance = check;
                highData = value;
            } else if (check == 0) {
                lowData = value;
                highData = value;
                highDataX = location;
                break;
            }
        }
        return new ShooterValues(interpolate(highData.angle, highDataX, lowData.angle, lowDataX, location),
                interpolate(highData.rpm, highDataX, lowData.rpm, lowDataX, location),
                interpolate(highData.robotAngle, highDataX, lowData.robotAngle, lowDataX, location));
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
