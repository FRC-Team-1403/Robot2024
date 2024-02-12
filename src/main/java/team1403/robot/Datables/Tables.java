package team1403.robot.Datables;

public class Tables {
    private HashMap3D table;

    public Tables(HashMap3D table) {
        this.table = table;

    }
    public ShooterValues get(int distance, int xSpeed, int ySpeed) {
        return table.getValue(distance, xSpeed, ySpeed);
    }

    public ShooterValues compute(int distance) {
        Values<ShooterValues[]> pointsX = new Values<>();
        pointsX.findClosest(table, locationX);
        Values<ShooterValues> pointsLow = new Values<>();
        if (pointsX.low != null) {
            pointsLow.findClosest(pointsX.low, locationY);
        }
        Values<ShooterValues> pointsHigh = new Values<>();
        if (pointsX.high != null) {
            pointsHigh.findClosest(pointsX.high, locationY);
        }
        if (pointsX.low != null) {
            pointsLow.findClosest(pointsX.low, locationY);
        }
        ShooterValues high = null;
        ShooterValues low = null;
        if (pointsHigh.high != null && pointsHigh.low != null) {
            high = pointsHigh.high.interpolateOther(pointsHigh.low, pointsHigh.highDataDistance,
                    pointsHigh.lowDataDistance);
        } else {
            return pointsLow.high.interpolateOther(pointsLow.low, pointsLow.highDataDistance,
                    pointsLow.lowDataDistance);
        }
        if (pointsLow.high != null && pointsLow.low != null) {
            low = pointsLow.high.interpolateOther(pointsLow.low, pointsLow.highDataDistance,
                    pointsLow.lowDataDistance);
        } else {
            return high;
        }
        System.out.println("Low is :" + low);
        System.out.println();
        if (pointsX.highDataDistance == 0) {
            return high;
        } else if (pointsX.lowDataDistance == 0) {
            return low;
        }
        return new ShooterValues(
                ShooterValues.interpolate(high.angle, pointsX.highDataDistance, low.angle, pointsX.lowDataDistance),
                ShooterValues.interpolate(high.rpm, pointsX.highDataDistance, low.rpm, pointsX.lowDataDistance),
                ShooterValues.interpolate(high.robotAngle, pointsX.highDataDistance, low.robotAngle,
                        pointsX.lowDataDistance));
    }
}

class Values {
    public ShooterValues highLeft;
    public ShooterValues highRight;
    public ShooterValues lowLeft;
    public ShooterValues lowRight;
    public int xhighDataDistance = 0;
    public int yhighDataDistance = 0;
    public int xlowDataDistance = 0;
    public int ylowDataDistance = 0;

    private final int count = 4;

    public Values() {
    }

    public ShooterValues get()
    public void findClosest(ShooterValues[][] data, int xlocation, int ylocation) {
        RoundOff roundOffXHigh = new RoundOff(count, xlocation, 1);
        xhighDataDistance = roundOffXHigh.offset;
        RoundOff roundOffXLow = new RoundOff(count, xlocation, -1);
        xlowDataDistance = roundOffXLow.offset;
        RoundOff roundOffYHigh = new RoundOff(count, xlocation, 1);
        yhighDataDistance = roundOffYHigh.offset;
        RoundOff roundOffYLow = new RoundOff(count, xlocation, -1);
        ylowDataDistance = roundOffXHigh.offset;
        this.high = findHigh(roundOffHigh.value, data);
        this.low = findLow(roundOffLow.value, data);
        if (this.low == null) {
            this.low = this.high;
            this.lowDataDistance = this.highDataDistance;
            this.high = findHigh(roundOffHigh.value + this.count, data);
            this.highDataDistance += count;
        } else if (this.high == null) {
            this.high = this.low;
            this.highDataDistance = this.lowDataDistance;
            this.low = findLow(roundOffLow.value - this.count, data);
            this.lowDataDistance += count;
        }
    }

    private Points findHigh(int xlocationRounded, int ylocationRounded, ShooterValues[][] data,  ) {

        if (data == null) {
            return null;
        }
        while (xlocationRounded >= 0 && xlocationRounded < data.length) {
            if (data[xlocationRounded] != null) {
                while (ylocationRounded >= 0 && ylocationRounded < data[xlocationRounded].length) {
                    if (data[xlocationRounded][ylocationRounded] != null) {
                        return 
                    }
                    xlocationRounded += count;
                    xhighDataDistance += count;
                }
            }
            xlocationRounded += count;
            xhighDataDistance += count;
        }
        xhighDataDistance = 0;
        xhighDataDistance = 0;
        return null;
    }


    private Points findLow(int locationRounded, T[] data) {
        if (data == null) {
            return null;
        }
        while (locationRounded >= 0 && locationRounded < data.length) {
            if (data[locationRounded] != null) {
                return data[locationRounded];
            }
            locationRounded -= count;
            lowDataDistance += count;
        }
        lowDataDistance = 0;
        return null;
    }
}

class Points {
    ShooterValues high;
    ShooterValues low;
    int xDistance;
    int yDistance;
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
