package team1403.robot;

public class DataTables {
    private ShooterValues[][] table;

    public DataTables() {
        // test data
        table = new ShooterValues[14][14];
        table [4][8] = new ShooterValues(20, 20, 20);
        table [8][8] = new ShooterValues(20, 20, 20);
        table [8][4] = new ShooterValues(10, 10, 10);
        table [4][4] = new ShooterValues(10, 10, 10);

        };

    public ShooterValues compute(int locationX, int locationY) {
        Values<ShooterValues[]> pointsX = new Values<>();
        pointsX.findClosest(table, locationX);
        System.out.println();
        Values<ShooterValues> pointsLow = new Values<>();
        pointsLow.findClosest(pointsX.low, locationY);
        Values<ShooterValues> pointsHigh = new Values<>();
        pointsHigh.findClosest(pointsX.high, locationY);
        System.out.println(pointsHigh.highDataDistance);
        ShooterValues high = pointsHigh.high.interpolateOther(pointsHigh.low, pointsHigh.highDataDistance, pointsHigh.lowDataDistance);
        System.out.println(high);
        System.out.println();
        ShooterValues low = pointsLow.high.interpolateOther(pointsLow.low, pointsLow.highDataDistance, pointsLow.lowDataDistance);
        System.out.println(low);
        System.out.println();
        if (pointsX.highDataDistance  == 0) {
            return high;
        }
        else if (pointsX.lowDataDistance  == 0) {
            return low;
        }
        return new ShooterValues(
                ShooterValues.interpolate(high.angle, pointsX.highDataDistance, low.angle, pointsX.lowDataDistance),
                ShooterValues.interpolate(high.rpm, pointsX.highDataDistance, low.rpm, pointsX.lowDataDistance),
                ShooterValues.interpolate(high.robotAngle, pointsX.highDataDistance, low.robotAngle, pointsX.lowDataDistance)
        );
    }
}

class Values<T> {
    public T high;
    public T low;
    public int highDataDistance =0;
    public int lowDataDistance = 0;
    private final int count = 4;

    public Values(T high, T low) {
        this.high = high;
        this.low = low;
    }

    public Values() {
    }

    public void findClosest(T[] data, int location) {
        RoundOff roundOffHigh = new RoundOff(count, location, 1);
        highDataDistance = roundOffHigh.offset;
        RoundOff roundOffLow = new RoundOff(count, location, -1);
        lowDataDistance = roundOffLow.offset;
        this.high = findHigh(roundOffHigh.value, data);
        this.low = findLow(roundOffLow.value, data);
        if (this.low == null) {
            this.low = this.high;
            this.lowDataDistance = this.highDataDistance;
            this.high =  findHigh(roundOffHigh.value + this.count, data);
            this.highDataDistance =+ count;
        }else  if (this.high == null) {
            this.high = this.low;
            this.highDataDistance = this.lowDataDistance;
            this.low =  findLow(roundOffLow.value - this.count, data);
            this.lowDataDistance =+ count;
        }
    }

    private T findHigh(int locationRounded, T[] data) {
        if (data == null) {
            return null;
        }
        while (locationRounded != 0 && locationRounded < data.length) {
            try {
                if (data[locationRounded] != null) 
                    return data[locationRounded];
            } catch (Exception e) {}
            locationRounded += count;
            highDataDistance += count;
        }
        highDataDistance = 0;
        return null;
    }

    private T findLow(int locationRounded, T[] data) {
        if (data == null) {
            return null;
        }
        while (locationRounded != 0 && locationRounded < data.length) {
            try {
                if (data[locationRounded] != null) 
                    return data[locationRounded];
            } catch (Exception e) {}
            locationRounded -= count;
            lowDataDistance += count;
        }
        lowDataDistance = 0;
        return null;
    }
}

class ShooterValues {
    public double angle;
    public double rpm;
    public double robotAngle;
    private final int regressionThreshold = 8;
    public ShooterValues interpolateOther(ShooterValues other, int selfDistance, int otherDistance) {
        if (selfDistance  == 0) {
            return this;
        }
        else if (otherDistance  == 0) {
            return other;
        }
        if (selfDistance > otherDistance && selfDistance <= regressionThreshold){
            this.angle = SimpleRegression.calc(this.angle, other.angle, otherDistance, selfDistance);
            this.robotAngle = SimpleRegression.calc(this.robotAngle, other.robotAngle, otherDistance, selfDistance);
            this.rpm = SimpleRegression.calc(this.rpm, other.rpm, otherDistance, selfDistance);
            return this;
        }else if(otherDistance > selfDistance && selfDistance <= regressionThreshold) {
            this.angle = SimpleRegression.calc(this.angle, other.angle, otherDistance, selfDistance);
            this.robotAngle = SimpleRegression.calc(this.robotAngle, other.robotAngle, otherDistance, selfDistance);
            this.rpm = SimpleRegression.calc(this.rpm, other.rpm, otherDistance, selfDistance);
            return this;
        }
        return new ShooterValues(interpolate(this.angle, selfDistance, other.angle, otherDistance),
        interpolate(this.rpm, selfDistance, other.rpm, otherDistance),
        interpolate(this.robotAngle, selfDistance, other.robotAngle, otherDistance));
     }

    public static double interpolate(double highData, int highDataDistance, double lowData, int lowDataDistance) {
        return ((highData / highDataDistance) + (lowData / lowDataDistance)) *
        (highDataDistance * lowDataDistance) / 4;
    }
    public ShooterValues(double angle, double rpm, double offset) {
        this.angle = angle;
        this.rpm = rpm;
        this.robotAngle = offset;
    }

    public String toString() {
        return "angle: " + angle + " rpm: " + rpm + " robotAngle: " + robotAngle;
    }
}


class RoundOff {
    public int value;
    public int offset;
    public RoundOff(int count, int num, int increment) {
        value = num;
        while (value % count != 0) {
            value += increment;
            offset++;
        }
    }
}

class SimpleRegression {
    public static double calc(double first, double last, int distanceFirst, int distanceLast) {
        double slope = (first - last) / (distanceFirst - distanceLast);
        double x = (last - slope * distanceLast) / slope;
        return x;
    }
}