package team1403.robot.Datables;

public class Tables {
    private ShooterValues[][] table;

    public Tables(ShooterValues[][] table) {
        this.table = table;

    }
    public void setValues(ShooterValues[][] table) {
        table[1][1] = new ShooterValues(10, 10, 10);
        table[1][2] = new ShooterValues(20, 20, 10);
        table[2][1] = new ShooterValues(30, 30, 20);
        table[2][2] = new ShooterValues(40, 40, 20);
    }
    public ShooterValues compute(int locationX, int locationY) {
        if (locationX == 0 || locationY == 0)
        return new ShooterValues(0, 0, 0);
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
    private final int count = 1;

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