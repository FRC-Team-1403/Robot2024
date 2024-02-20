import java.util.HashMap;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import team1403.robot.Datables.ShooterValues;
import team1403.robot.Datables.Tables;

public class datatablesTest {
    Tables table;
    HashMap<Double, ShooterValues> checkPlot = new HashMap<Double, ShooterValues>();
    @BeforeEach
    void setup() {
        HashMap<Double, ShooterValues> testPlot = new HashMap<Double, ShooterValues>();
        for (double j = 10; j < 60; j += 1.5) {
            testPlot.put(j, new ShooterValues((j + 1) * 10, (j + 1) * 10, (j + 1) * 10));
        }
        table = new Tables(testPlot, null);
         for (double j = 10; j < 60; j = roundToTenths(j + 0.1)) {
            checkPlot.put(j, new ShooterValues((j + 1) * 10, (j + 1) * 10, (j + 1) * 10));
        }
    }
    public double roundToTenths(double num) {
        return Math.round(num * 10.0) / 10.0;
    }
    @Test // marks this method as a test
    void regressionTest() {
        assert(table.get(8.1) ==  checkPlot.get(8.1));
        assert(table.get(80.1) ==  checkPlot.get(80.1));
        System.out.println("Low test: " + table.get(8.1));
        System.out.println("High test: " + table.get(80.1));
    }

    @Test // marks this method as a test
    void normalTest() {
        System.out.println("Normal test: " + table.get(42) + " Other: " + checkPlot.get(42.0));
                // assert(table.get(42) ==  checkPlot.get(42.0));
    }

    @Test // marks this method as a test
    void zero() {
                assert(table.get(0) ==  checkPlot.get(0.0));
        System.out.println("Zero test: " + table.get(0));
    }

    @Test // marks this method as a test
    void midPoint() {
        System.out.println("Mid test: " + table.get(45.2) + " Other: " + checkPlot.get(45.2));
        // assert(table.get(45.2) ==  checkPlot.get(45.2));
    }
    @Test 
    void angleShot() {
        ShooterValues val = table.getValues(30, 30);
        
    }
}
