import java.util.HashMap;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import team1403.robot.Datables.ShooterValues;
import team1403.robot.Datables.Tables;

public class datatablesTest {
    Tables table;

    @BeforeEach
    void setup() {
        HashMap<Double, ShooterValues> plot = new HashMap<Double, ShooterValues>();
        for (double j = 10; j < 60; j += .1) {
            plot.put(j, new ShooterValues((j + 1) * 10, (j + 1) * 10, (j + 1) * 10));
        }
        table = new Tables(plot);
    }

    @Test // marks this method as a test
    void regressionTest() {
        System.out.println("Low test: " + table.get(8.1));
        System.out.println("High test: " + table.get(80.1));
    }

    @Test // marks this method as a test
    void normalTest() {
        System.out.println("Normal test: " + table.get(42.1));
    }

    @Test // marks this method as a test
    void zero() {
        System.out.println("Zero test: " + table.get(0.1));
    }

    @Test // marks this method as a test
    void midPoint() {
        System.out.println("Mod test: " + table.get(45));
    }
}
