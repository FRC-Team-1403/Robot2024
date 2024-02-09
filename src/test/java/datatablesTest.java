import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import team1403.robot.Datables.ShooterValues;
import team1403.robot.Datables.Tables;

public class datatablesTest {
    Tables table;

    @BeforeEach // this method will run before each test
    void setup() {
        ShooterValues[][] plot = new ShooterValues[800][800];
        for (int i = 100; i < 600; i++) {
            for (int j = 100; j < 600; j++) {
                plot[i][j] = new ShooterValues((i + 1) * 10, (i + 1) * 10, (i + 1) * 10);
            }
        }
        table = new Tables(plot);
    }
    @Test // marks this method as a test
    void regressionTest() {
        System.out.println("Low test: " + table.compute(0, 0));
        System.out.println("Hight test: " + table.compute(550, 550));
    }

    @Test // marks this method as a test
    void normalTest() {
        table.compute(0, 0);
    }

    @Test // marks this method as a test
    void extremeRegressionTest() {
        table.compute(0, 0);
    }

    @Test // marks this method as a test
    void zero() {
        table.compute(0, 0);
    }

    @Test // marks this method as a test
    void midPoint() {
        table.compute(0, 0);
    }
}
