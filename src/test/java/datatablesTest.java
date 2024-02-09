import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import team1403.robot.Datables.ShooterValues;
import team1403.robot.Datables.Tables;

public class datatablesTest {
    Tables table;

    @BeforeEach // this method will run before each test
    void setup() {
        ShooterValues[][] plot = new ShooterValues[800][800];
        for (int i = 100; i < 600; i=+4) {
            for (int j = 100; j < 600; j+=4) {
                plot[i][j] = new ShooterValues((i + 1) * 10, (i + 1) * 10, (i + 1) * 10);
            }
        }
        table = new Tables(plot);
    }
    // @Test // marks this method as a test
    // void regressionTest() {
    //     System.out.println("Low test: " + table.compute(50, 50));
    //     System.out.println("High test: " + table.compute(700, 700));
    // }

    @Test // marks this method as a test
    void normalTest() {
        System.out.println("Normal test: " + table.compute(400, 400));
    }

    // @Test // marks this method as a test
    // void zero() {
    //     System.out.println("Zero test: " + table.compute(0, 0));
    // }

    // @Test // marks this method as a test
    // void midPoint() {
    //     System.out.println("Zero test: " + table.compute(399, 399));
    // }
}
