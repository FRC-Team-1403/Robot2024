package team1403.robot.Datables;

import java.util.HashMap;

public class HashMap3D {
    public HashMap<Double, HashMap<Double, HashMap<Double, ShooterValues>>> map;

    public HashMap3D() {
        this.map = new HashMap<>();
    }

    public void setValue(double x, double y, double z, ShooterValues value) {
        map
            .computeIfAbsent(x, k -> new HashMap<>())
            .computeIfAbsent(y, k -> new HashMap<>())
            .put(z, value);
    }

    public ShooterValues getValue(double x, double y, double z) {
        return map
            .getOrDefault(x, new HashMap<>())
            .getOrDefault(y, new HashMap<>())
            .get(z);
    }
}
