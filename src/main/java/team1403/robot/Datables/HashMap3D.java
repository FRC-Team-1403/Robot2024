package team1403.robot.Datables;

import java.util.HashMap;

public class HashMap3D {
    private HashMap<Integer, HashMap<Integer, HashMap<Integer, ShooterValues>>> map;

    public HashMap3D() {
        this.map = new HashMap<>();
    }

    public void setValue(int x, int y, int z, ShooterValues value) {
        map
            .computeIfAbsent(x, k -> new HashMap<>())
            .computeIfAbsent(y, k -> new HashMap<>())
            .put(z, value);
    }

    public ShooterValues getValue(int x, int y, int z) {
        return map
            .getOrDefault(x, new HashMap<>())
            .getOrDefault(y, new HashMap<>())
            .get(z);
    }
}
