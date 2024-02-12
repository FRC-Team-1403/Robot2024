package team1403.robot.Datables;

import java.util.HashMap;

public class HashMap3D {
    private HashMap<Integer, HashMap<Integer, HashMap<Integer, Object>>> map;

    public HashMap3D() {
        this.map = new HashMap<>();
    }

    public void setValue(int x, int y, int z, Object value) {
        map
            .computeIfAbsent(x, k -> new HashMap<>())
            .computeIfAbsent(y, k -> new HashMap<>())
            .put(z, value);
    }

    public Object getValue(int x, int y, int z) {
        return map
            .getOrDefault(x, new HashMap<>())
            .getOrDefault(y, new HashMap<>())
            .get(z);
    }

    public static void main(String[] args) {
        HashMap3D my3DMap = new HashMap3D();

        // Set values
        my3DMap.setValue(1, 2, 3, "Hello");
        my3DMap.setValue(2, 3, 4, "World");

        // Retrieve values
        System.out.println(my3DMap.getValue(1, 2, 3)); // Output: Hello
        System.out.println(my3DMap.getValue(2, 3, 4)); // Output: World
        System.out.println(my3DMap.getValue(5, 6, 7)); // Output: null
    }
}
