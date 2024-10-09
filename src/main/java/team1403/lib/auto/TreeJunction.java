package team1403.lib.auto;

import java.util.function.BooleanSupplier;

public class TreeJunction extends TreeCommandNode {
    
    private BooleanSupplier sup;

    public TreeJunction(BooleanSupplier junc) {
        sup = junc;
    }

    @Override
    public boolean isSuccess() {
        return sup.getAsBoolean();
    }

    @Override
    public boolean isFinished() { return true; }

    @Override
    public TreeJunction clone() {
        TreeJunction ret = new TreeJunction(sup);
        ret.left = left.clone();
        ret.right = right.clone();
        return ret;
    }
}
