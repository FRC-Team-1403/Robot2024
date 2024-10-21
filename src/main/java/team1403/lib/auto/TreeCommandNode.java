package team1403.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeCommandNode extends Command implements Cloneable {
    public TreeCommandNode left = null;
    public TreeCommandNode right = null;

    public boolean isSuccess() { return true; }

    public TreeCommandNode setNext(TreeCommandNode l, TreeCommandNode r) {
        left = l;
        right = r;

        return this;
    }

    public TreeCommandNode setNext(TreeCommandNode l) {
        return setNext(l, null);
    }

    //clone the entire tree recursively
    @Override
    public TreeCommandNode clone() { 
        TreeCommandNode ret = new TreeCommandNode();
        if(left != null)
            ret.left = left.clone();
        if(right != null)
            ret.right = right.clone();
        return ret;
    }
}
