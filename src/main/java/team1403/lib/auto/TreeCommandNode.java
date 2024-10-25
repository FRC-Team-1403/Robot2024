package team1403.lib.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeCommandNode extends Command implements Cloneable {
    public final ArrayList<TreeCommandNode> child = new ArrayList<>();

    public int getBranch() { return 0; }

    public TreeCommandNode setNext(TreeCommandNode... nodes) {

        child.clear();
        for(TreeCommandNode n : nodes) {
            child.add(n);
        }

        return this;
    }

    //clone the entire tree recursively
    @Override
    public TreeCommandNode clone() { 
        TreeCommandNode ret = new TreeCommandNode();
        for(TreeCommandNode n : child) {
            ret.child.add(n.clone());
        }
        return ret;
    }
}
