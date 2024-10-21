package team1403.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeAuto extends Command {

    private TreeCommandNode cur;
    private TreeCommandNode orig_root;

    public TreeAuto(TreeCommandNode root) {

        if (root == null) new IllegalArgumentException("Need to pass in a valid TreeCommandNode!");

        cur = root;
        orig_root = root;
    }

    @Override
    public void initialize() {
        cur = orig_root;
        cur.schedule();
    }


    @Override
    public void execute() {

        if (cur == null) return;

        if (cur.isFinished()) {

            if (cur.isSuccess()) cur = cur.left;
            else cur = cur.right;

            if (cur != null) cur.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return cur == null;
    }
}
