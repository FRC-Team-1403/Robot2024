package team1403.lib.auto;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeCommandProxy extends TreeCommandNode {

    private Supplier<Command> cmd_s;
    private Command cmd = null;
    private Supplier<Integer> branch_select;

    public TreeCommandProxy(Command c) {
        this(() -> c, () -> 0);
    }

    public TreeCommandProxy(Command c, Supplier<Integer> override) {
        this(() -> c, override);
    }

    public TreeCommandProxy(Supplier<Command> c_s) {
        this(c_s, () -> 0);
    }

    public TreeCommandProxy(Supplier<Command> c, Supplier<Integer> override) {
        cmd_s = c;
        branch_select = override;
    }

    //binary variant for compatibility with older code
    public TreeCommandProxy(Supplier<Command> c, BooleanSupplier override) {
        cmd_s = c;
        branch_select = () -> override.getAsBoolean() ? 0 : 1;
    }

    public TreeCommandProxy(Command c, BooleanSupplier override) {
        this(() -> c, override);
    }
    
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        cmd.cancel();
      }
      cmd = null;
    }

    @Override
    public void initialize() {
        cmd = cmd_s.get();
        if (cmd != null) cmd.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return cmd == null || !cmd.isScheduled();
    }

    @Override
    public int getBranch() {
        return branch_select.get();
    }

    @Override
    public TreeCommandProxy clone() {
        TreeCommandProxy ret = new TreeCommandProxy(cmd_s, branch_select);
        for(TreeCommandNode n : child) {
            ret.child.add(n.clone());
        }
        return ret;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
