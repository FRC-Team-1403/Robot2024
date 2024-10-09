package team1403.lib.auto;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeCommandProxy extends TreeCommandNode {

    private Supplier<Command> cmd_s;
    private Command cmd = null;
    private BooleanSupplier success;

    public TreeCommandProxy(Command c) {
        this(() -> c, () -> true);
    }

    public TreeCommandProxy(Supplier<Command> c_s) {
        this(c_s, () -> true);
    }

    //overrides success to false if you pass in false
    public TreeCommandProxy(Supplier<Command> c, BooleanSupplier override) {
        cmd_s = c;
        success = override;
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
    public boolean isSuccess() {
        return success.getAsBoolean();
    }

    @Override
    public TreeCommandProxy clone() {
        TreeCommandProxy ret = new TreeCommandProxy(cmd_s, success);
        if(left != null)
            ret.left = left.clone();
        if(right != null)
            ret.right = right.clone();
        return ret;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
