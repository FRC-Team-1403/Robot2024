package team1403.lib.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeCommandProxy extends TreeCommand {

    public Command cmd;
    public BooleanSupplier success;

    public TreeCommandProxy(Command c) {
        this(c, () -> true);
    }

    //overrides success to false if you pass in false
    public TreeCommandProxy(Command c, BooleanSupplier override) {
        cmd = c;
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

}
