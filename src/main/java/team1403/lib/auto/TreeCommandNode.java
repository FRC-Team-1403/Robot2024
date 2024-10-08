package team1403.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class TreeCommandNode extends Command {
    public TreeCommandNode left = null;
    public TreeCommandNode right = null;

    public boolean isSuccess() { return true; }
}
