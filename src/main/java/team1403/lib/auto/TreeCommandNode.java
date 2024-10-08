package team1403.lib.auto;

public class TreeCommandNode {
    private TreeCommand command;
    public TreeCommandNode left;
    public TreeCommandNode right;

    public TreeCommandNode(TreeCommand cmd) {
        this.command = cmd;
        left = right = null;
    }

    public TreeCommand getCommand() {
        return command;
    }

    public void initialize() {
        command.initialize();
    }

    public boolean isFinished() {
        return command.isFinished();
    }

    public boolean isSuccess() {
        return command.isSuccess();
    }

    public void execute() {
        command.execute();
    }

}
