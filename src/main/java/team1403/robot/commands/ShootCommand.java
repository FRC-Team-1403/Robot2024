// package team1403.robot.commands;



// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import team1403.robot.subsystems.IntakeAndShooter;

// public class ShootCommand extends Command {
//     private IntakeAndShooter m_intake;
//     private double rpm;
//     private boolean ready = false;
//     private boolean Done = false;
//     private Timer timer = new Timer();

//     public ShootCommand(IntakeAndShooter intake, double rpm) {
//         m_intake = intake;
//         this.rpm = rpm;
//     }

//     @Override
//     public void initialize() {
//         m_intake.setShooterRPM(rpm);
//         timer.reset();
//     }

//     @Override
//     public boolean isFinished() {
//         // waits 4ms
//         if (ready && timer.hasElapsed(1)) {
//             m_intake.shooterStop();
//             m_intake.setIntakeSpeed(0);
//             return true;
//         }
//         return false;

//     }

//     @Override
//     public void execute() {
//         m_intake.setShooterRPM(rpm);
//         if (!ready && (m_intake.getShooterRPMBottom() > rpm / 1.1 &&  m_intake.getShooterRPMTop() > rpm / 1.1)) {
//             m_intake.setIntakeSpeed(1);
//             ready = true;
//             timer.reset();
//             timer.start();
//         }
//     }
// }