package team1403.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface IAprilTagCamera {
 
    //gets the 3d pose returned by the camera
    public Pose3d getPose();

    //gets the 2d pose
    public Pose2d getPose2D();

    //gets the timestamp of the latest pose
    public double getTimestamp();

    //gets the estimated standard deviation of the current pose
    public Matrix<N3, N1> getEstStdv();

    public boolean hasTarget();

    public boolean hasPose();
    
    public boolean checkVisionResult();
}
