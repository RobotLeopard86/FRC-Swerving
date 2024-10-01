package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule fl, fr, bl, br;

    public SwerveDrive() {
        fl = new SwerveModule(DriveConstants.frontLeft);
        fr = new SwerveModule(DriveConstants.frontRight);
        bl = new SwerveModule(DriveConstants.backLeft);
        br = new SwerveModule(DriveConstants.backRight);
    }


}
