package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.turret.Turret;

public class RunTurretRaw extends Command {
    private final XboxController xboxController;
    private final Turret turret;

    public RunTurretRaw(XboxController xboxController) {
        this.turret = Turret.getInstance();
        this.xboxController = xboxController;

        addRequirements(turret);
    }

    public void execute() {
        double maxAngleDeg = 360.0;

        double xInput = MathUtil.applyDeadband(xboxController.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND);
        double angleDelta = xInput * maxAngleDeg;

        turret.setTurretAngle(new Rotation2d(Units.degreesToRadians(angleDelta)));
    }

    public boolean isFinished() {
        return turret.reachedMaxRotation();
    }

    public void end(boolean interrupted) {}
}