    package org.firstinspires.ftc.teamcode.PID;

    import com.arcrobotics.ftclib.command.ParallelCommandGroup;
    import org.firstinspires.ftc.teamcode.twoWheelsMr.SimplifiedOdometryRobot;

    public class PIDController extends ParallelCommandGroup {
        private double kP;
        private double targetPosition;

        /*public PIDController(SimplifiedOdometryRobot robot){
            addCommands();
        }*/

        public PIDController(double kP) {
            this.kP = kP;
        }

        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        public double calculate(double currentPosition) {
            double error = targetPosition - currentPosition;
            return error * kP;
        }
    }
