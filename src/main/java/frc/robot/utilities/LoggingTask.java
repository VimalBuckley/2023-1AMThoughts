package frc.robot.utilities;

import java.util.TimerTask;
import java.util.function.Consumer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.messaging.Messaging;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;


public class LoggingTask extends TimerTask {
    @Override
    public void run() {
        // TODO: Change these!
        log("Swerve", Swerve.getInstance()::logData);
        log("Gamepiece Vision", GamePieceVision.getInstance()::logData);
        log("Apriltag Vision", AprilTagVision.getInstance()::logData);
        log("Messaging", Messaging.getInstance()::logData);
    }
    
    private void log(String key, Consumer<LogTable> toLog) {
        Logger.getInstance().processInputs(
            key, 
            new LoggableInputs() {
                public void toLog(LogTable table) {toLog.accept(table);}
                public void fromLog(LogTable table) {}   
            }
        );
    }
}
