package frc.robot.subsystems.messaging;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Messaging extends SubsystemBase implements LoggableInputs {
    private static Messaging instance;
    public static synchronized Messaging getInstance() {
        if (instance == null) instance = new Messaging();
        return instance;
    }

    private StringBuilder messages;
    private boolean enabled;

    private Messaging() {
        messages = new StringBuilder("MESSAGES APPEAR BELOW");
        enabled = false;
    }

    public void addMessage(String newMessage) {
        if (enabled) messages.append("\n" + newMessage);
    }

    public void enableMessaging(boolean enable) {
        enabled = enable;
    }    

    @Override
    public void toLog(LogTable table) {
        table.put("Messages", messages.toString());   
    }

    @Override
    public void fromLog(LogTable table) {}
}