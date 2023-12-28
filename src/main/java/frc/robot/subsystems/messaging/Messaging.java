package frc.robot.subsystems.messaging;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Messaging extends SubsystemBase {
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

    public void logData(LogTable table) {
        table.put("Messages", messages.toString());   
    }
}