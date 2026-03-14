package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public final class Elastic {
    private static final StringTopic notificationTopic =
        NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
    private static final StringPublisher notificationPublisher =
        notificationTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

    public enum NotificationLevel {
        INFO, WARNING, ERROR
    }

    public static void sendNotification(NotificationLevel level, String title, String description, int displayTimeMillis) {
        String json = String.format(
            "{\"level\":\"%s\",\"title\":\"%s\",\"description\":\"%s\",\"displayTime\":%d,\"width\":350.0,\"height\":-1.0}",
            level.name(), title, description, displayTimeMillis
        );
        notificationPublisher.set(json);
    }

    public static void sendNotification(NotificationLevel level, String title, String description) {
        sendNotification(level, title, description, 3000);
    }
}
