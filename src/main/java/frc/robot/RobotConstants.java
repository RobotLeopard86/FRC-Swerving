package frc.robot;

public class RobotConstants {
    /// Aceius: What is this even here for? Also why isnt this a static class
    private RobotConstants() {}

    /// Aceius: Convention is to use SCREAMING_CAPS
    /// also this isn't even a constant! It's just static...
    /// I could totally just do RobotConstants.controllerPort = 12 somewhere
    public static int controllerPort = 0;
    
}
