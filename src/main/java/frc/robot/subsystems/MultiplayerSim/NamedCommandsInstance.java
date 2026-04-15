package frc.robot.subsystems.MultiplayerSim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Utility class for managing named commands */
public class NamedCommandsInstance {
  private final HashMap<String, Command> namedCommands = new HashMap<>();

  /**
   * Registers a command with the given name.
   *
   * @param name the name of the command
   * @param command the command to register
   */
  public void registerCommand(String name, Command command) {
    namedCommands.put(name, command);
  }

  /**
   * Registers a list of commands with their associated names.
   *
   * @param commands the list of commands to register
   */
  public void registerCommands(List<Pair<String, Command>> commands) {
    for (var pair : commands) {
      registerCommand(pair.getFirst(), pair.getSecond());
    }
  }

  /**
   * Registers a map of commands with their associated names.
   *
   * @param commands the map of commands to register
   */
  public void registerCommands(Map<String, Command> commands) {
    namedCommands.putAll(commands);
  }

  /**
   * Returns whether a command with the given name has been registered.
   *
   * @param name the name of the command to check
   * @return true if a command with the given name has been registered, false otherwise
   */
  public boolean hasCommand(String name) {
    return namedCommands.containsKey(name);
  }

  /**
   * Returns the command with the given name.
   *
   * @param name the name of the command to get
   * @return the command with the given name, wrapped in a functional command, or a none command if
   *     it has not been registered
   */
  public Command getCommand(String name) {
    if (hasCommand(name)) {
      return MultiplayerCommandUtil.wrappedEventCommand(namedCommands.get(name));
    } else {
      DriverStation.reportWarning(
          "PathPlanner attempted to create a command '"
              + name
              + "' that has not been registered with NamedCommands.registerCommand",
          false);
      return Commands.none();
    }
  }

  /** Removes all registered named commands. */
  public void clearAll() {
    namedCommands.clear();
  }
}
