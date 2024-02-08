// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.function.Supplier;

/**
 * Defers Command construction to runtime. Runs the command returned by the
 * supplier when this
 * command is initialized, and ends when it ends. Useful for performing runtime
 * tasks before
 * creating a new command. If this command is interrupted, it will cancel the
 * command.
 *
 * <p>
 * Note that the supplier <i>must</i> create a new Command each call. For
 * selecting one of a
 * preallocated set of commands, use {@link SelectCommand}.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class BuildDynamicCommand extends Command {
  private final Command m_nullCommand = new PrintCommand("[DeferredCommand] Supplied command was null!");

  private final Supplier<Command> m_supplier;
  private Command m_command = m_nullCommand;

  /**
   * Creates a new DeferredCommand that runs the supplied command when
   * initialized, and ends when it
   * ends. Useful for lazily creating commands at runtime. The {@link Supplier}
   * will be called each
   * time this command is initialized. The Supplier <i>must</i> create a new
   * Command each call.
   *
   * @param supplier     The command supplier
   * @param requirements The command requirements. This is a {@link Set} to
   *                     prevent accidental
   *                     omission of command requirements. Use {@link Set#of()} to
   *                     easily construct a requirement
   *                     set.
   */
  @SuppressWarnings("this-escape")
  public BuildDynamicCommand(Supplier<Command> supplier, Set<Subsystem> requirements) {
    m_supplier = requireNonNullParam(supplier, "supplier", "DeferredCommand");
    addRequirements(requirements.toArray(new Subsystem[0]));
  }

  enum State {
    WAITFORCOMMAND, RUNNINGCOMMAND, NOCOMMAND
  };

  private State state;
  private Thread thread;

  @Override
  public void initialize() {
    //System.out.println("Start Initialize");
    state = State.WAITFORCOMMAND;

    thread = new Thread(() -> m_command = m_supplier.get());
    //System.out.println("Created Thread");
    thread.start();
    System.out.println("Thread Started");
  }

  @Override
  public void execute() {
    //System.out.println("Execute" + state);
    if (state == State.WAITFORCOMMAND && !thread.isAlive()) {
      //System.out.println("In thread not alive" + state);
      if (m_command != null) {
        //System.out.println("In m command not equal null");
        CommandScheduler.getInstance().registerComposedCommands(m_command);
        m_command.initialize();
        state = State.RUNNINGCOMMAND;
      } else {
        state = State.NOCOMMAND;
      }
    } else if (state == State.RUNNINGCOMMAND) {
      //System.out.println("Executing");
      m_command.execute();
    }
    //System.out.println("execute end" + state);

  }

  @Override
  public boolean isFinished() {
    //System.out.println("isFinished" + state);
    switch (state) {
      case NOCOMMAND:
        return true;
      // This is a potential error: m_command is return false always
      case RUNNINGCOMMAND:
      System.out.println("case running command");
        if (m_command.isFinished()) {
          // state = State.NOCOMMAND;
          System.out.println("I'm finished" + state);
        }
        return m_command.isFinished();
      case WAITFORCOMMAND:
        return false;
    }
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    // Might have to change the logic to figure out how to end a command.
    System.out.println("In the end method");
    if (state == State.RUNNINGCOMMAND) {
      m_command.end(interrupted);
      m_command = m_nullCommand;
      System.out.println("m_command.end(interrupted)");
      // state = State.NOCOMMAND;
    }
  }
  
  @Override
  @SuppressWarnings("PMD.CompareObjectsWithEquals")
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty(
        "deferred", () -> m_command == m_nullCommand ? "null" : m_command.getName(), null);
  }
}
