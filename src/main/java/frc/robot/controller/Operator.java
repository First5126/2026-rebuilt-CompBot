package frc.robot.controller;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.Indexer;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  // Private constructor to prevent instantiation from outside
  private Operator() {
    super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  }

  // Public method to access the single instance
  public static Operator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Operator();
    }
    return INSTANCE;
  }

  public static Operator init(Indexer m_indexer) {
    return getInstance();
  }

  @Override
  public Operator configureBindings() {
    // TODO: add methods to bind controller
    return this;
  }
}
