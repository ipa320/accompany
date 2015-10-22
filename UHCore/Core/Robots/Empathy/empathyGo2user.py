from Robots.robotFactory import Factory
import time

if __name__ == '__main__':
    c = Factory.getRobot('Care-O-Bot 3.2')
    # base straight
    c.setComponentState('base', [1.52, 0.4, 1.55], False)
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    c.setComponentState('torso', 'home')
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    c.setComponentState('torso', 'home')
