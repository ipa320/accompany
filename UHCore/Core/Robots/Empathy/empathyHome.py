from Robots.robotFactory import Factory
import time

if __name__ == '__main__':
    c = Factory.getRobot('Care-O-Bot 3.2')
    # base straight
    # c.setComponentState('base', [4.4,1,0])
    # base right
    # c.setComponentState('base', [4.4,1,-0.4])
    # base left
    # c.setComponentState('base', [4.4,1,0.4])
    # base straight and torso down and eyes down
    # c.setComponentState('base', [4.4,1,-0.2], False)
    # c.setComponentState('head', [[-3.14]], False)
    # c.setComponentState('torso', [[0,0,0,0.35]])
    # c.setComponentState('torso', [[0,0,0,0.35]])
    # eyes up and torso left and up
    c.setComponentState('head', 'front', False)
    # c.setComponentState('torso', [[0,0,-0.2,0]])
    # torso right and up
    c.setComponentState('torso', 'home')
    # torso down and center
    # c.setComponentState('torso', [[0,0,0,0.66]])
    # c.setComponentState('head', [[-2.84]],False)
    # c.setComponentState('head', [[-2.84]])
    # c.setComponentState('torso', [[0,0,0.2,0.7]])
    # c.setComponentState('torso', [[0,0,0,0.35]])
    # c.setComponentState('torso', [[0,0,0,0.7]])
    # c.setComponentState('head', [[-2.84]], False)
    # time.sleep(1)
    # c.setComponentState('head', [[-3.14]], False)
    # time.sleep(1)
    # c.setComponentState('torso', 'right')
    # torso up
    # c.setComponentState('torso', 'left')
    # c.setComponentState('torso', 'home', False)
    # c.setComponentState('head', 'front', False)
    # c.setComponentState('torso', 'left')
    # c.setComponentState('torso', 'right')
    # c.setComponentState('torso', 'front')
    # c.setComponentState('head', [[-3.0]], False)
    # time.sleep(1)
    # c.setComponentState('head', [[-2.8]], False)
    # time.sleep(1)
    # c.setComponentState('head', [[-3.0]], False)
    # time.sleep(1)
    # c.setComponentState('head', [[-2.8]], False)
