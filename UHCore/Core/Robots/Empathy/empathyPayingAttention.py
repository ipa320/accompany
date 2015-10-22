from Robots.robotFactory import Factory
import time
if __name__ == '__main__':
    c = Factory.getRobot('Care-O-Bot 3.2')
    time.sleep(5)
    # SEQ1
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    # eyes up
    c.setComponentState('head', [[-2.84]], False)
    # eyes down
    c.setComponentState('head', [[-3.14]], False)
    time.sleep(5)
    
    # SEQ2
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    # torso center and up
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    time.sleep(5)
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    
    # SEQ3
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    # eyes up
    c.setComponentState('head', [[-2.84]], False)
    # eyes down
    c.setComponentState('head', [[-3.14]], False)
    time.sleep(5)
    
    # SEQ4
    # eyes up
    c.setComponentState('head', [[-2.84]], False)
    # eyes down
    time.sleep(2)
    c.setComponentState('head', [[-3.14]], False)
    time.sleep(5)
    # torso center and up
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    time.sleep(5)
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    
    # SEQ5
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    time.sleep(1)
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    time.sleep(3)
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    time.sleep(5)
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    time.sleep(3)
    # torso center and up
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    #-----------------------------------
    # 1min 35sec approx
    #-----------------------------------
    # SEQ2
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    # torso center and up
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    time.sleep(5)
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    
    
    # SEQ4
    # eyes up
    c.setComponentState('head', [[-2.84]], False)
    # eyes down
    time.sleep(2)
    c.setComponentState('head', [[-3.14]], False)
    time.sleep(5)
    # torso center and up
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    time.sleep(5)
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    
    # SEQ1
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    # eyes up
    c.setComponentState('head', [[-2.84]], False)
    # eyes down
    c.setComponentState('head', [[-3.14]], False)
    time.sleep(5)
    
    # SEQ5
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    time.sleep(1)
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    time.sleep(3)
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    time.sleep(5)
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    time.sleep(3)
    # torso center and up
    c.setComponentState('torso', [[0, 0, 0, 0.35]])
    
    # SEQ3
    # torso right and down
    c.setComponentState('torso', [[0, 0, 0.2, 0.66]])
    # torso left and down
    c.setComponentState('torso', [[0, 0, -0.2, 0.66]])
    # torso center and down
    c.setComponentState('torso', [[0, 0, 0, 0.66]])
    # eyes up
    c.setComponentState('head', [[-2.84]], False)
    # eyes down
    c.setComponentState('head', [[-3.14]], False)
    time.sleep(5)
    #-----------------------------------
    # 3min 10sec approx
    #-----------------------------------
    
    # base straight
    # c.setComponentState('base', [4.4,1,0])
    # base right
    # c.setComponentState('base', [4.4,1,-0.4])
    # base left
    # c.setComponentState('base', [4.4,1,0.4])
    # base straight
    # c.setComponentState('base', [4.4,1,-0.2], False)
    # torso down
    # c.setComponentState('torso', [[0,0,0,0.35]])
    # torso left and down
    # c.setComponentState('torso', [[0,0,-0.2,0.66]])
    # torso right and down
    # c.setComponentState('torso', [[0,0,0.2,0.66]])
    # c.setComponentState('torso', [[0,0,0,0.35]])
    # c.setComponentState('torso', [[0,0,0,0.66]])
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
