import eyelink
from psychopy import visual, core
import numpy as np
import rospy
import pickle
from random import seed, shuffle
from datetime import datetime
import csv

def main(edffile='testvid.edf', #8 characters max name
         screen_width=1680,
         screen_height=1050,
         full_screen=True,
         dot_duration=2.0, #2.0
         is_random_point=True):
    # read set up
    # with open('eparams.pkl','rb') as handle:
	#     exp_info = pickle.load(handle)
    rospy.init_node("eyelink_sync")
    # create a window
    win = visual.Window(
        size=(screen_width, screen_height), fullscr=full_screen, screen=0,
        allowGUI=True, allowStencil=False,
        monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
        blendMode='avg', useFBO=False)

    # use pixels as units
    win.setUnits('pix')

    # create a visual stimulus for the dots
    dot = visual.Circle(win, radius=30.0,
                        fillColor="white",
                        lineColor="white", 
                        units='pix',
                        fillColorSpace='rgb',
                        lineColorSpace='rgb')
    margins = [0.8*win.size[0]//2, 0.8*win.size[1]//2]
    dot_pos =   [   
                    (-margins[0], margins[1]), (0, margins[1]), (margins[0], margins[1]),
                    (margins[0]/2, margins[1]/2), (-margins[0]/2, margins[1]/2),
                    (-margins[0],           0), (0,           0), (margins[0],           0),
                    (margins[0]/2, -margins[1]/2), (-margins[0]/2, -margins[1]/2),
                    (-margins[0], -margins[1]), (0, -margins[1]), (margins[0], -margins[1])
                ]
    
    # establish a connection to the tracker
    tracker = eyelink.Eyelink(win, edffile)

    # start the recording
    tracker.start_recording()
    #tracker.send_message('Start Trial {}'.format(rospy.get_time()))
    tracker.send_message('Start Trial {}'.format("0"))
    # keep track of time
    master_timer = core.Clock()
    mini_timer = core.Clock()
    idx = 0

    #if is_random_point:
    #    seed(datetime.now())
    #    shuffle(dot_pos)

    dot_time = np.empty([len(dot_pos),1])
    mini_timer.reset()

    for idx in range(len(dot_pos)):
        dot.pos = dot_pos[idx]
        dot_time[idx] = master_timer.getTime()
        while mini_timer.getTime() < dot_duration:
            dot.draw()
            win.flip()
        else:
            idx += 1
            mini_timer.reset()

    dots = np.hstack((np.array(dot_pos),np.array(dot_time)))
    # tracker.send_message('dot_pos {}'.format(dot_pos))
    # tracker.send_message('dot_time {}'.format(dot_time))
    with open('data/dots_calibration.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['dot_pos_x', 'dot_pos_y', 'dot_time'])
        writer.writerows(dots)

    '''
    #win.callOnFlip(lambda: tracker.send_message('dot pos: {}, time: {}'.format(dot.pos, rospy.get_time())))
    while True:
        dot.pos = dot_pos[idx]
        dot.draw()
        if clock.getTime() > dot_duration:
            idx += 1
            clock.reset()
            #win.callOnFlip(lambda: tracker.send_message('dot pos: {}, time: {}'.format(dot.pos, rospy.get_time())))

        if idx >= len(dot_pos):
            break
            # idx=0
            # if is_random_point:
            #     seed(datetime.now())
            #     shuffle(dot_pos)
        
        win.flip()
    '''

    win.close()
    tracker.stop_recording()

if __name__ == '__main__':
    main()