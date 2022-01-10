#!/usr/bin/wowpython

# xterm -fa 'Monospace' -fs 2 -geometry 250x100+450+0 -e python print_test.py 
import curses
import numpy as np
import time
import os
from collections import deque


stdscr = curses.initscr()
print(stdscr.getmaxyx())
curses.noecho()
curses.start_color()
curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_BLUE)
curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_RED)
curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_GREEN)
curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_CYAN)
curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_YELLOW)
curses.init_pair(6, curses.COLOR_BLACK, curses.COLOR_BLACK)
i=0
begin_x = 0; begin_y = 0
height = 90; width = 240
win = curses.newwin(height+5, width+5, begin_y, begin_x)

t=0
zero_y=int(height*0.5)
rate_y=height*0.5
x1=deque([], maxlen=width)
x2=deque([], maxlen=width)
x3=deque([], maxlen=width)
x4=deque([], maxlen=width)
x5=deque([], maxlen=width)
for i in range(width):
    x1.append(zero_y)
    x2.append(zero_y)
    x3.append(zero_y)
    x4.append(zero_y)
    x5.append(zero_y)
while True:
    t0=time.time()
    t=t+0.02
    x1.append(int(np.sin(t)*rate_y+zero_y))
    x2.append(int((np.sin(t)+np.sin(3*t)/3)*rate_y+zero_y))
    x3.append(int((np.sin(t)+np.sin(3*t)/3+np.sin(5*t)/5)*rate_y+zero_y))
    x4.append(int((np.sin(t)+np.sin(3*t)/3+np.sin(5*t)/5+np.sin(7*t)/7)*rate_y+zero_y))
    x5.append(int((np.sin(t)+np.sin(3*t)/3+np.sin(5*t)/5+np.sin(7*t)/7+np.sin(9*t)/9)*rate_y+zero_y))
    for col in range(width):
        for row in range(height):
            if row==x1[col]:
                win.addstr(row, col, " ",curses.color_pair(1))
            elif row==x2[col]:
                win.addstr(row, col, " ",curses.color_pair(2))
            elif row==x3[col]:
                win.addstr(row, col, " ",curses.color_pair(3))
            elif row==x4[col]:
                win.addstr(row, col, " ",curses.color_pair(4))
            elif row==x5[col]:
                win.addstr(row, col, " ",curses.color_pair(5))
            else:
                win.addstr(row, col, " ",curses.color_pair(6))
    win.refresh()

time.sleep(1)