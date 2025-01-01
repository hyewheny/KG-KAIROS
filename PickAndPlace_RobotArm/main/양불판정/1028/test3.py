# 그리퍼 제어
from pymycobot.mycobot import MyCobot
import time


mc = MyCobot('COM5',115200)
mc.send_angles([0,0,0,0,0,0],10)