#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""controller_test controller."""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_speed = 6.28


left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(10.0)
right_motor.setPosition(10.0)
left_motor.setVelocity(0.1*max_speed)
right_motor.setVelocity(0.1*max_speed)

while robot.step(timestep) != -1:
    pass

# Enter here exit cleanup code.
