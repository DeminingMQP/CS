#!/usr/bin/env python
import rospy
import numpy as np
from math import *
import RPi.GPIO as GPIO
from demining_mqp.msg import*


class FourBarPosition:
    def __init__(self, height, mdx, mdz, length, space):
        """
        Initializes the Four Bar information for calculation.  Assumes a parallelogram linkage.
        All dimensions are in inches
        :param height: Initial vertical distance
        :param mdx: Initial horizontal distance of the metal detector from the start of the
        :param mdz: Initial height of the metal detector
        :param length: the length of the driven links
        :param space: the separation between the two links
        """
        self.height = float(height)
        self.mdx = float(mdx)
        self.mdz = float(mdz)
        self.linkLength = float(length)
        self.linkSpace = float(space)
        # TODO update with actual dimensions from CAD/Assembled robot
        self.startingAngle = 10
        self.mdXDist = 13
        self.mdZDist = -15
        self.currentAngle = 0
        self.roverHeight = 13.25

    def calcPos(self, encoderValue):
        """
        Calculates and updates the positions of points of interest with respect to the ground.
        Assumes that links are parallel
        :param encoderValue: Encoder ticks from Hall effect encoder on one Tetrix Torquenado motor
        :return: none
        """

        m1deg = encoderValue / 20
        self.currentAngle = radians(m1deg - startingAngle)

        self.height = self.linkLength * sin(self.currentAngle) + self.linkSpace
        self.mdx = self.linkLength * cos(self.currentAngle) + self.mdXDist
        self.mdz = self.linkLength * sin(self.currentAngle) + self.mdZDist

