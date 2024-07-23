#!/usr/bin/python

import numpy as np
import math


def wrap_angle(ang):
    ang_wrapped = np.mod(ang + np.pi, 2*np.pi) - np.pi
    return ang_wrapped

