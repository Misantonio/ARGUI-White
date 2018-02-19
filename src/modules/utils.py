#!/usr/bin/env python
from scipy import signal
from math import pi
import math
import numpy as np
from sympy import symbols, sympify, lambdify, diff
import time
import os

class L(list):
    def __init__(self,maxlen):
        super(L,self).__init__()
        self.maxlen = maxlen

    def __repr__(self):
        return '%s:%s' % (self.__class__.__name__,list.__repr__(self),)

    def append(self, item):
         list.append(self, item)
         if len(self) > self.maxlen:
             self[:-self.maxlen]= []

class RefTrajectory(object):
    def __init__(self,mx, my, mz, myaw):
        t = symbols('t')
        self.subscribers = []
        self.publishers = []
        self.mx_list = []
        self.my_list = []
        self.mz_list = []
        self.myaw_list = []

        if not isinstance(mx, str):
            raise ValueError('mx must be string')
        if not isinstance(my, str):
            raise ValueError('my must be string')
        if not isinstance(mz, str):
            raise ValueError('mz must be string')

        self.sympy_mx = sympify(mx)
        self.sympy_mxp = diff(mx, t)
        self.sympy_mxpp = diff(self.sympy_mxp, t)
        self.func_mx = lambdify(t, self.sympy_mx)
        self.func_mxp = lambdify(t, self.sympy_mxp)
        self.func_mxpp = lambdify(t, self.sympy_mxpp)

        self.sympy_my = sympify(my)
        self.sympy_myp = diff(my, t)
        self.sympy_mypp = diff(self.sympy_mxp, t)
        self.func_my = lambdify(t, self.sympy_my)
        self.func_myp = lambdify(t, self.sympy_myp)
        self.func_mypp = lambdify(t, self.sympy_mypp)

        self.sympy_mz = sympify(mz)
        self.sympy_mzp = diff(mz, t)
        self.sympy_mzpp = diff(self.sympy_mzp, t)
        self.func_mz = lambdify(t, self.sympy_mz)
        self.func_mzp = lambdify(t, self.sympy_mzp)
        self.func_mzpp = lambdify(t, self.sympy_mzpp)

        self.sympy_myaw = sympify(myaw)
        self.sympy_myawp = diff(myaw, t)
        self.sympy_myawpp = diff(self.sympy_myawp, t)
        self.func_myaw = lambdify(t, self.sympy_myaw)
        self.func_myawp = lambdify(t, self.sympy_myawp)
        self.func_myawpp = lambdify(t, self.sympy_myawpp)

        self.get_vals(0)

    def __repr__(self):
        return self.__class__

    def __eval_pos(self, t):
        return self.func_mx(t), self.func_my(t), self.func_mz(t), \
                   self.func_myaw(t)

    def __eval_vel(self, t):
        return self.func_mxp(t), self.func_myp(t), self.func_mzp(t), \
               self.func_myawp(t)

    def __eval_acel(self, t):
        return self.func_mxpp(t), self.func_mypp(t), self.func_mzpp(t), \
               self.func_myawpp(t)

    def append_vals(self, t):
        self.mx_list.append(self.func_mx(t))
        self.my_list.append(self.func_my(t))
        self.mz_list.append(self.func_mz(t))
        self.myaw_list.append(self.func_myaw(t))

    def get_vals(self, t):
        self.mx, self.my, self.mz, self.myaw = self.__eval_pos(t)
        self.mxp, self.myp, self.mzp, self.myawp = self.__eval_vel(t)
        self.mxpp, self.mypp, self.mzpp, self.myawpp = self.__eval_acel(t)

    def get_plot_data(self):
        return [self.mx_list, self.my_list, self.mz_list, self.myaw_list]

def kill_roscore():
    os.system("kill $(ps -ef | grep xterm | grep roscore | head -1 | awk "
              "'{print $2}')")

def start_roscore():
    os.system("xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T roscore "
              "-e roscore&")

def degtorad(x):
    return x*pi/180

def deriv(x,xp,h):
    return (x-xp)/h

def avg_error(arrs):
    """ Function that computes the average error between a group of arrays
        arrs can be a tuple or a list"""

    if len(arrs) == 1:
        set1, set2 = arrs[0]
        err_total = np.sqrt(np.power((set1-set2),2))
        return np.sum(err_total)/len(err_total)
    elif len(arrs) == 3:
        err_total = 0
        for i in range(len(arrs[0][0])):
            e = 0
            for sett in arrs:
                for tup in sett:
                    e += (tup[0][i] - tup[1][i])**2
            err_total += math.sqrt(e)
        return err_total/len(arrs[0][0])


def filter_FIR(cutoff, x, new_val):
    """
    Apply a lowpass FIR filter to a signal

    The x list must be in the next order:
        x = [val[n],val[n-1],val[n-2]...,val[n-N]]
    """

    for i in reversed(range(len(x))):
        if i == 0:
            x[i] = new_val
        else:
            x[i] = x[i - 1]

    order = len(x)
    coefs = signal.firwin(order, cutoff, window='hamming')
    filtered_signal = 0
    for i in range(order):
        filtered_signal += coefs[i]*x[i]
    return filtered_signal

def is_empty(text):
    if text == '':
        return True
    else:
        return False

def RK4(f,x,v,tp,h):
    k1 = h*v
    l1 = h*f(x,v,tp)
    k2 = h*(v+(l1/2.0))
    l2 = h*f(x+(k1/2.0),v+(l1/2.0),tp+(h/2.0))
    k3 = h*(v+(l2/2.0))
    l3 = h*f(x+(k2/2.0),v+(l2/2.0),tp+(h/2.0))
    k4 = h*(v+l3)
    l4 = h*f(x+k3,v+l3,tp+h)
    x = x+(1.0/6.0)*(k1+(2.0*k2)+(2.0*k3)+k4)
    v = v+(1.0/6.0)*(l1+(2.0*l2)+(2.0*l3)+l4)
    return x,v

def RK4_2(f,x,v,tp,h):
    k1 = h*f(x,v,tp)
    k2 = h*f(x,v+(k1/2.0),tp+(h/2.0))
    k3 = h*f(x,v+(k2/2.0),tp+(h/2.0))
    k4 = h*f(x,v+k3,tp+h)
    v = v+(1.0/6.0)*(k1+(2.0*k2)+(2.0*k3)+k4)
    return v

def timing_val(func):
    def wrapper(*arg, **kw):
        t1 = time.time()
        res = func(*arg, **kw)
        t2 = time.time()
        if t2-t1 > arg[0].control_period:
            print '[{}] Time elapsed: {}'.format(func.__name__,(t2-t1))
        return func(*arg)
    return wrapper

if __name__ == '__main__':
    ref = RefTrajectory('cos(t/30)','t','t')
    ref.eval_acel(1)

    x = '/hhh/hmll/lasldf.py'
    print x.split('/')[-1][:-3]