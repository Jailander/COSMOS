#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
__author__  =   "Jifu Zhao"
__email__   =   "jzhao59@illinois.edu"
__date__    =   "04/07/2017"
__modify__  =   "05/15/2017"
"""

# -------------------------------------------------------------------------
# Currently there are two problems
# 1. for semivariogram, when h = 0, is it correct to manually setting 0 ?
#       line 160, 183
# 2. for covariance C, use estimated variance or fitted curve to calculate ?
#       line 161, 162, 184, 185
# 3. for estimated variance, use estimated variance or fitted curve ?
#       line 194, 195
# -------------------------------------------------------------------------

import warnings
import numpy as np
import pandas as pd
from itertools import product
import matplotlib.pyplot as plt
from sklearn.metrics.pairwise import pairwise_distances

warnings.filterwarnings("ignore")


class PoissonKriging(object):
    """ Poisson Kriging in 2D applications """
    def __init__(self):
        """ initialization of Poisson Kriging """
        self.x = None           # location in x direction, 1d array
        self.y = None           # location in y direction, 1d array
        self.z = None           # measurements, 1d array
        self.loc = None         # location array, 2d array, format (x, y)
        self.distance = None    # pairwise distance, 2d array
        self.lags = None        # number of lags for semivariogram model
        self.mu = None          # estimated mean value
        self.var_z = None       # estimated variance of z
        self.var_y = None       # estimated variance of y
        self.c0 = None          # nugget c0 for semivariogram
        self.c = None           # sill c for semivariogram
        self.a = None           # range a for semivariogram
        self.grids = None       # location to be estimated, 2d array
        self.xmin = None        # minimum x location
        self.xmax = None        # maximum x location
        self.ymin = None        # minimum y location
        self.ymax = None        # maximum y location
        self.pred = None        # predicted value corresponding to grids
        self.pred_var = None    # predicted variance
        self.A = None           # matrix A to solve A*x = b
        self.inv_A = None       # inverse of matrix A
        self.alpha = None       # Lagrange multiplier
        self.model = None       # spherical, gaussian or exponential

    def fit(self, x, y, z, xmin=None, xmax=None, ymin=None,
            ymax=None, xsplit=100, ysplit=100):
        """ fit the model, calculate the pairwise distance """
        self.x = x  # location in x direction, 1d array
        self.y = y  # location in y direction, 1d array
        self.z = z  # measurements, 1d array
        self.loc = np.concatenate((self.x[:, np.newaxis],
                                   self.y[:, np.newaxis]), axis=1)

        # calculate the estimate of mean value and variance
        self.mu = np.mean(z)
        self.var_z = np.var(z)
        self.var_y = self.var_z - self.mu

        # calculate the pairwise l2 distance
        self.distance = pairwise_distances(self.loc, metric='l2', n_jobs=-1)

        # split the experimental region into grids
        self.xmin = min(x) if (xmin == None) else xmin
        self.xmax = max(x) if (xmax == None) else xmax
        self.ymin = min(y) if (ymin == None) else ymin
        self.ymax = max(y) if (ymax == None) else ymax
        self.grids = self._grid(self.xmin, self.xmax, self.ymin, self.ymax,
                                xsplit, ysplit)

    def semivariogram(self, c0_range, c_range, a_range, lags=10, model=None,
                      figsize=(10, 5), plotv=True):
        """
            plot the semivariogram for exploratory analysis (for z not y)
            model can be 'spherical', 'exponential', 'gaussian' or None
        """
        # calculate the semivariogram
        h, gamma = self._semivariogram(lags, model)

        # calculate the fitted curve
        c0, c, a, x, pred, mse = self._fit_model(h, gamma, c0_range, c_range,
                                                 a_range, model)

        print('*' * 40)
        print('  {0:25s} {1:10.3f}'.format('Minimum distance is:',
                                           np.min(self.distance)))
        print('  {0:25s} {1:10.3f}'.format('Maximum distance is:',
                                           np.max(self.distance)))
        print('  {0:25s} {1:10.3f}'.format('Best c0 (or nugget) is:', c0))
        print('  {0:25s} {1:10.3f}'.format('Best c (or sill) is:', c))
        print('  {0:25s} {1:10.3f}'.format('Best a (or range) is:', a))
        print('*' * 40)

        if plotv:
            # plot the semivariogram curve
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)
            ax1.set_title('Semivariogram Curve', fontsize=15)
            ax1.plot(h, gamma, '.-', label="Actual Value")
            ax1.plot(x, pred, 'r-', label="Fitted Value")
            ax1.set_xlabel("Distance")
            ax1.set_ylabel("Semivariance")
            ax1.legend(fontsize=12)
    
            ax2.set_title("Distribution of Distances", fontsize=15)
            ax2.hist(self.distance.reshape(-1), bins=len(h), rwidth=0.6)
            ax2.set_xlabel("Distance")
            ax2.set_ylabel("Frequency")

        return fig, mse, c0, c, a

    def predict(self, c0_range=None, c_range=None, a_range=None, loc=None,
                lags=10, c0=None, c=None, a=None, model=None, fit=True):
        """ function to make predictions """
        self.model = model
        self.lags = lags

        if fit == True:
            # calculate the semivariogram
            h, gamma = self._semivariogram(lags, model)

            # calculate the fitted curve
            self.c0, self.c, self.a, x, pred, mse =\
                self._fit_model(h, gamma, c0_range, c_range, a_range, model)
        else:
            self.c0, self.c, self.a = c0, c, a

        # make predictions
        if loc == None:
            self.pred = np.zeros(len(self.grids))
            self.pred_var = np.zeros(len(self.grids))
            for i in range(len(self.grids)):
                x = self.grids[i][0]
                y = self.grids[i][1]
                self.pred[i], self.pred_var[i] = self._solve(x, y, model)
        else:
            x = loc[0]
            y = loc[1]
            return self._solve(x, y, model)

    def _solve(self, x, y, model):
        """ function to solve A*x = b, return prediction and variance """
        # calculate matrix A
        n = len(self.z)
        if (self.A == None) or (self.model != model):
            self.model = model
            self.A = np.ones((n + 1, n + 1))
            semi_z = self._fit(self.c0, self.c, self.a, self.distance, model)
            semi_y = semi_z - self.mu
            # semi_y[self.distance == 0] = 0  # ---------------- problem ????
            # C = self.c - semi_y - self.mu  # ---------------- problem ????
            C = self.var_y - semi_y
            self.A[:n, :n] = C
            for i in range(n):
                self.A[i, i] += self.mu
            self.A[n, n] = 0

            try:
                self.inv_A = np.linalg.inv(self.A)
            except:
                print("Cannot do inverse on A, use pseudo-inverse instead")
                try:
                    self.inv_A = np.linalg.pinv(self.A)
                except:
                    print("Cannot do pseudo-inverse on A, error")
                    return

        # make predictions on location (x, y)
        b = np.ones(n + 1)
        h = np.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        semi_z = self._fit(self.c0, self.c, self.a, h, model)
        semi_y = semi_z - self.mu
        # semi_y[h == 0] = 0  # ------------------------------ problem ????
        # C = self.c - semi_y - self.mu  # ---------------- problem ????
        C = self.var_y - semi_y

        b[:n] = C
        coeff = np.dot(self.inv_A, b)
        Lambda = coeff[:n]  # lambda coefficient
        self.alpha = coeff[-1]  # Lagrange multiplier

        # calculate prediction and corresponding variance
        pred = np.sum(Lambda * self.z)
        # var = self.c - self.mu - np.sum(Lambda * C) - self.alpha  # -problem?
        var = self.var_y - np.sum(Lambda * C) - self.alpha

        return pred, var

    def _semivariogram(self, lags, model):
        """ function to fit the semivariogram model (for z not y) """
        # sort the distance
        dist_sort = sorted(self.distance.reshape(-1))
        step = len(dist_sort) // lags

        h = []  # distance
        gamma = []  # semivariogram value at distance h
        for i in range(lags):
            low = dist_sort[i * step]
            tmp_idx = min((i + 1) * step, len(dist_sort) - 1)
            upp = dist_sort[tmp_idx]
            idx1, idx2 = np.where((self.distance >= low) &
                                  (self.distance <= upp))
            if len(idx1) == 0:
                continue
            tmp = np.mean([0.5 * (self.z[i] - self.z[j]) ** 2
                          for (i, j) in zip(idx1, idx2)])
            h.append(np.mean(self.distance[idx1, idx2]))
            gamma.append(tmp)

        return h, gamma

    def _fit_model(self, h, gamma, c0_range, c_range, a_range, model):
        """
            function to fit the semivariogram model by minimize the MSE
            model: "spherical", "exponential", "gaussian"
        """
        if c0_range == None:
            mse = np.zeros((len(c_range), len(a_range)))
            for i in range(len(c_range)):
                for j in range(len(a_range)):
                    tmp_c = c_range[i]
                    tmp_a = a_range[j]
                    fitted = self._fit(0, tmp_c, tmp_a, h, model)
                    mse[i, j] = np.mean((gamma - fitted) ** 2)
        else:
            mse = np.zeros((len(c0_range), len(c_range), len(a_range)))
            for i in range(len(c0_range)):
                for j in range(len(c_range)):
                    for k in range(len(a_range)):
                        tmp_c0 = c0_range[i]
                        tmp_c = c_range[j]
                        tmp_a = a_range[k]
                        fitted = self._fit(tmp_c0, tmp_c, tmp_a, h, model)
                        mse[i, j, k] = np.mean((gamma - fitted) ** 2)

        # find the minimum mse
        if c0_range == None:
            idx = np.array(np.where(mse == np.min(mse)))[:, 0]
            c0 = 0
            c = c_range[idx[0]]
            a = a_range[idx[1]]
        else:
            idx = np.array(np.where(mse == np.min(mse)))[:, 0]
            c0 = c0_range[idx[0]]
            c = c_range[idx[1]]
            a = a_range[idx[2]]

        # make predictions
        x = np.linspace(min(h), max(h), len(h) * 5)
        pred = self._fit(c0, c, a, x, model)

        return c0, c, a, x, pred, mse

    def _fit(self, c0, c, a, h, model):
        """ function to fit the model """
        if model == 'spherical':
            fitted = c0 + self._spherical(a, h, c)
        elif model == 'exponential':
            fitted = c0 + self._exponential(a, h, c)
        elif model == 'gaussian':
            fitted = c0 + self._gaussian(a, h, c)
        else:
            print('Error, cannot find the model')
            return

        return fitted

    def _spherical(self, a, h, c):
        """ Spherical model for semivariogram """
        h = np.array(h)
        ans = c * (1.5 * h / a - 0.5 * (h / a) ** 3)
        ans[h > a] = c

        return ans

    def _exponential(self, a, h, c):
        """ Exponential model for semivariogram """
        h = np.array(h)
        ans = c * (1 - np.exp(-h / a))

        return ans

    def _gaussian(self, a, h, c):
        """ Gaussian model for semivariogram """
        h = np.array(h)
        ans = c * (1 - np.exp(- (h / a) ** 2))

        return ans

    def _grid(self, xmin, xmax, ymin, ymax, xsplit, ysplit):
        """ function to divide the interested area into grids """
#        xstep = (xmax - xmin) / xsplit
#        ystep = (ymax - ymin) / ysplit
#        x_loc = np.arange(xmin + xstep / 2.0, xmax, xstep)
#        y_loc = np.arange(ymin + ystep / 2.0, ymax, ystep)

        x_loc = np.arange(xmin, xmax, 1)
        y_loc = np.arange(ymin, ymax, 1)

        # form the grids
        grids = product(x_loc, y_loc)

        return np.array(list(grids))

    def plot2D(self, fitted=False, figsize=(8, 6), s=50):
        """ plot the fitted surface in 2D """
        if (fitted == True) and (self.pred == None):
            print("Error, grid not calculated")
            return

        if fitted == False:
            fig, ax = plt.subplots(figsize=figsize)
            img = ax.scatter(self.x, self.y, c=self.z, s=s)
            ax.axis('image')
            ax.set_xlim((self.xmin, self.xmax))
            ax.set_ylim((self.ymin, self.ymax))
            ax.set_xlabel('X position', fontsize=12)
            ax.set_ylabel('Y position', fontsize=12)
            plt.colorbar(img, fraction=0.046, pad=0.04)

            return fig
        else:
            fig, ax = plt.subplots(figsize=figsize)
            img = ax.scatter(self.grids[:, 0], self.grids[:, 1], c=self.pred,
                             linewidths=0)
            if s != 0:
                ax.scatter(self.x, self.y, facecolor='none', s=s,
                           edgecolors='black', linewidths=0.8)
            ax.axis('image')
            ax.set_xlim((self.xmin, self.xmax))
            ax.set_ylim((self.ymin, self.ymax))
            ax.set_xlabel('X position', fontsize=12)
            ax.set_ylabel('Y position', fontsize=12)
            plt.colorbar(img, fraction=0.046, pad=0.04)

            return fig

    def plot_variance(self, figsize=(8, 6), s=50):
        """ function to plot the variance """
        if self.pred == None:
            print("Error, grid not calculated")
            return

        fig, ax = plt.subplots(figsize=figsize)
        img = ax.scatter(self.grids[:, 0], self.grids[:, 1], c=self.pred_var,
                         linewidths=0)
        if s != 0:
            ax.scatter(self.x, self.y, facecolor='none', s=s,
                       edgecolors='black', linewidths=0.8)
        ax.axis('image')
        ax.set_xlim((self.xmin, self.xmax))
        ax.set_ylim((self.ymin, self.ymax))
        ax.set_xlabel('X position', fontsize=12)
        ax.set_ylabel('Y position', fontsize=12)
        plt.colorbar(img, fraction=0.046, pad=0.04)

        return fig

    def get_result(self, verbose=False):
        """ get the fitted result """
        df = pd.DataFrame({'x': self.grids[:, 0], 'y': self.grids[:, 1],
                           'estimate': self.pred, 'variance': self.pred_var})
        df = df[['x', 'y', 'estimate', 'variance']]

        if verbose == True:
            print('*' * 40)
            print('  {0:25s} {1:10.3f}'.format('Minimum distance is:',
                                               np.min(self.distance)))
            print('  {0:25s} {1:10.3f}'.format('Maximum distance is:',
                                               np.max(self.distance)))
            print('  {0:25s} {1:10.3f}'.format('Best c0 (or nugget) is:',
                                               self.c0))
            print('  {0:25s} {1:10.3f}'.format('Best c (or sill) is:', self.c))
            print('  {0:25s} {1:10.3f}'.format('Best a (or range) is:',
                                               self.a))
            print('*' * 40)

        return self.distance, self.mu, self.c0, self.c, self.a, df                