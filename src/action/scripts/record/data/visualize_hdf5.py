#!/usr/bin/env python

# Copyright (c) 2016, Simon Brodeur
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#   
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
# OF SUCH DAMAGE.

import io
import sys
import os
import time
import logging
import numpy as np
import cv2
import scipy
import scipy.io.wavfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import multiprocessing
from multiprocessing import Pool

from h5utils import Hdf5Dataset
from optparse import OptionParser

logger = logging.getLogger(__name__)

def is_cv2():
    import cv2 as lib
    return lib.__version__.startswith("2.")
 
def is_cv3():
    import cv2 as lib
    return lib.__version__.startswith("3.")

# Adapted from: http://stackoverflow.com/questions/22867620/putting-arrowheads-on-vectors-in-matplotlibs-3d-plot
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def set_data(self, xs, ys, zs):
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
        
# Adapted from: https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py
def quat2mat(q):
    w, x, y, z = q
    Nq = w*w + x*x + y*y + z*z
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)
    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    return np.array(
           [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])

def exportQuaternionFramesAsVideo(frames, fs, filename, title, grid=False, downsampleRatio=1):

    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs/float(downsampleRatio), codec='libx264', extra_args=['-preset', 'ultrafast'])
    
    fig = plt.figure(figsize=(5,4), facecolor='white', frameon=False)
    
    # Create arrows
    arrows = []
    labels = ['x', 'y', 'z']
    colors = ['r', 'g', 'b']
    vectors = np.eye(3)
    for i in range(3):
        x,y,z = vectors[:,i]
        arrow = Arrow3D([0.0, x], [0.0, y], [0.0, z], mutation_scale=20, 
                        lw=3, arrowstyle="-|>", color=colors[i], label=labels[i])
        arrows.append(arrow)
    
    ax = fig.gca(projection='3d')
    fig.tight_layout()
    fig.subplots_adjust(left=0.10, bottom=0.10)
    
    ax.grid(grid)
    ax.set_title(title)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.get_xaxis().set_ticks([])
    ax.get_yaxis().set_ticks([])
    ax.set_zticks([])
    ax.view_init(elev=45.0, azim=45.0)
    ax.axis([-1.0, 1.0, -1.0, 1.0])
    ax.set_zlim(-1.0, 1.0)
    
    # Create arrows
    for arrow in arrows:
        ax.add_artist(arrow)
    
    proxies = [plt.Rectangle((0, 0), 1, 1, fc=c) for c in colors]
    legend = ax.legend(proxies, labels, loc='upper right')
        
    startTime = time.time()
    with writer.saving(fig, filename, 100):

        for n, frame in enumerate(frames):
            
            if n % int(downsampleRatio) == 0:
                
                # Convert from quaternion (w, x, y, z) to rotation matrix
                x,y,z,w = frame
                quaternion = np.array([w,x,y,z])
                R = quat2mat(quaternion)
                
                # Apply the rotation to the axis vectors (pointing in Y-axis)
                directions = np.eye(3) # x, y, z as column vectors
                vectors = np.dot(R, directions)
                assert np.allclose(np.linalg.norm(vectors, 2, axis=0), np.ones((3,)), atol=1e-6)
                
                # Update existing plot
                for i in range(3):
                    x,y,z = vectors[:,i]
                    arrows[i].set_data([0.0, x], [0.0, y], [0.0, z])
                    
                writer.grab_frame()
    
    elapsedTime = time.time() - startTime
    logger.info('FPS = %f frame/sec' % (len(frames)/elapsedTime))
    
    plt.close(fig)


def exportPositionFramesAsVideo(framesPos, framesOri, fs, filename, title, grid=False, downsampleRatio=1):
    assert len(framesPos) == len(framesOri)

    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs/float(downsampleRatio), codec='libx264', extra_args=['-preset', 'ultrafast'])
    
    fig = plt.figure(figsize=(5,4), facecolor='white', frameon=False)
    ax = fig.add_subplot(111)
    
    scatPast = ax.scatter([0,], [0,], s=10)
    scatCur = ax.scatter([0,], [0,], c=[1.0,0.0,0.0], s=50)
    fig.tight_layout()
    fig.subplots_adjust(left=0.20, bottom=0.15)
    
    ax.grid(grid)
    ax.set_title(title)
    ax.set_xlabel("x [meter]")
    ax.set_ylabel("y [meter]")
    ax.axis([-1.0, 1.0, -1.0, 1.0])
    
    windowsSize = 10000
    startTime = time.time()
    with writer.saving(fig, filename, 100):

        nbPoints = 0
        data = np.zeros((windowsSize, 2), dtype=np.float32)
        for n, (position, orientation) in enumerate(zip(framesPos,framesOri)):
            
            # Update buffer
            data[0:-1:,:] = data[1::,:]
            data[-1,:] = position[0:2]
            if nbPoints < windowsSize:
                nbPoints += 1
                
            # Convert from quaternion (w, x, y, z) to rotation matrix
            x,y,z,w = orientation
            quaternion = np.array([w,x,y,z])
            R = quat2mat(quaternion)
            
            # Apply the rotation to the vector pointing in the forward direction (y-axis)
            direction = np.array([0.0, 1.0, 0.0])
            vector = np.dot(R, direction)
            vector /= np.linalg.norm(vector, 2)
            
            if n % int(downsampleRatio) == 0:
                cdata = data[windowsSize-nbPoints:,:]
                scatPast.set_offsets(cdata)
                scatCur.set_offsets(cdata[-1,:])
                
                border = 0.5
                xlim = np.array([np.min(cdata[:,0])-border, np.max(cdata[:,0])+border])
                ylim = np.array([np.min(cdata[:,1])-border, np.max(cdata[:,1])+border])
                scale = np.max([xlim[1] - xlim[0], ylim[1] - ylim[0]])
                ax.set_xlim(xlim)
                ax.set_ylim(ylim)

                x, y = cdata[-1,0], cdata[-1,1]
                dx, dy = vector[0] * 0.05 * scale, vector[1] * 0.05 * scale
                lines = ax.plot([x, x+dx], [y, y+dy], c='r')
                
                writer.grab_frame()
                
                lines.pop(0).remove()
    
    elapsedTime = time.time() - startTime
    logger.info('FPS = %f frame/sec' % (len(framesPos)/elapsedTime))
    
    plt.close(fig)

def exportBatteryFramesAsVideo(frames, fs, filename, title, labels, ylim=None, windowSize=None, grid=False, legend=None, downsampleRatio=1):
    
    ndim = 2
    assert frames.shape[1] == ndim
    
    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs/float(downsampleRatio), codec='libx264', extra_args=['-preset', 'ultrafast'])
    
    fig = plt.figure(figsize=(5,4), facecolor='white', frameon=False)
    ax = fig.add_subplot(111)
    fig.tight_layout()
    fig.subplots_adjust(left=0.20, bottom=0.15, right=0.80)
    
    if windowSize is None:
        windowSize = int(2*fs)
    
    xlabel, ylabel, ylabel2 = labels
    ax.grid(grid)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    
    ax2 = ax.twinx()
    ax2.set_ylabel(ylabel2)
    
    if ylim is not None:
        ax.set_xlim([-windowSize/fs, 0.0])
        ax.set_ylim(ylim[0])
        ax2.set_xlim([-windowSize/fs, 0.0])
        ax2.set_ylim(ylim[1])
    
    xdata=-np.arange(windowSize)[::-1]/float(fs)
    ydata=np.zeros(windowSize)
    lines = []
    lines.append(ax.plot(xdata,ydata, '-r')[0])
    lines.append(ax2.plot(xdata,ydata, '-b')[0])
        
    if legend is not None:
        l1 = ax.legend([legend[0],], loc='upper left')
        plt.setp(l1.get_texts(),fontsize='small')
        l2 = ax2.legend([legend[1],], loc='upper right')
        plt.setp(l2.get_texts(),fontsize='small')
        
    data = np.zeros((windowSize, ndim))
    
    startTime = time.time()
    with writer.saving(fig, filename, 100):
        for n, frame in enumerate(frames):
            # Update buffer
            data[0:-1:,:] = data[1::,:]
            data[-1,:] = frame
            
            if n % int(downsampleRatio) == 0:
                
                # Update existing line plots
                for i in range(ndim):
                    ydata=np.array(data[:,i])
                    lines[i].set_data(xdata, ydata)
                    
                writer.grab_frame()
    
    elapsedTime = time.time() - startTime
    logger.info('FPS = %f frame/sec' % (len(frames)/elapsedTime))
    
    plt.close(fig)


def exportSensorFramesAsVideo(frames, fs, filename, title, labels, ylim=None, windowSize=None, grid=False, legend=None, downsampleRatio=1):
    
    ndim = frames.shape[1]
    assert ndim <= 6
            
    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs/float(downsampleRatio), codec='libx264', extra_args=['-preset', 'ultrafast'])
    
    fig = plt.figure(figsize=(5,4), facecolor='white', frameon=False)
    ax = fig.add_subplot(111)
    fig.tight_layout()
    fig.subplots_adjust(left=0.20, bottom=0.15)
    
    if windowSize is None:
        windowSize = int(2*fs)
    
    xlabel, ylabel = labels
    ax.grid(grid)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    
    if ylim is not None and ylim is not 'boolean':
        ax.axis([-windowSize/fs, 0.0, ylim[0], ylim[1]])
    elif ylim == 'boolean':
        ax.axis([-windowSize/fs, 0.0, -0.1, 1.1])
        plt.yticks([0.0, 1.0], ['Off', 'On'])
    
    xdata=-np.arange(windowSize)[::-1]/float(fs)
    ydata=np.zeros(windowSize)
    lines = []
    colors = ['-r', '-g', '-b', 'c', 'm', 'y']
    for i in range(ndim):
        lines.append(ax.plot(xdata,ydata, colors[i])[0])
        
    if legend is not None:
        l = ax.legend(legend, loc='upper left')
        plt.setp(l.get_texts(),fontsize='small')
        
    ymax = 0.0
    data = np.zeros((windowSize, ndim))
    
    startTime = time.time()
    with writer.saving(fig, filename, 100):
        for n, frame in enumerate(frames):
            # Update buffer
            data[0:-1:,:] = data[1::,:]
            data[-1,:] = frame
            
            if n % int(downsampleRatio) == 0:
                
                # Update existing line plots
                for i in range(ndim):
                    ydata=np.array(data[:,i])
                    lines[i].set_data(xdata, ydata)
                    
                if ylim is None:
                    cmax = np.max(np.abs(data))
                    if cmax > ymax:
                        ymax = cmax
                    ax.axis([-windowSize/fs, 0.0, -ymax, ymax])
                    
                writer.grab_frame()
    
    elapsedTime = time.time() - startTime
    logger.info('FPS = %f frame/sec' % (len(frames)/elapsedTime))
    
    plt.close(fig)

def processIRrange(dataset, outDirPath, downsampleRatio=1):

    group = 'collision'
    name = 'range'
    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    # Convert raw data into the interval [0,1]
    maxValue = 65535
    raw = np.array(raw, dtype=np.float32) / maxValue

    title = 'IR range sensors'
    labels = ['Time [sec]', "Amplitude"]
    ylim=[0, 0.25]
    legend = ['Wall', 'Cliff left', 'Cliff front-left', 'Cliff front-right', 'Cliff right']
    exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)

def processContacts(dataset, outDirPath, name=None, downsampleRatio=1):

    group = 'collision'
    names = ['contact', 'cliff', 'wheel-drop']
    
    if name is not None:
        names = [name,]
        
    for name in names:
        [_, _, raw, clock, shape] = dataset.getStates('switch', group)
        
        # Estimate sampling rate from clock
        fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
        logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
        windowSize = 2*fs
    
        outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
        logger.info('Writing to output video file %s' % (outputVideoFile))
    
        # Convert raw data into the interval [0,1]
        raw = np.array(raw, dtype=np.float32)
    
        if name == 'contact':
            # Only keep a subset of the sensors related to bumpers and wall
            raw = raw[:, [0,1,9,10]]
        
            title = 'Contact sensors'
            legend = ['bumper left', 'bumper right', 'wall', 'virtual wall']
        elif name == 'cliff':
            # Only keep a subset of the sensors related to cliff sensors
            raw = raw[:, [5,6,7,8]]
        
            title = 'Cliff sensors'
            legend = ['left', 'front-left', 'front-right', 'right']
            
        elif name == 'wheel-drop':
            # Only keep a subset of the sensors related to wheel and caster drop
            raw = raw[:, [2,3,4]]
        
            title = 'Wheel drop sensors'
            legend = ['caster', 'wheel left', 'wheel right']
              
        labels = ['Time [sec]', "Activation"]
        ylim = 'boolean'
              
        exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)

def processBattery(dataset, outDirPath, downsampleRatio=1):

    group = 'battery'
    name = 'charge'
    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    # Select only the subset related to battery voltage and current
    raw = np.array(raw[:,[0,1]], dtype=np.float32)

    # Convert voltage from mV to V
    raw[:,0] = raw[:,0]/1000.0

    title = 'Battery'
    labels = ['Time [sec]', "Voltage [V]", 'Current [mA]']
    ylim=[[10.0, 18.0], [-1000.0, 1000.0]]
    legend = ['Voltage', 'Current']
    
    exportBatteryFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)

def processPosition(dataset, outDirPath, downsampleRatio=1):
    group = 'odometry'
    name = 'position'
    [_, _, rawPos, clock, _] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    
    [_, _, rawOri, clock, _] = dataset.getStates('orientation', group)

    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, 'orientation', group))
    windowSize = 2*fs
    
    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'Odometry position and orientation (x-y)'
    exportPositionFramesAsVideo(rawPos, rawOri, fs, outputVideoFile, title, grid=True, downsampleRatio=downsampleRatio)
    
def processOrientation(dataset, outDirPath, downsampleRatio=1, rawValues=False):
    group = 'imu'
    
    if rawValues:
        name = 'orientation_raw'
    else:
        name = 'orientation'
    
    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'Orientation'
    exportQuaternionFramesAsVideo(raw, fs, outputVideoFile, title, grid=False, downsampleRatio=downsampleRatio)
    
    
def processMotors(dataset, outDirPath, downsampleRatio=1):
    group = 'motors'
    name = 'speed'
    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'Motor velocity'
    labels = ['Time [sec]', "Motor velocity [mm/s]"]
    ylim=[-250, 250]
    legend = ['left', 'right']
    exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
    
def processImu(dataset, outDirPath, name=None, downsampleRatio=1):
    group = 'imu'
    names = {'imu-gyro':'angular_velocity', 'imu-accel':'linear_acceleration', 'imu-mag':'magnetic_field'}

    # Convert dictionary to list
    if name is not None:
        names = [names[name],]
    else:
        names = names.items()
        
    for name in names:
        [_, _, raw, clock, shape] = dataset.getStates(name, group)
        
        # Estimate sampling rate from clock
        fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
        logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
        windowSize = 2*fs

        outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
        logger.info('Writing to output video file %s' % (outputVideoFile))

        if name == 'angular_velocity':
            title = 'Angular velocity'
            labels = ['Time [sec]', "Angular velocity [rad/s]"]
            ylim=[-2.0, 2.0]
        elif name == 'linear_acceleration':
            title = 'Linear acceleration'
            labels = ['Time [sec]', "Linear acceleration [m/s^2]"]
            ylim=[-12.0, 12.0]
        elif name == 'magnetic_field':
            title = 'Magnetic field'
            labels = ['Time [sec]', "Magnetic field [uT]"]
            raw *= 1e6 # Convert from T to uT
            ylim=[-50.0, 50.0]
        legend = ['x-axis', 'y-axis', 'z-axis']
        
        exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
        
def processImuRaw(dataset, outDirPath, name=None, downsampleRatio=1):
    group = 'imu'
    names = {'imu-gyro-raw':'angular_velocity_raw', 'imu-accel-raw':'linear_acceleration_raw'}

    # Convert dictionary to list
    if name is not None:
        names = [names[name],]
    else:
        names = names.items()
        
    for name in names:
        [_, _, raw, clock, shape] = dataset.getStates(name, group)
        
        # Estimate sampling rate from clock
        fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
        logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
        windowSize = 2*fs

        outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
        logger.info('Writing to output video file %s' % (outputVideoFile))

        if name == 'angular_velocity_raw':
            title = 'Angular velocity (raw)'
            labels = ['Time [sec]', "Angular velocity [rad/s]"]
            ylim=[-2.0, 2.0]
        elif name == 'linear_acceleration_raw':
            title = 'Linear acceleration (raw)'
            labels = ['Time [sec]', "Linear acceleration [m/s^2]"]
            ylim=[-12.0, 12.0]
        legend = ['x-axis', 'y-axis', 'z-axis']
        
        exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
        
        
def processAudioSignal(dataset, outDirPath, name=None, fs=16000, tolerance=0.25, downsampleRatio=1.0):
    group = 'audio'
    names = {'audio-signal-left':'left', 'audio-signal-right':'right'}

    # Convert dictionary to list
    if name is not None:
        names = [names[name],]
    else:
        names = names.items()
        
    for name in names:
        [_, _, raw, clock, shape] = dataset.getStates(name, group)
        
        # Estimate sampling rate from clock
        fps = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
        logger.info('Estimated packet sampling rate of %d Hz for %s (group: %s)' % (fps, name, group))

        # Validate audio length
        audioLength = raw.shape[0] * raw.shape[1] / float(fs)
        referenceLength = clock[-1] + raw.shape[1] / float(fs)
        if not np.allclose(audioLength, referenceLength, atol=tolerance):
            logger.warn('Audio clock is incoherent: audio length is %f sec, but clock says %f sec' % (audioLength, referenceLength))

        raw = np.array(raw.flatten(), dtype=np.float32)[:,np.newaxis] / np.iinfo('int16').max
        windowSize = 2*fs
        downsampleRatio *= int(fs/5)

        outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
        logger.info('Writing to output video file %s' % (outputVideoFile))
        
        if name == 'left':
            title = 'Microphone (left)'
        elif name == 'right':
            title = 'Microphone (right)'
        
        labels = ['Time [sec]', "Ampitude"]
        ylim=[-1.0, 1.0]
        legend = None
        
        exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
        
def processTemperature(dataset, outDirPath, downsampleRatio=1.0):
    group = 'imu'
    name = 'temperature'

    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'IMU temperature'
    labels = ['Time [sec]', "Temperature [celcius]"]
    ylim=[15.0, 50.0]
    legend = None
    
    exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
        
def processOdometryTwistAngular(dataset, outDirPath, downsampleRatio):
    group = 'odometry'
    name = 'twist_angular'

    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'Odometry angular velocity'
    labels = ['Time [sec]', "Angular velocity [rad/s]"]
    ylim=[-2.0, 2.0]
    legend = ['x-axis', 'y-axis', 'z-axis']

    exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
        

def processOdometryTwistLinear(dataset, outDirPath, downsampleRatio):
    group = 'odometry'
    name = 'twist_linear'

    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'Odometry linear velocity'
    labels = ['Time [sec]', "Linear velocity [m/s]"]
    ylim=[-0.5, 0.5]
    legend = ['x-axis', 'y-axis', 'z-axis']

    exportSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend, downsampleRatio=downsampleRatio)
        
def processAudio(dataset, outDirPath, fs=16000, tolerance=0.25):
    group = 'audio'
    names = ['left', 'right']
    
    data = []
    for name in names:
        [_, _, raw, clock, shape] = dataset.getStates(name, group)

        # Estimate sampling rate from clock
        fps = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
        logger.info('Estimated packet sampling rate of %d Hz for %s (group: %s)' % (fps, name, group))

        # Validate audio length
        audioLength = raw.shape[0] * raw.shape[1] / float(fs)
        referenceLength = clock[-1] + raw.shape[1] / float(fs)
        if not np.allclose(audioLength, referenceLength, atol=tolerance):
            logger.warn('Audio clock is incoherent: audio length is %f sec, but clock says %f sec' % (audioLength, referenceLength))

        data.append(raw.flatten())
    data = np.array(data, dtype=np.int16)
    
    outputWavFile = os.path.abspath(os.path.join(outDirPath, '%s_left-right.wav' % (group)))
    logger.info('Writing to output audio file %s' % (outputWavFile))
    
    scipy.io.wavfile.write(outputWavFile, fs, data.T)
            
def processVideo(dataset, outDirPath):
    group = 'video'
    names = ['left', 'right']
    
    for name in names:
        [_, _, raw, clock, shape] = dataset.getStates(name, group)

        # Estimate sampling rate from clock
        fps = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
        logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fps, name, group))

        outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
        logger.info('Writing to output video file %s' % (outputVideoFile))

        # Decode first frame to check image size
        data = raw[0,:shape[0,0]]
        img = cv2.imdecode(data, flags=1) # cv2.CV_LOAD_IMAGE_COLOR
        height, width, layers =  img.shape

        # Initialize video writer based on image size
        if is_cv2():
            codec = cv2.cv.CV_FOURCC(*'XVID')
        elif is_cv3():
            codec = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(outputVideoFile, codec, fps, (width, height))
        
        # Process each frame
        startTime = time.time()
        nbFrames = len(raw)
        for i in range(nbFrames):
            data = raw[i,:shape[i,0]]
            
            # Decode raw JPEG data into image
            img = cv2.imdecode(data, flags=1) # cv2.CV_LOAD_IMAGE_COLOR
            
            # Write frame
            writer.write(img)
        
        # Close video writer
        writer.release()

        elapsedTime = time.time() - startTime
        logger.info('FPS = %f frame/sec' % (nbFrames/elapsedTime))

def process(name, datasetPath, outDirPath, downsampleRatio=1):
    with Hdf5Dataset(datasetPath, mode='r') as dataset:
        if name == 'position':
            processPosition(dataset, outDirPath, downsampleRatio)
        elif name == 'orientation':
            processOrientation(dataset, outDirPath, downsampleRatio)
        elif name == 'orientation-raw':
            processOrientation(dataset, outDirPath, downsampleRatio, rawValues=True)
        elif name == 'motors':
            processMotors(dataset, outDirPath, downsampleRatio)
        elif name == 'video':
            processVideo(dataset, outDirPath)
        elif name == 'imu-accel':
            processImu(dataset, outDirPath, name, downsampleRatio)
        elif name == 'imu-gyro':
            processImu(dataset, outDirPath, name, downsampleRatio)
        elif name == 'imu-mag':
            processImu(dataset, outDirPath, name, downsampleRatio)
        elif name == 'imu-accel-raw':
            processImuRaw(dataset, outDirPath, name, downsampleRatio)
        elif name == 'imu-gyro-raw':
            processImuRaw(dataset, outDirPath, name, downsampleRatio)
        elif name == 'range':
            processIRrange(dataset, outDirPath, downsampleRatio)
        elif name == 'contact':
            processContacts(dataset, outDirPath, name, downsampleRatio)
        elif name == 'wheel-drop':
            processContacts(dataset, outDirPath, name, downsampleRatio)
        elif name == 'cliff':
            processContacts(dataset, outDirPath, name, downsampleRatio)
        elif name == 'battery':
            processBattery(dataset, outDirPath, downsampleRatio)
        elif name == 'audio':
            processAudio(dataset, outDirPath)
        elif name == 'audio-signal-left':
            processAudioSignal(dataset, outDirPath, name)
        elif name == 'audio-signal-right':
            processAudioSignal(dataset, outDirPath, name)
        elif name == 'twist_linear':
            processOdometryTwistLinear(dataset, outDirPath, downsampleRatio)
        elif name == 'twist_angular':
            processOdometryTwistAngular(dataset, outDirPath, downsampleRatio)
        elif name == 'imu-temperature':
            processTemperature(dataset, outDirPath, downsampleRatio)
        else:
            raise Exception('Unknown name: %s' % (name))

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input hdf5 dataset file')
    parser.add_option("-o", "--output-dir", dest="outputDir", default='.',
                      help='specify the path of the output directory')
    parser.add_option("-c", "--nb-processes", dest="nbProcesses", type='int', default=1,
                      help='Specify the number of parallel processes to spawn')
    parser.add_option("-d", "--downsample-ratio", dest="downsampleRatio", type='int', default=1,
                      help='Specify the downsampling ratio')
    parser.add_option("-t", "--sensors", dest="sensors", default=None,
                      help='Specify the sensors from which to export videos')
    (options,args) = parser.parse_args(args=args)

    datasetPath = os.path.abspath(options.input)
    logger.info('Using input HDF5 dataset file: %s' % (datasetPath))

    outDirPath = os.path.abspath(options.outputDir)    
    logger.info('Using output directory: %s' % (outDirPath))
    
    if not os.path.exists(outDirPath):
        logger.info('Creating output directory: %s' % (outDirPath))
        os.makedirs(outDirPath)
    
    if options.nbProcesses <= 0:
        nbProcesses = multiprocessing.cpu_count()
    else:
        nbProcesses = options.nbProcesses
    
    logger.info('Using a multiprocessing pool of %d' % (nbProcesses))
    p = Pool(processes=nbProcesses, maxtasksperchild=1)
    
    logger.info('Using a downsampling ratio of %d' % (options.downsampleRatio))
    
    defaultNames = ['orientation-raw', 'imu-accel-raw', 'imu-gyro-raw', 'twist_linear', 'twist_angular','imu-temperature',
                    'position', 'audio-signal-left', 'audio-signal-right', 'orientation','battery',
                    'contact','cliff','wheel-drop','range', 'imu-accel', 'imu-gyro', 'imu-mag',
                    'motors', 'video', 'audio']
    
    if options.sensors is not None:
        names = str(options.sensors).split(',')
        for name in names:
            if name not in defaultNames:
                raise Exception('Unknown sensor name: %s' % (name))
    else:
        names = defaultNames
    logger.info('Exporting videos for those sensors: %s' % (','.join(names)))
    
    for name in names:
        process(name, datasetPath, outDirPath, options.downsampleRatio)
        #p.apply_async(process, args=(name, datasetPath, outDirPath, options.downsampleRatio))
    p.close()
    p.join()
    
    logger.info('All done.')
                
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
