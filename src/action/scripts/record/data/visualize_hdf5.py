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

from h5utils import Hdf5Dataset
from optparse import OptionParser

logger = logging.getLogger(__name__)

# TODO: odometry-related: orientation, position, twist_angular, twist_linear
# TODO: collision-related: range sensor, switch sensors
# TODO: battery-related: charge, status


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

def exportQuaternionFramesAsVideo(frames, fs, filename, title, grid=False):

    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs, codec='libx264')
    
    fig = plt.figure(figsize=(5,4), facecolor='white', frameon=False)
    ax = fig.gca(projection='3d')
    fig.tight_layout()
    fig.subplots_adjust(left=0.20, bottom=0.15)
    
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
    
    # See: http://stackoverflow.com/questions/15042129/changing-position-of-vertical-z-axis-of-3d-plot-matplotlib
    tmp_planes = ax.zaxis._PLANES
    ax.zaxis._PLANES = ( tmp_planes[2], tmp_planes[3], 
                         tmp_planes[0], tmp_planes[1],
                         tmp_planes[4], tmp_planes[5])
    
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
        ax.add_artist(arrow)
    
    proxies = [plt.Rectangle((0, 0), 1, 1, fc=c) for c in colors]
    legend = ax.legend(proxies, labels, loc='upper right')
        
    def update(frame):
        quaternion = frame
        
        # Convert from quaternion (w, x, y, z) to rotation matrix
        R = quat2mat(quaternion)
        
        # Apply the rotation to the axis vectors (pointing in Y-axis)
        directions = np.eye(3) # x, y, z as column vectors
        vectors = np.dot(R, directions)
        assert np.allclose(np.linalg.norm(vectors, 2, axis=0), np.ones((3,)), atol=1e-6)
        
        # Update existing plot
        for i in range(3):
            x,y,z = vectors[:,i]
            arrows[i].set_data([0.0, x], [0.0, y], [0.0, z])
                
    ani = animation.FuncAnimation(fig, update, frames=frames, blit=True)
    ani.save(filename, dpi=100, writer=writer)
    plt.close(fig)
            
def export3DSensorFramesAsVideo(frames, fs, filename, title, labels, ylim=None, windowSize=None, grid=False, legend=['x-axis', 'y-axis', 'z-axis']):
            
    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs, codec='libx264')
    
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
    
    if ylim is not None:
        ax.axis([-windowSize/fs, 0.0, ylim[0], ylim[1]])
    
    xdata=-np.arange(windowSize)[::-1]/float(fs)
    ydata=np.zeros(windowSize)
    lines = []
    lines.append(ax.plot(xdata,ydata,'-r'))
    lines.append(ax.plot(xdata,ydata,'-g'))
    lines.append(ax.plot(xdata,ydata,'-b'))
        
    ax.legend(legend, loc='upper left')
        
    ymax = 0.0
    data = np.zeros((windowSize, 3))
    def update(frame):
        # Update buffer
        data[0:-1:,:] = data[1::,:]
        data[-1,:] = frame
        
        # Update existing line plots
        for i in range(3):
            ydata=np.array(data[:,i])
            lines[i][0].set_data(xdata, ydata)
            
        if ylim is None:
            cmax = np.max(np.abs(data))
            if cmax > ymax:
                ymax = cmax
            ax.axis([-windowSize/fs, 0.0, -ymax, ymax])
                
    ani = animation.FuncAnimation(fig, update, frames=frames, blit=True)
    ani.save(filename, dpi=100, writer=writer)
    plt.close(fig)

def export2DSensorFramesAsVideo(frames, fs, filename, title, labels, ylim=None, windowSize=None, grid=False, legend=['x-axis', 'y-axis']):
            
    # Create the video file writer
    writer = animation.FFMpegWriter(fps=fs, codec='libx264')
    
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
    
    if ylim is not None:
        ax.axis([-windowSize/fs, 0.0, ylim[0], ylim[1]])
    
    xdata=-np.arange(windowSize)[::-1]/float(fs)
    ydata=np.zeros(windowSize)
    lines = []
    lines.append(ax.plot(xdata,ydata,'-r'))
    lines.append(ax.plot(xdata,ydata,'-g'))
        
    ax.legend(legend, loc='upper left')
        
    ymax = 0.0
    data = np.zeros((windowSize, 2))
    def update(frame):
        # Update buffer
        data[0:-1:,:] = data[1::,:]
        data[-1,:] = frame
        
        # Update existing line plots
        for i in range(2):
            ydata=np.array(data[:,i])
            lines[i][0].set_data(xdata, ydata)
            
        if ylim is None:
            cmax = np.max(np.abs(data))
            if cmax > ymax:
                ymax = cmax
            ax.axis([-windowSize/fs, 0.0, -ymax, ymax])
                
    ani = animation.FuncAnimation(fig, update, frames=frames, blit=True)
    ani.save(filename, dpi=100, writer=writer)
    plt.close(fig)

def processPosition(dataset, outDirPath):
    group = 'odometry'
    name = 'position'
    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    # TODO: implement
            
def processOrientation(dataset, outDirPath):
    group = 'imu'
    name = 'orientation'
    [_, _, raw, clock, shape] = dataset.getStates(name, group)
    
    # Estimate sampling rate from clock
    fs = int(np.round(1.0/np.mean(clock[1:] - clock[:-1])))
    logger.info('Estimated sampling rate of %d Hz for %s (group: %s)' % (fs, name, group))
    windowSize = 2*fs

    outputVideoFile = os.path.abspath(os.path.join(outDirPath, '%s_%s.avi' % (group, name)))
    logger.info('Writing to output video file %s' % (outputVideoFile))

    title = 'Orientation'
    exportQuaternionFramesAsVideo(raw, fs, outputVideoFile, title, grid=False)
    
    
def processMotors(dataset, outDirPath):
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
    export2DSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False, legend=legend)
    
def processImu(dataset, outDirPath):
    group = 'imu'
    names = ['angular_velocity', 'linear_acceleration', 'magnetic_field']

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
        
        export3DSensorFramesAsVideo(raw, fs, outputVideoFile, title, labels, ylim, windowSize=int(2*fs), grid=False)
        
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
        data = raw[0,:shape[0]]
        img = cv2.imdecode(data, flags=1) # cv2.CV_LOAD_IMAGE_COLOR
        height, width, layers =  img.shape

        # Initialize video writer based on image size
        # codec = cv2.cv.CV_FOURCC(*'XVID')
        codec = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(outputVideoFile, codec, fps, (width, height))
        
        # Process each frame
        nbFrames = len(raw)
        for i in range(nbFrames):
            data = raw[i,:shape[i]]
            
            # Decode raw JPEG data into image
            img = cv2.imdecode(data, flags=1) # cv2.CV_LOAD_IMAGE_COLOR
            
            # Write frame
            writer.write(img)
        
        # Close video writer
        writer.release()

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input hdf5 dataset file')
    parser.add_option("-o", "--output-dir", dest="outputDir", default='.',
                      help='specify the path of the output directory')
    (options,args) = parser.parse_args(args=args)

    datasetPath = os.path.abspath(options.input)
    logger.info('Using input HDF5 dataset file: %s' % (datasetPath))

    outDirPath = os.path.abspath(options.outputDir)    
    logger.info('Using output directory: %s' % (outDirPath))
    
    if not os.path.exists(outDirPath):
        logger.info('Creating output directory: %s' % (outDirPath))
        os.makedirs(outDirPath)
    
    with Hdf5Dataset(datasetPath, mode='r') as dataset:
        
        processPosition(dataset, outDirPath)
        
        processOrientation(dataset, outDirPath)
        
        processMotors(dataset, outDirPath)
        
        processVideo(dataset, outDirPath)
        
        processImu(dataset, outDirPath)

        processAudio(dataset, outDirPath)
                
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
