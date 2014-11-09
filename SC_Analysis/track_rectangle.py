#!/usr/bin/env python

'''
Lucas-Kanade tracker
====================

Lucas-Kanade sparse optical flow demo. Uses goodFeaturesToTrack
for track initialization and back-tracking for match verification
between frames.

Usage
-----
lk_track.py [<video_source>]


Keys
----
ESC - exit
'''



'''
TODO:
1) See if KEEPING  points that are not initially in a joint improves performance
2) See if its necessary to actually remove points that are not moving in the mean direction
3) Try treating the center of the joint as the center of considered points (plus constants for new points
4) See if increasing trackLen yields greater performance (due to the fact that the current implementation for
    displacment is smoothed. See Joint.Update and App.AddToJoint. This would also require keeping points that are
    in joints, but don't have enough data to well determine if they are going in the "same" direction.
'''
import numpy as np
import cv2
import video
import math
from common import anorm2, draw_str
from time import clock

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

overlapFudge = 0.9
framesAnalyzedBeforeStart = 10
STDEV_FUDGE = 4

class Joint:
    def __init__(self, circle):
        self.circle = circle
        self.pointList = []
        self.meanDispHat = (1,0)
        self.meanDispPerpHat = (0,1)
        self.perpStdev = 0

    def Update(self):
        """
        Updates joint.
        :param joint: Joint object to be updated.
        """
        #Calculate mean and perpendicular vectors
        meanDispHat = (0,0)
        for point in self.pointList:
            dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
            meanDispHat = np.add(meanDispHat, dispVector)
        meanMag = np.linalg.norm(meanDispHat)
        meanDispHat = meanDispHat / meanMag
        perpHat = [-meanDispHat[1], meanDispHat[0]]# orthogonal vector

        # Find stdev of perpendicular component magnitude
        stdev = 0
        for point in self.pointList:
            dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
            stdev = stdev + math.pow(np.dot(perpHat, dispVector),2)# The mean perpendicular component is 0 by definition
        stdev = math.sqrt(stdev / len(self.pointList))
        newJointPoints = []

        # Remove points if the stdev is large
        if stdev > meanMag / STDEV_FUDGE:
            #Remove points that have significant perpendicular components and going in the direction of the mean vector
            # , and recalculate mean vector
            newMDP = (0,0)
            for point in self.pointList:
                dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
                if abs(np.dot(dispVector, perpHat)/stdev) < 2 and np.dot(dispVector, meanDispHat) > 0: # if perpendicular component is small
                    newJointPoints.append(point)
                    newMDP = np.add(newMDP, dispVector)
            meanMag = np.linalg.norm(newMDP)
            meanDispHat = newMDP / meanMag
            perpHat = [-meanDispHat[1], meanDispHat[0]]# orthogonal vector

            # Re-find stdev of perpendicular component magnitude
            stdev = 0
            for point in self.pointList:
                dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
                stdev = stdev + math.pow(np.dot(perpHat, dispVector),2)# The mean perpendicular component is 0 by definition
            stdev = math.sqrt(stdev / len(self.pointList))

        #Update all joint variables
        self.meanDispHat = meanDispHat
        self.meanDispPerpHat = perpHat
        if stdev > meanMag / STDEV_FUDGE: # If the standard deviation is large-ish relative to the main displacement vector
            self.perpStdev = stdev
        else:
            self.perpStdev = meanMag / STDEV_FUDGE
        self.pointList = newJointPoints
        self.circle[0] = np.add(self.circle[0], meanDispHat)

def InCircle(circle, point):
    """
    Returns true if the point is within the radius of the circle
    :param circle: list of len 2, as [(x,y), r]
    :param point: tuple as (x,y)
    :return: true or false if in circle or not
    """
    return np.linalg.norm(np.subtract(circle[0], point)) <= circle[1] * circle[1]

def UpdateTracks(tracks, img0, img1, track_len):
    """
    Updates all the point lists using new and old image data.
    :param tracks: List of lists of points, in increasing order of recentness
    :param img0: old image
    :param img1: new image
    :return: Updated list of points
    """
    if len(tracks) > 0:
        #convert old pointlist to matrix, taking the last element in each point tracked
        p0 = np.float32([tr[-1] for tr in tracks]).reshape(-1, 1, 2)
        #new pointlist from old points
        p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
        #old pointlist from new points
        p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
        #
        d = abs(p0-p0r).reshape(-1, 2).max(-1)
        good = d < 1
        new_tracks = []
        for tr, (x, y), good_flag in zip(tracks, p1.reshape(-1, 2), good):
            if not good_flag:
                continue
            tr.append((x, y))
            #eliminate track length size. I don't care about history, so make track len small
            if len(tr) > track_len:
                del tr[0]
            new_tracks.append(tr)
        return new_tracks

class App:
    def __init__(self, video_src, circles):
        self.track_len = 2
        self.detect_interval = 5
        self.roguePoints = []
        self.cam = video.create_capture(video_src)
        self.frame_idx = -1#This is because the frame index updates BEFORE anything is done.

        #Save frame to start at and the initial circles.
        self.allJoints = []
        self.circles = []
        self.pointsInCircles = []
        f = open(circles, 'r')
        self.initalFrame = int(f.readline())
        for line in f:
            temp = map(int, line.split())#x y r (with spaces)
            self.circles.append( [(temp[0], temp[1]) , temp[2]] )# circle in form [(x,y),r]
        f.close()

    #Add points that satisfy the condition of pointing in the direction of the mean vector of the joint, and having a minimal perpendicular component
    def AddToJoints(self):
        """
        Add points to joint that are within the radius of the joint circle and are "mostly" pointing in the direction the joint is going
        """
        for point in self.roguePoints:
            for joint in self.allJoints:
                if InCircle(joint.circle,point[-1]):# Within the point radius
                    dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
                    if np.dot(dispVector, joint.meanDispHat) > 0:# Pointing in same direction
                        if np.dot(dispVector, joint.meanDispPerpHat)/joint.perpStdev < 2:# Less than 2 stdevs away
                            self.allJoints.append(point)# Now add point
                            continue#Add point to only one joint
        self.roguePoints = []

    #return the first pair of joints that overlap, or zero if none exist
    def OverLappingJoints(self):
        for i in range(0,len(self.allJoints)):
            for j in range(0,i):
                #overlapFudge decreases radius
                if np.linag.norm(np.subtract(self.allJoints[i].circle[0], self.allJoints[j].circle[0])) < overlapFudge * max([self.allJoints[i].circle[1], self.allJoints[j].circle[1]]):
                    return [i, j]
        return 0

    def WriteData(self, outData):

        """
        Writes out the joint locations and frame number, in the form frame#, (x, y), (x2,y2)...(xn,yn),
        :param outData: File for writing to
        """
        outString = str(self.frame_idx) + ","
        for joint in self.allJoints:
            outString = outString + str(joint.circle[0]) + ","
        outData.write(outString + "\n")

    def run(self, outData):
        while True:
            # Read data
            try:
                ret, frame = self.cam.read()
            except:
                return 0
            self.frame_idx += 1
            #Only start doing calculations after the proper frame is reached
            if self.frame_idx >= self.initalFrame - framesAnalyzedBeforeStart:

                #Update current known rogue points
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                vis = frame.copy()
                img0, img1 = self.prev_gray, frame_gray# Get images
                if len(self.roguePoints > 0):
                    self.roguePoints = UpdateTracks(self.roguePoints,img0,img1, self.track_len)

                #Joint handling
                if self.frame_idx == self.initalFrame:
                    # Add initial good points to circles, "removing" from roguePoints
                    # Create Joints
                    for circle in self.circles:
                        self.allJoints.append(Joint(circle))
                    # Add points to joints
                    for onePoint in self.roguePoints:
                        added = False
                        for joint in self.allJoints:
                            #Get most recent point
                            if InCircle(joint.circle, onePoint[-1]):
                                if added: # If a point is in two joints
                                    raise Exception("Overlapping joints upon initialization.")
                                joint.pointList.append(onePoint[-1])
                    self.roguePoints = []# Throw out points that are not in joints
                else:
                    #Update all points in each segment
                    badJoints = []
                    for i in range(0,len(self.allJoints)):
                        #Update each joint point and remove as necessary
                        self.allJoints[i].pointList = UpdateTracks(self.allJoints[i].pointList,img0,img1, self.track_len)
                        self.allJoints[i].Update()
                        if len(self.allJoints[i].pointList) == 0:
                            badJoints.append(i)
                    if len(badJoints) != 0:# If there are joints with no points.
                        return [self.frame_idx, badJoints] #return frame number and joint numbers

                    #Add joints that are almost definitely in the joint
                    self.AddToJoints() # This makes roguePoints EMPTY!

                    #Return as error if there are significantly overlapping joints (This should never happen in a squat)
                    overLapping = self.OverLappingJoints()
                    if overLapping != 0:
                        return [self.frame_idx, overLapping]# return frame number and joint numbers

                #Repopulate roguePoints
                if self.frame_idx % self.detect_interval == 0 or self.frame_idx < self.initalFrame:
                    mask = np.zeros_like(frame_gray)
                    p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                    if p is not None:
                        for x, y in np.float32(p).reshape(-1, 2):
                            self.roguePoints.append([(x, y)])
                #save old image and write to file
                self.prev_gray = frame_gray
                self.WriteData(outData)
                #cv2.imshow('lk_track', vis)

def Test():
    circle = [(10,10),1]
    circle2 =  [(11,10),2]
    circles = [circle, circle2]
    point = (12,12)
    joint = Joint(circle)
    pointList = [[(10,10), (11,10)], [(10,10), (11,10)], [(10,10), (10,11)], [(10,10), (10,11)], [(10,10), (9,9)]]# Two points going in the same direction, two another direction, and one backwards
    joint.pointList = pointList
    joint.Update()
    print joint.pointList
    inCircleT = not InCircle(circle, point)
    print inCircleT
def main():
    import sys
    try: video_src = sys.argv[1]
    except: video_src = 0
    circles = sys.argv[2]
    print __doc__
    app = App(video_src, circles)
    outData = open('out.csv', 'w')
    exitValue = app.run(outData)
    outData.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    #main()
    Test()
    '''
    Test:
    Joint.Update
    InCircle
    UpdateTracks(ish)

    AddToJoints
    OverLappingJoints
    WriteData
    run
    '''
