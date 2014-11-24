#!/usr/bin/env python

''' TODO:
1) See if KEEPING  points that are not initially in a joint improves performance
2) See if its necessary to actually remove points that are not moving in the mean direction
3) Try treating the center of the joint as the center of considered points (plus constants for new points
4) See if increasing trackLen yields greater performance (due to the fact that the current implementation for
    displacment is smoothed. See Joint.Update and App.AddToJoint. This would also require keeping points that are
    in joints, but don't have enough data to well determine if they are going in the "same" direction.

Note: centerMatrix i > j and i corresponds to left point
'''


# region Parameters and Imports
import numpy as np
import cv2
import video
import math
from common import anorm2, draw_str
from time import clock

lk_params = dict( winSize = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500, qualityLevel = 0.3, minDistance = 7, blockSize = 7 )

OVERLAP_FUDGE = 0.9
FRAMES_ANAL_BEFORE_START = 10
STDEV_FUDGE = 4
MAX_JOINT_POINTS = 5
# endregion

# region Helper Methods
def InCircle(circle, point):
    """
    Returns true if the point is within the radius of the circle
    :param circle: list of len 2, as [(x,y), r]
    :param point: tuple as (x,y)
    :return: true or false if in circle or not
    """
    return np.linalg.norm(np.subtract(circle[0], point)) <= circle[1] * circle[1]

def CalceMeanVec(pointList):
    meanDispHat = (0,0)
    for point in pointList:
        dispVector = np.divide(np.subtract(point[-1], point[0]),(len(point)-1))
        meanDispHat = np.add(meanDispHat, dispVector)
    meanMag = np.linalg.norm(meanDispHat)
    meanDispHat = meanDispHat / meanMag
    perpHat = [-meanDispHat[1], meanDispHat[0]]# orthogonal vector
    return (meanDispHat, meanMag, perpHat)

def CalcStdev(pointList, perpHat):
    """

    :param pointList:
    :param perpHat:
    :return:
    """
    stdev = 0
    for point in pointList:
        dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
        stdev = stdev + math.pow(np.dot(perpHat, dispVector),2)# The mean perpendicular component is 0 by definition
    stdev = math.sqrt(stdev / len(pointList))
    return stdev

def RelativePoint(pointL, pointR, point):
    pointVecHat = np.subtract(pointR,pointL)
    pointMag = np.linalg.norm(pointVecHat)
    pointVecHat = np.divide(pointVecHat, pointMag)
    perpVecHat = (-pointVecHat[0], pointVecHat[1])

    relativeCenter = np.subtract(pointL, point) #Check this

    centerx = (np.dot(pointVecHat, relativeCenter) + pointL[0])
    centery = (np.dot(perpVecHat, relativeCenter) + pointL[1])
    return ((centerx, centery), pointMag)

def InverseRelativePoint(pointL, pointR, localCenter, originalMag):
    pointVecHat = np.subtract(pointR,pointL)
    pointMag = np.linalg.norm(pointVecHat)
    pointVecHat = np.divide(pointVecHat, pointMag)
    perpVecHat = (-pointVecHat[0], pointVecHat[1])

    localCenter = np.multiply(localCenter, pointMag/originalMag)
    xval = np.dot(localCenter, pointVecHat) * pointVecHat[0] +np.dot(localCenter, perpVecHat) * perpVecHat[0]
    yval = np.dot(localCenter, pointVecHat) * pointVecHat[1] +np.dot(localCenter, perpVecHat) * perpVecHat[1]
    return np.add(pointL, (xval, yval))

def Calibrate(indicies, retCenter, pointlist, centerMatrix):
    for pair in indicies:
        centerMatrix[pair[0]][pair[1]] = RelativePoint(pointlist[pair[1]][-1], pointlist[pair[0]][-1], retCenter)

def CalcCenterMeanStdev(pointList, centerMatrix):
        """

        :param pointList:
        :param centerMatrix:
        :return:
        """
        retCenter = (0,0)
        centers = []
        centerInds = []
        retStdev = 0
        # Calculate mean center
        for i in range(0, len(pointList)):
            pointR = pointList[i][-1]
            for j in range(0,i):
                pointL = pointList[j][-1]
                center = InverseRelativePoint(pointL, pointR, centerMatrix[i][j][0], centerMatrix[i][j][1])
                retCenter = np.add(retCenter, center)
                centers.append(center)
                centerInds.append((i,j))

        # Average RMS deviaiton from center
        for center in centers:
            retStdev = retStdev + np.linalg.norm(np.subtract(retCenter, center))
        retStdev = retStdev / math.sqrt(len(centers))
        #If a point is past 2 stdevs, remove point and recalibrate.
        oldRetCenter = retCenter
        indicies = []
        for i in range(0,len(centers)):
            if np.linalg.norm(np.subtract(centers[i] - oldRetCenter)) < 2 * max(retStdev, 1):
                retCenter = np.subtract(retCenter, centers[i])
                indicies.append(i)
                print 'x'
        retCenter = np.divide(retCenter, len(centers-len(indicies)))
        for i in range(0,len(indicies)):
            Calibrate(centerInds[indicies], retCenter, pointList, centerMatrix)
        return retCenter

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

def Initialize(circles, roguePoints):
    allJoints = []
    # Create Joints
    for circle in circles:
        allJoints.append(Joint(circle))
    # Add points to joints
    for pointR in roguePoints:
        added = False
        for joint in allJoints:
            #Get most recent point
            if InCircle(joint.circle, pointR[-1]):
                if added: # If a point is in two joints
                    raise Exception("Overlapping joints upon initialization.")
                matCol = []
                for pointL in joint.pointList:
                    # Find center in local coordinate system
                    matCol.append(RelativePoint(pointL, pointR, joint.circle[0]))
                joint.pointList.append(pointR)
                joint.centerMatrix.append(matCol)
    # Throw out points that are not in joints
    roguePoints[:] = []#Not sure if this works
    return allJoints

def AddToJoints(roguePoints, allJoints):
    """
    Add points to joint that are within the radius of the joint circle and are "mostly" pointing in the direction the joint is going
    """
    for point in roguePoints:
        for joint in allJoints:
            if InCircle(joint.circle,point[-1]):# Within the point radius
                dispVector = np.subtract(point[-1], point[0])/(len(point)-1)
                if np.dot(dispVector, joint.meanDispHat) > 0:# Pointing in same direction
                    if np.dot(dispVector, joint.meanDispPerpHat)/joint.perpStdev < 2:# Less than 2 stdevs away
                        allJoints.append(point)# Now add point
                        continue#Add point to only one joint
    roguePoints[:] = []

def WriteData(frame_idx, allJoints, outData):

    """
    Writes out the joint locations and frame number, in the form frame#, (x, y), (x2,y2)...(xn,yn),
    :param outData: File for writing to
    """
    outString = str(frame_idx) + ","
    for joint in allJoints:
        outString = outString + str(joint.circle[0]) + ","
    outData.write(outString + "\n")

def OverLappingJoints(allJoints):
    for i in range(0,len(allJoints)):
        for j in range(0,i):
            #OVERLAP_FUDGE decreases radius
            if np.linag.norm(np.subtract(allJoints[i].circle[0], allJoints[j].circle[0])) < OVERLAP_FUDGE * max([allJoints[i].circle[1], allJoints[j].circle[1]]):
                return [i, j]
    return 0
# endregion

# region Classes
class Joint:
    def __init__(self, circle):
        self.circle = circle
        self.pointList = []
        self.centerMatrix = []
        self.meanDispHat = (1,0)
        self.meanMag = -1
        self.meanDispPerpHat = (0,1)
        self.perpStdev = 0

    def Update(self):
        """
        Updates joint.
        :param joint: Joint object to be updated.
        """

        #Calculate mean and perpendicular vectors
        self.meanDispHat, self.meanMag, self.perpHat = CalceMeanVec(self.pointList)

        # Find stdev of perpendicular component magnitude
        self.perpStdev = CalcStdev(self.pointList, self.perpHat)
        self.circle[0] = CalcCenterMeanStdev(self.pointList, self.centerMatrix)
        #self.circle[0] = np.add(self.circle[0], meanDispHat)

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



    def run(self, outData):
        while True:
            # Read data
            try:
                ret, frame = self.cam.read()
            except:
                return 0
            self.frame_idx += 1
            #Only start doing calculations after the proper frame is reached
            if self.frame_idx >= self.initalFrame - FRAMES_ANAL_BEFORE_START:

                #Update current known rogue points
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                vis = frame.copy()
                img0, img1 = self.prev_gray, frame_gray# Get images
                if len(self.roguePoints > 0):
                    self.roguePoints = UpdateTracks(self.roguePoints,img0,img1, self.track_len)

                #Joint handling
                if self.frame_idx == self.initalFrame:
                    # Add initial good points to circles, "removing" from roguePoints
                    self.allJoints = Initialize(self.circles,self.roguePoints)
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
                    self.AddToJoints(self.roguePoints, self.allJoints) # This makes roguePoints EMPTY!

                    #Return as error if there are significantly overlapping joints (This should never happen in a squat)
                    overLapping = OverLappingJoints(self.allJoints)
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
# endregion

# region Testing
def InCircleT():
    circle = [(1,1),1]
    point = (1.5,1)
    return "InCircleT Passed: " + str(InCircle(circle,point))
def RelativePointAndInvT():
    pointL, pointR, point = ((0,1),(0,3),(1,2))
    relP, relMag = RelativePoint(pointL, pointR, point)
    print "RelativePointT Passed: " + relP
    ivrp = InverseRelativePoint(pointL, pointR, relP, relMag)
    print "InverseRelativePointT Passed " + ivrp
    print 'Write better tests.'
def InitTCalcCentMeanStdevT():
    circles = [[(1,1),2], [(5,5),1]]
    roguePoints = [[(1,1),(0,0)], [(2,2),(1,1)]]
    joints = Initialize(circles, roguePoints)
    print "InitializeT Passed: " + len(joints[0].pointList) + " " + len(joints[1].pointList)
    retCenter = CalcCenterMeanStdev(joints[0].pointList, joints[0].centerMatrix)
    print "CalcCenterMeanStdevT Passed:" + retCenter
def WriteDataT():
    import StringIO
    output = StringIO.StringIO()
    frame_idx = 0
    joint = Joint()
    joint.circle = [(0,0), 1]
    joints = [joint, joint]
    WriteData(frame_idx, joints, output)
    print "WriteDataT Passed: " + output.getvalue()
    output.close()
def JointUpdateT():
    circles = [[(1,1),2], [(5,5),1]]
    roguePoints = [[(1,1),(0,0)], [(2,2),(1,1)], [(2.1,2),(1,1)], [(2,2),(1,-10)]]
    joints = Initialize(circles, roguePoints)
    joints[0].Update()
    print "Joint.Update Passed: " + len(joints[0].pointList)
def CalcMeanVecT():
    return "CalcmeanVecT test not implemented yet."
def CalcStdevT():
    return "CalcStdevT test not implemented yet."
def AddToJointsT():
    print "AddToJointsT not implemented yet."
def Test():
    InCircleT()
    RelativePointAndInvT()
    InitTCalcCentMeanStdevT()
    WriteDataT()
    JointUpdateT()
    CalcMeanVecT()
    CalcStdevT()
    AddToJointsT()
# endregion

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