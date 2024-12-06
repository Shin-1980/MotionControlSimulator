import math
import numpy as np
import csv
import pandas as pd

from ProfileController import ProfileController
from TrapezoidalProfile import TrapezoidalProfile
from PUMA560 import PUMA560
    
class TestCase:

    def case1(self):
        prof = TrapezoidalProfile()

        startPos = 0.0
        tarPos = 50
        acc = 100
        dec = 100
        vel = 100
        cycleTime = 0.1
        prePos = startPos
        curPos = startPos
        preVel = 0
        curVel = 0
 
        filename = "./testCase/case1.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0

        if prof.makeProf(acc,dec,vel,tarPos - startPos):

            while prof.calDis(cycleTime):
                #st = str(prof.getCurDis()) + "\n"
                #fw.write(st)

                #print(prof.getCurDis(), float(df.iloc[idx]))
                if abs(float(df.iloc[idx].iloc[0]) - float(prof.getCurDis())) > 0.00001:
                    return False
                idx += 1

        return True
        #fw.close()

    def case2(self):
        prof = TrapezoidalProfile()

        startPos = 0.0
        tarPos = 25
        acc = 100
        dec = 100
        vel = 100
        cycleTime = 0.1
        prePos = startPos
        curPos = startPos
        preVel = 0
        curVel = 0
 
        filename = "./testCase/case2.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0

        if prof.makeProf(acc,dec,vel,tarPos - startPos):

            while prof.calDis(cycleTime):
                #st = str(prof.getCurDis()) + "\n"
                #fw.write(st)
                 
                #print(prof.getCurDis(), float(df.iloc[idx]))
                if abs(float(df.iloc[idx].iloc[0]) - float(prof.getCurDis())) > 0.00001:
                    return False
                idx += 1

        return True
        
        #fw.close()

    def case3(self):
        prof = TrapezoidalProfile()

        cycleTime = 0.01
        if prof.calDis(cycleTime) == False:
            return True
        
        return False

    def case4(self):
        prof = TrapezoidalProfile()

        if prof.getCurDis() == 0:
            return True        

    def case5(self):
        prof = TrapezoidalProfile()

        if prof.getCurDis() == None:
            return True        

        startPos = 0.0
        tarPos = -25
        acc = 100
        dec = 100
        vel = 100
        cycleTime = 0.1
        prePos = startPos
        curPos = startPos
        preVel = 0
        curVel = 0
 
        if prof.makeProf(acc,dec,vel,tarPos - startPos) == False:
            return True

        return False

    def case6(self):
        prof = TrapezoidalProfile()

        startPos = 0.0
        tarPos = 25
        acc = -100
        dec = 100
        vel = 100
        cycleTime = 0.1
        prePos = startPos
        curPos = startPos
        preVel = 0
        curVel = 0
 
        if prof.makeProf(acc,dec,vel,tarPos - startPos) == False:
            return True

        return False

    def case7(self):
        prof = TrapezoidalProfile()

        startPos = 0.0
        tarPos = 25
        acc = 100
        dec = -100
        vel = 100
        cycleTime = 0.1
        prePos = startPos
        curPos = startPos
        preVel = 0
        curVel = 0
 
        if prof.makeProf(acc,dec,vel,tarPos - startPos) == False:
            return True

        return False

    def case8(self):
        prof = TrapezoidalProfile()

        startPos = 0.0
        tarPos = 25
        acc = 100
        dec = 100
        vel = -100
        cycleTime = 0.1
        prePos = startPos
        curPos = startPos
        preVel = 0
        curVel = 0
 
        if prof.makeProf(acc,dec,vel,tarPos - startPos) == False:
            return True

        return False


    def case101(self):
        
        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 2000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        
        cycleTime = 0.01

        filename = "./testCase/case101.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0

        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            #st = str(cmdPose[0]) + "," + str(cmdPose[1]) + "," + str(cmdPose[2]) + "," + str(cmdPose[3]) + "," + str(cmdPose[4]) + "," + str(cmdPose[5]) + "\n"
            #fw.write(st)

            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False
            
            idx += 1

        # last cycle
        cmdPose = proCtr.getCmdPose()
        for axis in range(dof):
            if abs(targetPose[axis] - cmdPose[axis]) > 0.00001:
                return False

        return True

    def case102(self):
        
        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180 / 10
        #print("vel",targetVelsRad)
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 2000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)
        #print("acc",targetAccsRad)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        
        cycleTime = 0.01

        filename = "./testCase/case102.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0

        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            #st = str(cmdPose[0]) + "," + str(cmdPose[1]) + "," + str(cmdPose[2]) + "," + str(cmdPose[3]) + "," + str(cmdPose[4]) + "," + str(cmdPose[5]) + "\n"
            #fw.write(st)

            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False
            
            idx += 1

        # last cycle
        cmdPose = proCtr.getCmdPose()
        for axis in range(dof):
            if abs(targetPose[axis] - cmdPose[axis]) > 0.00001:
                return False

        return True

    def case103(self):
        proCtr = ProfileController()

        cycleTime = 0.01
        if proCtr.execCmd(cycleTime) == False:
            return True

    def case104(self):

        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        #print("vel",targetVelsRad)
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 2000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)
        #print("acc",targetAccsRad)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        
        cycleTime = 0.01

        filename = "./testCase/case101.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False
            idx += 1

        targetPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        
        idx = 0
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False 
            idx += 1
        # last cycle
        cmdPose = proCtr.getCmdPose()
        for axis in range(dof):
            if abs(targetPose[axis] - cmdPose[axis]) > 0.00001:
                return False

        return True

    def case105(self):

        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,-600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 2000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        
        cycleTime = 0.01

        if proCtr.execCmd(cycleTime) == False:
            return True

        return False

    def case106(self):

        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, -1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)
        
        cycleTime = 0.01

        if proCtr.execCmd(cycleTime) == False:
            return True

        return False

    def case107(self):

        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        targetPose = np.array([0.456344, -2.54862, 2.16375, 1.78586, -0.350709, -1.29741])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        cycleTime = 0.01
        filename = "./testCase/case107.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False 
            idx += 1
        # last cycle
        cmdPose = proCtr.getCmdPose()
        for axis in range(dof):
            if abs(targetPose[axis] - cmdPose[axis]) > 0.00001:
                return False

        return True

    def case108(self):

        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        targetPose = np.array([0.70, -2.37, 2.39, 2.10, -1.08, -1.15])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        cycleTime = 0.01
        filename = "./testCase/case108.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False 
            idx += 1
        # last cycle
        cmdPose = proCtr.getCmdPose()
        for axis in range(dof):
            if abs(targetPose[axis] - cmdPose[axis]) > 0.00001:
                return False

        return True

    def case109(self):

        proCtr = ProfileController()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        targetPose = np.array([0.456344, -2.54862, 2.16375, 1.78586, -0.350709, -1.29741])
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        targetPose = np.array([-0.449, -0.890494, 2.60354, 2.06081, -0.0605401, -0.624552])
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        cycleTime = 0.01
        filename = "./testCase/case109.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            for axis in range(dof):
                if abs(float(df.iloc[idx][axis]) - cmdPose[axis]) > 0.00001:
                    return False 
            idx += 1
        # last cycle
        cmdPose = proCtr.getCmdPose()
        for axis in range(dof):
            if abs(targetPose[axis] - cmdPose[axis]) > 0.00001:
                return False

        return True

    def case110(self):

        proCtr = ProfileController()
        puma560 = PUMA560()

        curPose = np.array([1.5708, -3.14159, 1.5708, 0, 0, 0])
        dof = 6
        proCtr.setCurrentPose(curPose,dof)

        targetPose = np.array([0.693782, -2.36364, 2.38406, 2.1103, -1.07417, -1.14081])
        targetVelsDeg = np.array([300,300,375,375,375,600])
        targetVelsRad = targetVelsDeg * math.pi / 180
        targetAccsDeg = np.array([1000,1000,1500,1500,1500, 1000])
        targetAccsRad = np.array(targetAccsDeg * math.pi / 180)

        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        targetPose = np.array([0.456344, -2.54862, 2.16375, 1.78586, -0.350709, -1.29741])
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        targetPose = np.array([-0.449, -0.890494, 2.60354, 2.06081, -0.0605401, -0.624552])
        proCtr.setCmd(targetPose, targetVelsRad, targetAccsRad, targetAccsRad)

        cycleTime = 0.01

        initial_pose = proCtr.getCmdPose()
        pretcp = puma560.forward_kinematics(initial_pose)
        pretcv = pretcp - pretcp

        filename = "./testCase/case110.csv"
        #fw = open(filename, 'w')
        df = pd.read_csv(filename, header=None)

        idx = 0
        while proCtr.execCmd(cycleTime):
            cmdPose = proCtr.getCmdPose()
            cmdVels = proCtr.getCmdVels()
            tcp = puma560.forward_kinematics(cmdPose)
            tcv = (tcp - pretcp) / cycleTime
            tca = (tcv - pretcv) / cycleTime
            pretcp = tcp
            pretcv = tcv

            for axis in range(3):
                if abs(float(df.iloc[idx][axis]) - tcp[axis]) > 0.00001:
                    return False 
            idx += 1

        # last cycle
        cmdPose = proCtr.getCmdPose()
        cmdVels = proCtr.getCmdVels()
        tcp = puma560.forward_kinematics(cmdPose)
        tcv = (tcp - pretcp) / cycleTime
        tca = (tcv - pretcv) / cycleTime
        pretcp = tcp
        pretcv = tcv
    
        return True


tc = TestCase()


testCase = 0

testCase += 1
if tc.case1():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case2():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case3():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case4():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case5():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case6():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case7():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case8():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase = 101
if tc.case101():
    print("PATH", testCase)
else:
    print("ERROR", testCase)


testCase += 1
if tc.case102():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case103():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case104():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case105():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case106():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case107():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case108():
    print("PATH", testCase)
else:
    print("ERROR", testCase)

testCase += 1
if tc.case109():
    print("PATH", testCase)
else:
    print("ERROR", testCase)


testCase += 1
if tc.case110():
    print("PATH", testCase)
else:
    print("ERROR", testCase)
