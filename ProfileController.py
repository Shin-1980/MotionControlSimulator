import numpy as np
from ProfileInfo import ProfileInfo
from TrapezoidalProfile import TrapezoidalProfile

class ProfileController:

    profileQueue = []

    curPose = []
    curVels = []
    prePose = []
    preVels = []
    curAccs = []
    execProf = None
    imposedProf = None

    def __init__ (self):
        self.profileQueue = []

        self.curPose = []
        self.curVels = []
        self.prePose = []
        self.preVels = []
        self.curAccs = []
        self.execProf = None
        self.imposedProf = None
        self.dof = 0

        self.basePose = []
        self.totalDis = []

    def setCurrentPose(self, curPose: list, dof: int):
        self.dof = dof
        self.curPose = curPose
        self.curVels = [0 for _ in range(dof)]
        self.prePose = curPose
        self.preVels = [0 for _ in range(dof)]

    def setCmd(self, targetPose: list, targetVels: list, targetAccs: list, targetDecs: list):
        
        pi = ProfileInfo()
        pi.setParam(targetPose, targetVels, targetAccs, targetDecs)

        prof = TrapezoidalProfile()
        prof.setProfInfo(pi)

        self.profileQueue.append(prof)

    def getCmdPose(self) -> list:
        return self.curPose

    def getCmdVels(self) -> list:
        return self.curVels

    def getCmdAccs(self) -> list:
        return self.curAccs
    
    def execCmd(self, cycleTime: float) -> bool:

        def makeLinearProf(prof: TrapezoidalProfile, startPose: list) -> bool:
                            
            if prof == None or prof.profInfo == None:
                return false

            profInfo = prof.profInfo
            profInfo.setStartPose(startPose)

            times = profInfo.getUnsignedTotalDistance()  / profInfo.getTargetVels()

            maxTime = 0

            for i in range(len(times)):
                if maxTime < times[i]:
                    maxTime = times[i]
                    profInfo.setBaseCoordinate(i)

            accs = profInfo.getTargetAccs()
            decs = profInfo.getTargetDecs()
            vels = profInfo.getTargetVels()
            diss = profInfo.getUnsignedTotalDistance()

            for i in range(len(accs)):
                if accs[i] < 0 or decs[i] < 0 or vels[i] < 0:
                    return False

            acc = accs[profInfo.getBaseCoordinate()]
            dec = decs[profInfo.getBaseCoordinate()]
            vel = vels[profInfo.getBaseCoordinate()]
            dis = diss[profInfo.getBaseCoordinate()]

            res = prof.makeProf(acc, dec, vel, dis)

            return res

        def isEnableToExecImposedProf(execProf: TrapezoidalProfile, imposedProf: TrapezoidalProfile) -> bool:

            if imposedProf != None and execProf.isDecelerating():
                if execProf.decTime < imposedProf.totalTime:
                    return True
                else:
                    if execProf.elapsedTime > (execProf.totalTime - imposedProf.accTime):
                        return True
            return False

        if self.execProf == None:
            if len(self.profileQueue) == 0:
                return False
            else:
                self.execProf = self.profileQueue.pop(0)
                if makeLinearProf(self.execProf, self.curPose) == False:
                    return False

        if self.imposedProf == None:
            if len(self.profileQueue) > 0:
                self.imposedProf = self.profileQueue.pop(0)
                #print("make imposed prof")
                if makeLinearProf(self.imposedProf, self.execProf.profInfo.getTargetPose()) == False:
                    return False

        if self.execProf.calDis(cycleTime):
            curDis = self.execProf.getCurDis()
            baseCoordinate = self.execProf.profInfo.getBaseCoordinate()
            totalDis = self.execProf.profInfo.getUnsignedTotalDistance()
            rate = curDis / totalDis[baseCoordinate]

            self.curPose = self.execProf.profInfo.getStartPose() + self.execProf.profInfo.getSignedTotalDistance() * rate

            #print(self.imposedProf, self.execProf.isDecelerating())

            if isEnableToExecImposedProf(self.execProf, self.imposedProf):
                if self.imposedProf.calDis(cycleTime):
                    #print("calc imposed prof")
                    ipDis = self.imposedProf.getCurDis()
                    ipBaseCoordinate = self.imposedProf.profInfo.getBaseCoordinate()
                    ipTotalDis = self.imposedProf.profInfo.getUnsignedTotalDistance()
                    ipRate = ipDis / ipTotalDis[ipBaseCoordinate]        
                    self.curPose += self.imposedProf.profInfo.getSignedTotalDistance() * ipRate

            self.curVels = (self.curPose - self.prePose) / cycleTime
            self.curAccs = (self.curVels - self.preVels) / cycleTime

            self.prePose = self.curPose
            self.preVels = self.curVels


        if self.execProf != None and self.execProf.isDone():
            if self.imposedProf == None:
                self.curVels = [0 for _ in range(self.dof)]
                self.preVel = [0 for _ in range(self.dof)]
                self.curPose = self.execProf.profInfo.getTargetPose()
                self.execProf = None
                return False
            else:
                self.execProf = self.imposedProf
                self.imposedProf = None

        return True

