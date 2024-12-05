import math

class TrapezoidalProfile:
    isCreated = False
    accTime = 0
    constTime = 0
    decTime = 0

    acc = 0
    dec = 0
    
    startVel = 0
    targetVel = 0
    endVel = 0
    
    accDis = 0
    constDis = 0
    decDis = 0
    totalDis = 0
    curDis = 0
    elapsedTime = 0

    profInfo = None

    def __init__ (self):
        self.isCreated = False

        self.accTime = 0
        self.constTime = 0
        self.decTime = 0
        
        self.acc = 0
        self.dec = 0

        self.startVel = 0
        self.targetVel = 0
        self.endVel = 0

        self.accDis = 0
        self.constDis = 0
        self.decDis = 0
        self.totalDis = 0
        self.curDis = 0
        self.elapsedTime = 0

        self.profInfo = None

    def setProfInfo(self, profInfo):
        self.profInfo = profInfo

    def getCurDis(self) -> float:
        return self.curDis

    def isDone(self) -> bool:
        if self.elapsedTime >= self.totalTime:
            return True
        else:
            return False

    def isDecelerating(self) -> bool:
        if (self.accTime + self.constTime) < self.elapsedTime < self.totalTime:
            return True
        else:
            return False

    def makeProf(self, acc: float, dec: float, vel: float, dis: float) -> bool:

        if acc <= 0 or dec <= 0 or vel <= 0 or dis <= 0:
            return False 

        self.acc = abs(acc)
        self.dec = abs(dec)
        self.targetVel = abs(vel)

        self.totalDis = abs(dis) 
        self.elapsedTime = 0
        self.accTime = self.targetVel / self.acc
        self.decTime = self.targetVel / self.dec        
        self.accDis = 0.5 * self.targetVel * self.accTime
        self.decDis = 0.5 * self.targetVel * self.decTime  
        self.constDis = self.totalDis - (self.accDis + self.decDis)
        
        if self.constDis < 0:
            tmp = 2. * (self.acc * self.dec) * self.totalDis / (self.acc + self.dec)
            if tmp > 0:
                self.targetVel = math.sqrt(tmp)
            else:
                return False
            self.accTime = self.targetVel / self.acc
            self.decTime = self.targetVel / self.dec        
            self.accDis = 0.5 * self.targetVel * self.accTime
            self.decDis = 0.5 * self.targetVel * self.decTime  
            self.constDis = 0
            
        self.constTime = self.constDis / self.targetVel
        self.accConstTime = self.accTime + self.constTime
        self.totalTime = self.accConstTime + self.decTime

        self.isCreated = True

        return self.isCreated 

    def calDis(self, cycleTime: float) -> bool:

        if not self.isCreated:
            return False

        self.elapsedTime += cycleTime
        self.curDis = 0

        if self.elapsedTime < self.accTime:
            self.curDis = 0.5 * self.elapsedTime * self.acc * self.elapsedTime

        elif self.elapsedTime < self.accConstTime:
            self.curDis = self.accDis + self.targetVel * (self.elapsedTime - self.accTime)
        else:            
            complementaryTime = self.totalTime - self.elapsedTime
            self.curDis = self.totalDis - 0.5 * complementaryTime * self.dec * complementaryTime

        if self.totalTime < self.elapsedTime:
            self.curDis = self.totalDis
            return False

        return True

