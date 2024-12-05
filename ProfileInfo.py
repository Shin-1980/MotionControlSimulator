class ProfileInfo:
    targetPose = []
    targetVels = []
    targetAccs = []
    targetDecs = []

    startPose = []
    
    baseCoordinate = 0

    def __init__ (self):
        self.targetPose = []
        self.targetVels = []
        self.targetAccs = []
        self.targetDecs = []

        self.startPose = []
        
        self.baseCoordinate = 0

    def setParam(self, targetPose: list, targetVels: list, targetAccs: list, targetDecs: list):
        self.targetPose = targetPose
        self.targetVels = targetVels
        self.targetAccs = targetAccs
        self.targetDecs = targetDecs

    def setStartPose(self, startPose):
        self.startPose = startPose

    def setBaseCoordinate(self, base):
        self.baseCoordinate = base

    def getTargetPose(self):
        return self.targetPose

    def getTargetVels(self):
        return self.targetVels

    def getTargetAccs(self):
        return self.targetAccs    

    def getTargetDecs(self):
        return self.targetDecs  

    def getStartPose(self):
        return self.startPose

    def getBaseCoordinate(self):
        return self.baseCoordinate

    def getUnsignedTotalDistance(self):
        return abs(self.targetPose - self.startPose)

    def getSignedTotalDistance(self):
        return self.targetPose - self.startPose
