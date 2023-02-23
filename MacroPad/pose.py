import math
robotY = 10
robotX = 10
robotAngle = -90
coneXToTheRobot = 5
coneYToTheRobot = 5
coneAngle = 0
hyp = math.sqrt(coneXToTheRobot * coneXToTheRobot + coneYToTheRobot * coneYToTheRobot)
print(hyp)
for robotAngle in range(-180,181,5):
    robotAngle = math.radians(robotAngle)
    deltaAngle = math.atan2(coneYToTheRobot, coneXToTheRobot)
    deltaAngle = robotAngle
    goalX = robotX + coneXToTheRobot * math.cos(deltaAngle) - coneYToTheRobot * math.sin(deltaAngle)
    goalY = robotY + coneXToTheRobot * math.sin(deltaAngle ) + coneYToTheRobot * math.cos(deltaAngle)
    goalAngle = robotAngle - coneAngle
    #print("Atan2:%.2f Robot Angle:%.2f Sin Robot: %.2f Cos Robot:%.2f" % (math.degrees(deltaAngle), math.degrees(robotAngle), math.sin(robotAngle), math.cos(robotAngle)))
    print("Goal X:%.2f Y:%.2f Angle:%.2f" % (goalX, goalY, math.degrees(goalAngle)))
 