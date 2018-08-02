from dvrk import psm
import PyKDL
import numpy as np

def rotationFromVector(vectorDesired):
    ''' Find a PyKDL rotation from a vector using taylor series expansion
    '''
    vector = vectorDesired / vectorDesired.Norm()
    # Cross product of vectorDesired with z vector
    v = PyKDL.Vector(-vector.y(), vector.x(), 0)
    s = v.Norm()

    if s == 0:
        retval = PyKDL.Rotation(vector.z(), 0, 0,
                                0, 1, 0,
                                0, 0,vector.z())
        retval.DoRotZ(np.pi/2)
        return retval


    skew = np.matrix([[   0.0, -v.z(),  v.y()],
                      [ v.z(),    0.0, -v.x()],
                      [-v.y(), v.x(),    0.0]])
    c = vector.z()
    R = np.eye(3) + skew + skew*skew*(1-c)/(s*s);

    kdlRotation = arrayToPyKDLRotation(R.tolist())
    z, y  = kdlRotation.GetEulerZYZ()[0:2]
    print(z,y)
    retval = PyKDL.Rotation()
    retval.DoRotZ(z)
    retval.DoRotY(y)
    retval = PyKDL.Rotation.Rot(retval.UnitZ(), np.pi/2) * retval
    return retval

def resolvedRates(config,currentPose,desiredPose):
    # compute pose error (result in kdl.twist format)
    poseError = PyKDL.diff(currentPose,desiredPose)
    posErrNorm = poseError.vel.Norm()
    rotErrNorm = poseError.rot.Norm()

    angVelMag = config['angVelMax']
    if rotErrNorm < config['tolRot']:
        angVelMag = 0.0
    elif rotErrNorm < angVelMag:
        angVelMag = rotErrNorm


    # compute velocity magnitude based on position error norm
    if posErrNorm>config['tolPos']:
        tolPosition = config['tolPos']
        lambdaVel = config['velRatio']
        if posErrNorm>(lambdaVel*tolPosition):
            velMag = config['velMax']
        else:
            velMax = config['velMax']
            velMin = config['velMin']
            velMag = velMin + (posErrNorm - tolPosition) * \
                     (velMax - velMin)/(tolPosition*(lambdaVel-1))
    else:
        velMag = 0.0
    # compute angular velocity based on rotation error norm
    if rotErrNorm>config['tolRot']:
        tolRotation = config['tolRot']
        lambdaRot = config['rotRatio']
        if rotErrNorm>(lambdaRot*tolRotation):
            angVelMag = config['angVelMax']
        else:
            angVelMax = config['angVelMax']
            angVelMin = config['angVelMin']
            angVelMag = angVelMin + (rotErrNorm - tolRotation) * \
                        (angVelMax - angVelMin)/(tolRotation*(lambdaRot-1))
    else:
        angVelMag = 0.0
    # The resolved rates is implemented as Nabil Simaan's notes
    # apply both the velocity and angular velocity in the error pose direction
    desiredTwist = PyKDL.Twist()
    poseError.vel.Normalize() # normalize to have the velocity direction
    desiredTwist.vel = poseError.vel*velMag
    poseError.rot.Normalize() # normalize to have the ang vel direction
    desiredTwist.rot = poseError.rot*angVelMag
    return desiredTwist

def arrayToPyKDLRotation(array):
    x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
    y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
    z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])
    return PyKDL.Rotation(x,y,z)

def arrayToPyKDLFrame(array):
    rot = arrayToPyKDLRotation(array)
    pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
    return PyKDL.Frame(rot,pos)

def resetZRot(robot):
    curr = robot.get_current_joint_position()
    # if(abs(curr[3]) > np.pi):
    #     print("RESETTING Z")
    robot.move_joint(np.array([curr[0],
                               curr[1],
                               curr[2],
                               0,
                               curr[4],
                               curr[5]]))

def main():
    robot = psm('PSM2')

    resolvedRatesConfig = \
    {   'velMin': 2.0/1000,
        'velMax': 30.0/1000,
        'angVelMin': 1.0/180.0*3.14,
        'angVelMax': 60.0/180.0*3.14,
        'tolPos': 0.1/1000.0, # positional tolerance
        'tolRot': 1.0/180.0*3.14, # rotational tolerance
        'velRatio': 1, # the ratio of max velocity error radius to tolarance radius, this value >1
        'rotRatio': 1,
        'dt': 1.0/rate, # this is the time step of the system. 
                        # if rate=1khz, then dt=1.0/1000. However, 
                        # we don't know if the reality will be the same as desired rate
    }


    safeSpot = PyKDL.Frame()
    safeSpot.p = PyKDL.Vector(0,0.00,-0.05)
    safeSpot.M = rotationFromVector(PyKDL.Vector(0,0,-.1))
    robot.move(safeSpot)
    resetZRot()



if __name__ == '__main__':
    main()