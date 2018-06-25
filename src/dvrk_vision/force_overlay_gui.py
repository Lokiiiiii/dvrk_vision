from registration_gui import cleanResourcePath
from geometry_msgs.msg import PoseStamped
import vtk
import sys
from dvrk_vision.vtk_stereo_viewer import StereoCameras
import dvrk_vision.vtktools as vtktools
import rospy
import PyKDL
from tf_conversions import posemath
from tf import transformations
import numpy as np
from dvrk_vision.overlay_gui import OverlayWidget
import cv2
from uv_to_world import UVToWorldConverter

def findEndPoints(actor):
    endPoints = actor.GetBounds()
    startPoint = [endPoints[0], endPoints[2], endPoints[4]]
    endPoint = [endPoints[1], endPoints[3], endPoints[5]]
    return startPoint, endPoint

class MeshLayer:
    def __init__(self, polyData, scalingFactor = 1):
        self.scalingFactor = scalingFactor
        self.meshData = polyData
        self.meshActor = vtk.vtkActor()
        self.meshTree = vtk.vtkOBBTree()
        self.ptsOfIntersection_current = []
        self.ptsOfIntersection_history = []
        self.start = None
        self.end = None  
    
    def scalingFilter(self):
        transform = vtk.vtkTransform()
        transform.Scale(self.scalingFactor, self.scalingFactor, self.scalingFactor)
        PDfilter = vtk.vtkTransformPolyDataFilter()
        PDfilter.SetInputData(self.meshData)
        PDfilter.SetTransform(transform)
        PDfilter.Update()
        return PDfilter.GetOutput()

    def buildTree(self):
        #Scale model and build OBB tree
        self.meshTree.SetDataSet(self.scalingFilter())
        self.meshTree.BuildLocator()
    
    def FindIntersection(self, start, end, rebuild=0):
        del self.ptsOfIntersection_current[:]
        if rebuild==1:
            self.buildTree()
        pts = vtk.vtkPoints()
        success = self.meshTree.IntersectWithLine(start, end, pts, None)
        pointsData = pts.GetData()
        num = pointsData.GetNumberOfTuples()
        for i in range(num):
            temp = pointsData.GetTuple3(i)
            self.ptsOfIntersection_current.append(temp)
        self.ptsOfIntersection_current.append(start)
        self.ptsOfIntersection_current.append(end)
        self.ptsOfIntersection_history.append(self.ptsOfIntersection_current)   
        return self.ptsOfIntersection_current

    def createMarker(self):
        src = vtk.vtkSphereSource()
        src.SetRadius(0.002)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(src.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0,1,0)
        actor.GetProperty().SetOpacity(0.55)
        return actor


def arrayToPyKDLRotation(array):
    x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
    y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
    z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])
    return PyKDL.Rotation(x,y,z)

def arrayToPyKDLFrame(array):
    rot = arrayToPyKDLRotation(array)
    pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
    return PyKDL.Frame(rot,pos)

def setActorMatrix(actor, npMatrix):
    transform = vtk.vtkTransform()
    origin = [1,1,1,0] + [1,1,0,0] + [0,0,1,0] + [0,0,0,1]
    transform.SetMatrix(npMatrix)
    transform.Scale(100,100,100)
    actor.SetUserTransform(transform)

def makeArrow(coneRadius = .01, shaftRadius = 0.005, tipLength = .05):
    arrowSource = vtk.vtkArrowSource()
    arrowSource.SetShaftRadius(shaftRadius)
    arrowSource.SetTipRadius(coneRadius)
    arrowSource.SetTipLength(tipLength)
    # arrowSource.InvertOn()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(arrowSource.GetOutputPort())
    arrowActor = vtk.vtkActor()
    arrowActor.SetMapper(mapper)
    arrowActor.GetProperty().SetOpacity(1)
    arrowActor.GetProperty().SetColor(0.05,0.01,0.7)
    return arrowActor

class ForceOverlayWidget(OverlayWidget):

    def __init__(self, camera, texturePath, meshPath, scale=1, cameraTransform=None, masterWidget=None, parent=None):
        if cameraTransform == None:
            cameraTransform = PyKDL.Frame.Identity()
        self.cameraTransform = cameraTransform
        OverlayWidget.__init__(self, camera, texturePath, meshPath, scale, masterWidget, parent)

    def renderSetup(self):
        OverlayWidget.renderSetup(self)
        self.arrowActor = makeArrow(tipLength=.15)
        self.vtkWidget.ren.AddActor(self.arrowActor)
        self.meshLayer = MeshLayer(self.actor_moving.GetMapper().GetInput(),
                                   scalingFactor=self.scale)
        self.meshLayer.buildTree()
        self.uvConverter = UVToWorldConverter(self.MeshLayer.scalingFilter())
        self.poseSub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current', PoseStamped, self.robotPoseCb)
        
        #Markers to debug Start point and end Point of tool tip extension
        self.DebugActorStartPoint=self.meshLayer.createMarker()
        self.vtkWidget.ren.AddActor(self.DebugActorStartPoint)
        self.DebugActorEndPoint=self.meshLayer.createMarker()
        self.vtkWidget.ren.AddActor(self.DebugActorEndPoint)
        self.textSource = vtk.vtkVectorText()
        textMapper = vtk.vtkPolyDatamapper()
        textMapper.SetInputConnection(textSource.GetOutputPort())
        self.textActor = vtk.vtkActor()
        self.textActor.SetMapper(textMapper)
        self.vtkWidget.ren.AddActor(self.textActor)


    def debugActors(self, debug):
        if debug==0:
            self.DebugActorStartPoint.VisibilityOff()
            self.DebugActorEndPoint.VisibilityOff()
        if debug==1:
            self.DebugActorStartPoint.VisibilityOn()
            self.DebugActorEndPoint.VisibilityOn()
        if debug==2:
            self.DebugActorStartPoint.VisibilityOff()
            self.DebugActorEndPoint.VisibilityOn()
            self.arrowActor.VisibilityOff()

    def robotPoseCb(self, data):
        toolPosition = data.pose.position
        toolRotation = data.pose.orientation
        mat = transformations.quaternion_matrix([toolRotation.x,toolRotation.y,toolRotation.z,toolRotation.w])
        mat[0:3,3] = [toolPosition.x,toolPosition.y,toolPosition.z]
        pose = posemath.fromMatrix(mat)
        normalRotation = PyKDL.Frame(PyKDL.Rotation.RotY(-np.pi/2), PyKDL.Vector(0, 0, 0))   
        pose = pose * normalRotation
        mat = posemath.toMatrix(cameraTransform.Inverse() * pose)
        # mat[0:3,0:3] *= .1
        startPoint = mat[0:3,3]
        endPoint = startPoint + mat[0:3,0]
        self.DebugActorStartPoint.SetPosition(startPoint)
        self.DebugActorEndPoint.SetPosition(endPoint)
        # Transform into organ frame
        organTransform = self.actor_moving.GetMatrix()
        organTransform.Invert()
        if organTransform==None:
            print('Error: Organ transform not available')
            return
        endPoint = organTransform.MultiplyPoint(np.append(endPoint,1))[0:3]
        startPoint = organTransform.MultiplyPoint(np.append(startPoint,1))[0:3]
        intersection = self.meshLayer.FindIntersection(startPoint, endPoint)
        if len(intersection)==0:
            print('No point of intersection found')
            return
        # Transform back into camera frame
        organTransform.Invert()
        intersectPoint = organTransform.MultiplyPoint(np.append(intersection[0],1))[0:3]
        self.DebugActorEndPoint.SetPosition(intersectPoint)
        mat[0:3,0] *= np.linalg.norm(np.subtract(mat[0:3,3], intersectPoint))
        mat[0:3,1:3] *= 0.2
        arrowTransform = vtk.vtkTransform()
        arrowTransform.Identity()
        arrowTransform.SetMatrix(mat.ravel())
        self.arrowActor.SetUserTransform(arrowTransform)
        self.debugActors(1)
        #cv2.imshow("Sandbox",self.image)
        uvPoint = self.uvConverter.toUVSpace(intersectPoint)
        color = self.image[uvPoint[0]][uvPoint[1]]
        print(color)
        self.textSource.SetText(string(color))
        self.textActor.GetProperty.SetColor(color)
        textTransform = vtk.vtkTransform()
        textTransform.Identity()
        textTransform.Translate([toolPosition.x,toolPosition.y,toolPosition.z])
        textTransform.Scale(1,1,1)
        self.textActor.SetUserTransform(textTransform)
        self.textActor.Update()


if __name__ == "__main__":
    # Which PyQt we use depends on our vtk version. QT4 causes segfaults with vtk > 6
    if(int(vtk.vtkVersion.GetVTKVersion()[0]) >= 6):
        from PyQt5.QtWidgets import QWidget, QVBoxLayout, QApplication
        from PyQt5 import uic
        _QT_VERSION = 5
    else:
        from PyQt4.QtGui import QWidget, QVBoxLayout, QApplication
        from PyQt4 import uic
        _QT_VERSION = 4

    import yaml
    yamlFile = cleanResourcePath("package://dvrk_vision/defaults/registration_params.yaml")
    #texturePath = "package://oct_15_demo/resources/largeProstate.png"
    texturePath = "largeProstate.png"
    with open(yamlFile, 'r') as stream:
        data = yaml.load(stream)
    cameraTransform = arrayToPyKDLFrame(data['transform'])
    app = QApplication(sys.argv)
    rosThread = vtktools.QRosThread()
    # meshPath = rospy.get_param("~mesh_path")
    # stlScale = rospy.get_param("~mesh_scale")
    meshPath = "largeProstate.obj"
    stlScale = 1.08
    frameRate = 15
    slop = 1.0 / frameRate
    cams = StereoCameras("/stereo/left/image_rect",
                         "/stereo/right/image_rect",
                         "/stereo/left/camera_info",
                         "/stereo/right/camera_info",
                         slop = slop)
    windowL = ForceOverlayWidget(cams.camL, texturePath, meshPath, scale=stlScale, cameraTransform=cameraTransform)
    #windowL.Initialize()
    #windowL.start()
    windowL.show()
    windowR = ForceOverlayWidget(cams.camR, texturePath, meshPath, scale=stlScale, cameraTransform=cameraTransform,  masterWidget=windowL)
    #windowR.Initialize()
    #windowR.start()
    windowR.show()
    rosThread.start()
    sys.exit(app.exec_())