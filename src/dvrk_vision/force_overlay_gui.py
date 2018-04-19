from registration_gui import RegistrationWidget, cleanResourcePath
from geometry_msgs.msg import PoseStamped
import vtk
import sys
from vtk_stereo_viewer import StereoCameras, QVTKStereoViewer
import vtktools
import rospy
import PyKDL
from tf_conversions import posemath
from tf import transformations
import numpy as np

def findEndPoints(actor):
    endPoints = actor.GetBounds()
    startPoint = [endPoints[0], endPoints[2], endPoints[4]]
    endPoint = [endPoints[1], endPoints[3], endPoints[5]]
    return startPoint, endPoint

class MeshLayer:
    def __init__(self, polyData, scalingFactor = 1):
        # self.srcFile = srcFile
        # self.scalingFactor = scalingFactor
        self.meshData = polyData
        self.meshActor = vtk.vtkActor()
        self.meshTree = vtk.vtkOBBTree()
        self.ptsOfIntersection_current = []
        self.ptsOfIntersection_history = []
        self.renderer = None
        self.start = None
        self.end = None  

    def loadOBJ(self):
        readerOBJ = vtk.vtkOBJReader()
        readerOBJ.SetFileName(self.srcFile)
        readerOBJ.Update()
        self.meshData = readerOBJ.GetOutput()
        if self.meshData.GetNumberOfPoints() == 0:
            raise ValueError("Point data not found")
    
    def scalingFilter(self):
        transform = vtk.vtkTransform()
        transform.Scale(self.scalingFactor, self.scalingFactor, self.scalingFactor)
        PDfilter = vtk.vtkTransformPolyDataFilter()
        PDfilter.SetInputData(self.meshData)
        PDfilter.SetTransform(transform)
        PDfilter.Update()
        return PDfilter.GetOutput()
    
    def createMesh(self, scale=1):
        self.loadOBJ()  
        mapper = vtk.vtkPolyDataMapper()
        if scale==1:
            mapper.SetInputData(self.meshData)
        if scale!=1:
            self.scalingFactor=scale
            mapper.SetInputData(vtk.vtkDataSet.SafeDownCast(self.scalingFilter()))
        self.meshActor.SetMapper(mapper)
        self.meshActor.GetProperty().SetOpacity(0.5)
        self.meshActor.GetProperty().SetColor(0.75,0,0)

    def getActor(self):
        return self.meshActor

    def buildTree(self):
        self.meshTree.SetDataSet(self.meshActor.GetMapper().GetInput())
        self.meshTree.BuildLocator()
        
    def getTree(self):
        return self.meshTree
    
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
    
    def setRenderer(self, ren):
        self.renderer = ren

    def markPoints(self, current=1):
        if current==1:
            for pt in self.ptsOfIntersection_current:
                markerActor = self.createMarker(pt)
                self.renderer.AddActor(markerActor)

    def createMarker(self, point):
        src = vtk.vtkSphereSource()
        src.SetCenter(point)
        src.SetRadius(0.01)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(src.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0,1,0)
        actor.GetProperty().SetOpacity(0.55)
        return actor

    def setEndPts(self, pt1, pt2):
        self.start = pt1
        self.end = pt2
    
    def intersectAndMark(self, cb1, cb2):
        self.FindIntersection(self.start, self.end, rebuild=1)
        self.markPoints(current=1)

def arrayToPyKDLRotation(array):
    x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
    y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
    z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])
    return PyKDL.Rotation(x,y,z)

def arrayToPyKDLFrame(array):
    rot = arrayToPyKDLRotation(array)
    pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
    return PyKDL.Frame(rot,pos)

def pyKDLFrameToArray(frame):
    frame.M

def setActorMatrix(actor, npMatrix):
    transform = vtk.vtkTransform()
    origin = [1,1,1,0] + [1,1,0,0] + [0,0,1,0] + [0,0,0,1]
    transform.SetMatrix(npMatrix)
    transform.Scale(100,100,100)
    actor.SetUserTransform(transform)

def makeArrow(coneRadius = .02, shaftRadius = 0.009, tipLength = .2):
    arrowSource = vtk.vtkArrowSource()
    arrowSource.SetShaftRadius(shaftRadius)
    arrowSource.SetTipRadius(coneRadius)
    arrowSource.SetTipLength(tipLength)
    # arrowSource.InvertOn()
    
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(arrowSource.GetOutputPort())
    arrowActor = vtk.vtkActor()
    arrowActor.SetMapper(mapper)
    arrowActor.GetProperty().SetOpacity(0.45)
    arrowActor.GetProperty().SetColor(0.05,0.01,0.7)
    return arrowActor

class ForceOverlayWidget(RegistrationWidget):

    def __init__(self, camera, meshPath, scale=1, cameraTransform=None, masterWidget=None, parent=None):
        if cameraTransform == None:
            cameraTransform = PyKDL.Frame.Identity()
        self.cameraTransform = cameraTransform
        RegistrationWidget.__init__(self, camera, meshPath, scale, masterWidget, parent)

    def renderSetup(self):
        RegistrationWidget.renderSetup(self)
        self.arrowActor = makeArrow()
        # Add actor
        self.vtkWidget.ren.AddActor(self.arrowActor)
        self.meshLayer = MeshLayer
        RegistrationWidget.renderSetup(self)
        self.poseSub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current', PoseStamped, self.robotPoseCb)

    def robotPoseCb(self, data):
        # self.pose = posemath.fromMsg(data.pose)
        # normal_rot = PyKDL.Frame(PyKDL.Rotation.RotY(np.pi/2), PyKDL.Vector(0, 0, 0))   
        # normal_pos = self.pose * normal_rot
        # normal_posMat = posemath.toMatrix(self.cameraMatrix.Inverse() * normal_pos)
        # print(normal_posMat)
        # setActorMatrix(self.arrowActor, normal_posMat)
        pos = data.pose.position
        rot = data.pose.orientation
        mat = transformations.quaternion_matrix([rot.x,rot.y,rot.z,rot.w])
        mat[0:3,3] = [pos.x,pos.y,pos.z]
        pose = posemath.fromMatrix(mat)
        normal_rot = PyKDL.Frame(PyKDL.Rotation.RotY(-np.pi/2), PyKDL.Vector(0, 0, 0))   
        pose = pose * normal_rot
        mat = posemath.toMatrix(cameraTransform.Inverse() * pose)
        mat[0:3,0:3] *= .1
        transform = vtk.vtkTransform()
        transform.Identity()
        transform.SetMatrix(mat.ravel())
        self.arrowActor.SetUserTransform(transform)
        self.arrowActor.VisibilityOn()  

    def setCameraTransform(self, matrix):
        self.cameraTransform = matrix



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
    # """A simple example that uses the QVTKRenderWindowInteractor class."""
    # import yaml
    # import PyKDL
    # from geometry_msgs.msg import PoseStamped
    # from tf_conversions import posemath
    # # every QT app needs an app
    # app = QApplication(['QVTKRenderWindowInteractor'])
    # yamlFile = cleanResourcePath("package://dvrk_vision/defaults/registration_params.yaml")
    # with open(yamlFile, 'r') as stream:
    #     data = yaml.load(stream)
    # cameraTransform = arrayToPyKDLFrame(data['transform'])

    # renderer = vtk.vtkRenderer()
    
    # prostateMesh = meshLayer()
    # FilePath = "largeProstate.obj"
    # prostateMesh.setSrcFile(FilePath)
    # prostateMesh.createMesh(20)
    # prostateMesh.setRenderer(renderer)
    # renderer.AddActor(prostateMesh.getActor())
    # arrow = makeArrow(cameraTransform)
    # startpt, endpt = findEndPoints(arrow)
    # renderer.AddActor(arrow)
    # renderer.SetBackground(1,1,1)
    # prostateMesh.setEndPts(startpt, endpt)
    
    # window = vtk.vtkRenderWindow()
    # window.AddRenderer(renderer)
    # interactor = vtk.vtkRenderWindowInteractor()
    # interactor.SetRenderWindow(window)
    # window.Render()
    # style = vtk.vtkInteractorStyleTrackballActor()
    # interactor.SetInteractorStyle(style)
    # interactor.AddObserver('LeftButtonPressEvent', prostateMesh.intersectAndMark)
    # interactor.Initialize()
    # interactor.Start()
    # every QT app needs an app
    import yaml
    yamlFile = cleanResourcePath("package://dvrk_vision/defaults/registration_params.yaml")
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
    windowL = ForceOverlayWidget(cams.camL, meshPath, scale=stlScale, cameraTransform=cameraTransform)
    windowL.show()
    # windowR = RegistrationWidget(cams.camR, meshPath, scale=stlScale, masterWidget=windowL)
    # windowR.show()
    rosThread.start()
    sys.exit(app.exec_())