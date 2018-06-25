#!/usr/bin/env python
import vtk
import numpy as np
import rospy
import cv2
# Which PyQt we use depends on our vtk version. QT4 causes segfaults with vtk > 6
if(int(vtk.vtkVersion.GetVTKVersion()[0]) >= 6):
    from PyQt5.QtWidgets import QWidget, QVBoxLayout, QApplication
    _QT_VERSION = 5
else:
    from PyQt4.QtGui import QWidget, QVBoxLayout, QApplication
    _QT_VERSION = 4
from QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import dvrk_vision.vtktools as vtktools
from force_sensor_gateway.msg import ForceSensorData
from geometry_msgs.msg import WrenchStamped
from dvrk_vision.vtk_stereo_viewer import StereoCameras, QVTKStereoViewer
from dvrk_vision.clean_resource_path import cleanResourcePath
from dvrk import psm
import PyKDL
from tf_conversions import posemath
import colorsys


#pose = PyKDL.Frame()

def cleanResourcePath(path):
	newPath = path
	if path.find("package://") == 0:
		newPath = newPath[len("package://"):]
		pos = newPath.find("/")
		if pos == -1:
			rospy.logfatal("%s Could not parse package:// format", path)
			quit(1)

		package = newPath[0:pos]
		newPath = newPath[pos:]
		package_path = rospkg.RosPack().get_path(package)

		if package_path == "":
			rospy.logfatal("%s Package [%s] does not exist",
						   path.c_str(),
						   package.c_str());
			quit(1)

		newPath = package_path + newPath;
	elif path.find("file://") == 0:
		newPath = newPath[len("file://"):]

	if not os.path.isfile(newPath):
		rospy.logfatal("%s file does not exist", newPath)
		quit(1)
	return newPath;

def poseCb(data):
	global pose
	pose = posemath.fromMsg(data.pose)

def makeArrowActor(coneRadius = .1, shaftRadius = 0.03, tipLength = 0.35):
    arrowSource = vtk.vtkArrowSource()
    arrowSource.SetShaftRadius (shaftRadius)
    arrowSource.SetTipRadius (coneRadius)
    arrowSource.SetTipLength (tipLength)
    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(arrowSource.GetOutput())
    else:
        mapper.SetInputConnection(arrowSource.GetOutputPort())
    arrowActor = vtk.vtkActor()
    arrowActor.SetMapper(mapper)
    return arrowActor

def setActorMatrix(actor, npMatrix):
    transform = vtk.vtkTransform()
    transform.Identity()
    transform.SetMatrix(npMatrix.ravel())
    actor.SetPosition(transform.GetPosition())
    actor.SetOrientation(transform.GetOrientation())
    actor.SetScale(transform.GetScale())

class ForceOverlayWidget(QVTKStereoViewer):
    def __init__(self, cam, camTransform, dvrkName, forceTopic, draw="bar", masterWidget=None, parent=None):
        super(ForceOverlayWidget, self).__init__(cam, parent=parent)
        self.masterWidget = masterWidget
        self.cameraTransform = cameraTransform
        self.drawType = "arrow"

        self.force = None
        self.pose = None
        self.forceSub = rospy.Subscriber('/force_sensor_topic',
                                     ForceSensorData, self.forceCb)
        self.pose = None
        self.poseSub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current',
                                        PoseStamped, self.poseCb)

        if self.masterWidget == None:
            pass
        else:
            pass

    def forceCb(self, data):
        force = np.array([data.data1, data.data2, data.data3, data.data4], np.float32) / 30.0
        force[force < 0] = 0
        self.force = force

    def poseCb(self, data):
        self.pose = posemath.fromMsg(data.pose)
        if self.masterWidget == None:
            self.robot = psm(dvrkName)
        else:
            self.robot = self.masterWidget.robot
        self.cameraTransform = camTransform
        self.drawType = draw
        rospy.Subscriber(forceTopic, WrenchStamped, self.forceCB)

    def renderSetup(self):
        if self.drawType == "arrow":
            arrowSource = vtk.vtkArrowSource()
            mapper = vtk.vtkPolyDataMapper()
            if vtk.VTK_MAJOR_VERSION <= 5:
                mapper.SetInput(arrowSource.GetOutput())
            else:
                mapper.SetInputConnection(arrowSource.GetOutputPort())
            # Create actor that we will position according to dVRK
            self.arrowActor = makeArrowActor()
            self.targetActor = makeArrowActor(coneRadius = .07,
                                              shaftRadius = .02)
            self.targetActor.GetProperty().SetOpacity(.2)
            self.ren.AddActor(self.arrowActor)
            self.ren.AddActor(self.targetActor)

        elif self.drawType == "bar":
            # Make two color bars to show current force
            source = vtk.vtkCubeSource()
            source.SetBounds((-.002, .002, 0, .05, 0, .001))
            mapper = vtk.vtkPolyDataMapper()
            if vtk.VTK_MAJOR_VERSION <= 5:
                mapper.SetInput(source.GetOutput())
            else:
                mapper.SetInputConnection(source.GetOutputPort())
            self.bar = vtk.vtkActor()
            self.bar.SetMapper(mapper)
            self.bar.GetProperty().SetColor(.2,.2,.2)
            self.bar.GetProperty().LightingOff()
            self.forceBar = vtk.vtkActor()
            self.forceBar.SetMapper(mapper)
            self.forceBar.GetProperty().LightingOff()
            self.ren.AddActor(self.bar)
            self.ren.AddActor(self.forceBar)
            # Make a green line to show target force
            source2 = vtk.vtkCubeSource()
            source2.SetBounds((-.002, .002, .0245, .0255, 0, .001001))
            mapper2 = vtk.vtkPolyDataMapper()
            if vtk.VTK_MAJOR_VERSION <= 5:
                mapper2.SetInput(source2.GetOutput())
            else:
                mapper2.SetInputConnection(source2.GetOutputPort())
            self.greenLine = vtk.vtkActor()
            self.greenLine.SetMapper(mapper2)
            self.greenLine.GetProperty().SetColor(.9,.9,.9)
            self.greenLine.GetProperty().LightingOff()
            self.ren.AddActor(self.greenLine)

        # Setup interactor
        self.iren = self.GetRenderWindow().GetInteractor()
        self.iren.RemoveObservers('LeftButtonPressEvent')
        self.iren.RemoveObservers('LeftButtonReleaseEvent')
        self.iren.RemoveObservers('MouseMoveEvent')
        self.iren.RemoveObservers('MiddleButtonPressEvent')
        self.iren.RemoveObservers('MiddleButtonPressEvent')
        self.currentForce
    
    def forceCB(self, data):
        self.currentForce = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]

    def imageProc(self,image):
        # Get current force
        force = self.currentForce
        force = np.linalg.norm(force)
        targetF = 4 # Newtons
        targetR = 1 # Newtons
        # Calculate color
        xp = [targetF-targetR, targetF, targetF+targetR]
        fp = [0, 1, 0]
        colorPos = np.interp(force, xp, fp)
        color = colorsys.hsv_to_rgb(colorPos**3 / 3, .8,1)

        if self.drawType == "arrow":
            self.arrowActor.GetProperty().SetColor(color[0], color[1], color[2])
            # Calculate pose of arrows
            initialRot = PyKDL.Frame(PyKDL.Rotation.RotY(np.pi / 2),
                                     PyKDL.Vector(0, 0, 0))
            pos = self.pose * initialRot
            pos = self.cameraTransform.Inverse() * pos
            posMat = posemath.toMatrix(pos)
            posMatTarget = posMat.copy()
            # Scale arrows
            posMat[0:3,0:3] = posMat[0:3,0:3] * targetR / 5 * targetF
            setActorMatrix(self.targetActor, posMat)
            posMat[0:3,0:3] = posMat[0:3,0:3] * force / targetF
            setActorMatrix(self.arrowActor, posMat)

        return image
        elif self.drawType == "bar":
            self.forceBar.GetProperty().SetColor(color[0], color[1], color[2])
            # Move background bar
            pos = self.robot.get_current_position()
            pos = self.cameraTransform.Inverse() * pos
            pos2 = PyKDL.Frame(PyKDL.Rotation.Identity(), pos.p)
            pos2.M.DoRotZ(np.pi)
            pos2.p = pos2.p + pos2.M.UnitX() * -.015
            posMat = posemath.toMatrix(pos2)
            setActorMatrix(self.bar, posMat)
            setActorMatrix(self.greenLine, posMat)
            # Scale color bar
            fp2 = [0, .5, 1]
            scalePos = np.interp(force, xp, fp2)
            print scalePos
            posMat[1,0:3] = posMat[1,0:3] * scalePos
            setActorMatrix(self.forceBar, posMat)

        return image

def arrayToPyKDLRotation(array):
	x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
	y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
	z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])
	return PyKDL.Rotation(x,y,z)

def arrayToPyKDLFrame(array):
	rot = arrayToPyKDLRotation(array)
	pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
	return PyKDL.Frame(rot,pos)

def findEndPoints(actor):
	endPoints = actor.GetBounds()
	startPoint = [endPoints[0], endPoints[2], endPoints[4]]
	endPoint = [endPoints[1], endPoints[3], endPoints[5]]
	return startPoint, endPoint

def logit(msg):
	fhandle = open("/home/loki/research/ws0/src/dvrk_vision/src/dvrk_vision/loki.log", "w+")
	fhandle.write(msg)
	fhandle.close()

class meshLayer:
	srcFile = None
	meshData = None
	meshActor = vtk.vtkActor()
	meshTree = vtk.vtkOBBTree()
	ptsOfIntersection_current = []
	ptsOfIntersection_history = []
	renderer = None
	start = None
	end = None
	scalingFactor=1
	
	def setSrcFile(self, fname):
		self.srcFile = fname	

	def loadOBJ(self):
		readerOBJ = vtk.vtkOBJReader()
		readerOBJ.SetFileName(self.srcFile)
		readerOBJ.Update()
		self.meshData = readerOBJ.GetOutput()
		if self.meshData.GetNumberOfPoints()==0:
			raise ValueError("Point data not found")
	
	def scalingFilter(self):
		transform = vtk.vtkTransform()
		transform.Scale(self.scalingFactor, self.scalingFactor, self.scalingFactor)
		logit("waypoints1")
		PDfilter = vtk.vtkTransformPolyDataFilter()
		PDfilter.SetInputData(self.meshData)
		logit("waypoints2")
		PDfilter.SetTransform(transform)
		logit("waypoints3")
		PDfilter.Update()
		return PDfilter.GetOutput()
	
	def createMesh(self, scale=1):
		self.loadOBJ()	
		logit("waypointX")
		mapper = vtk.vtkPolyDataMapper()
		if scale==1:
			mapper.SetInputData(self.meshData)
		if scale!=1:
			logit("waypointY")
			self.scalingFactor=scale
			mapper.SetInputData(vtk.vtkDataSet.SafeDownCast(self.scalingFilter()))
		logit("waypointZ")
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

if __name__ == "__main__":
    import sys
    import yaml
    import dvrk_vision.vtktools as vtktools
    """A simple example that uses the ForceOverlayWidget class."""

    # every QT app needs an app
	#app = QApplication(['QVTKRenderWindowInteractor'])
    app = QApplication(['Force Overlay'])
    yamlFile = cleanResourcePath("package://dvrk_vision/defaults/registration_params.yaml")
    with open(yamlFile, 'r') as stream:
        data = yaml.load(stream)
    cameraTransform = arrayToPyKDLFrame(data['transform'])

    rosThread = vtktools.QRosThread()
    rosThread.start()
    frameRate = 15
    slop = 1.0 / frameRate
    cams = StereoCameras("stereo/left/image_rect",
                         "stereo/right/image_rect",
                         "stereo/left/camera_info",
                         "stereo/right/camera_info",
                         slop = slop)

    windowL = ForceOverlayWidget(cam = cams.camL,
                                 camTransform = cameraTransform,
                                 dvrkName = 'PSM2',
                                 forceTopic = '/atinetft/wrench')
    windowL.Initialize()
    windowL.start()
    windowL.show()
    sys.exit(app.exec_())
//Merge from Loki
/*
	renderer = vtk.vtkRenderer()
	prostateMesh = meshLayer()
	FilePath = "/home/loki/research/ws0/src/dvrk_vision/src/dvrk_vision/largeProstate.obj"
	prostateMesh.setSrcFile(FilePath)
	prostateMesh.createMesh(20)
	prostateMesh.setRenderer(renderer)
	renderer.AddActor(prostateMesh.getActor())
	arrow = makeArrow(cameraTransform)
	startpt, endpt = findEndPoints(arrow)
	renderer.AddActor(arrow)
	renderer.SetBackground(1,1,1)
	prostateMesh.setEndPts(startpt, endpt)
	
	window = vtk.vtkRenderWindow()
	window.AddRenderer(renderer)
	interactor = vtk.vtkRenderWindowInteractor()
	interactor.SetRenderWindow(window)
	window.Render()
	style = vtk.vtkInteractorStyleTrackballActor()
	interactor.SetInteractorStyle(style)
	logit("waypoint1")
	interactor.AddObserver('LeftButtonPressEvent', prostateMesh.intersectAndMark)
	interactor.Initialize()
	interactor.Start()
*/
