from registration_gui import cleanResourcePath
from geometry_msgs.msg import PoseStamped
import vtk
import sys
import warnings
from dvrk_vision.vtk_stereo_viewer import StereoCameras
import dvrk_vision.vtktools as vtktools
import rospy
import PyKDL
from tf_conversions import posemath
from tf import transformations
import numpy as np
from overlay_gui import OverlayWidget
import cv2
from uvtoworld import UVToWorldConverter, rendertools
import os.path as path

class Obb:
    def __init__(self, polyData):
        self.meshData = polyData
        self.meshTree = vtk.vtkOBBTree()
        self.ptsOfIntersection_current = []
        self.start = None
        self.end = None
        self.meshTree.SetDataSet(self.meshData)
        self.buildTree() 

    def buildTree(self):
        self.meshTree.BuildLocator()
    
    def FindIntersection(self, start, end, rebuild=0):
        del self.ptsOfIntersection_current[:]
        if rebuild==1:
            self.buildTree()
        pts = vtk.vtkPoints()
        success = self.meshTree.IntersectWithLine(start, end, pts, None)
        if success==0:
            print("WARNING: Intersection point not found !")
        if success==-1:
            print("WARNING: Intersection point inside the Organ !")
        pointsData = pts.GetData()
        num = pointsData.GetNumberOfTuples()
        for i in range(num):
            temp = pointsData.GetTuple3(i)
            self.ptsOfIntersection_current.append(temp)
        self.ptsOfIntersection_current.append(start)
        self.ptsOfIntersection_current.append(end)
        return self.ptsOfIntersection_current

    def Visualize(self):
        visualData = vtk.vtkPolyData()
        self.meshTree.GenerateRepresentation(7, visualData)
        obbmapper = vtk.vtkPolyDataMapper()
        obbmapper.SetInputData(visualData)
        obbactor = vtk.vtkActor()
        obbactor.SetMapper(obbmapper)
        meshmapper = vtk.vtkPolyDataMapper()
        meshmapper.SetInputData(self.meshData)
        meshactor = vtk.vtkActor()
        meshactor.SetMapper(meshmapper)
        meshactor.GetProperty().SetColor(vtk.vtkNamedColors().GetColor3d("flesh"))
        return meshactor,obbactor

def createMarker(radius=0.0015, color=(0,1,0)):
    src = vtk.vtkSphereSource()
    src.SetRadius(radius)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(src.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)
    actor.GetProperty().SetOpacity(0.55)
    return actor

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
    def __init__(self, camera, texturePath, meshPath, scale=1, masterWidget=None, parent=None):
        super(ForceOverlayWidget, self).__init__(camera, texturePath, meshPath, scale, masterWidget, parent)
        import yaml
        yamlFile = cleanResourcePath("package://dvrk_vision/defaults/registration_params.yaml")
        with open(yamlFile, 'r') as stream:
            data = yaml.load(stream)
        self.cameraTransform = self.arrayToPyKDLFrame(data['transform'])
        self.activeWindow=0
        PNG = vtk.vtkPNGReader()
        PNG.SetFileName(cleanResourcePath(texturePath))
        PNG.Update()
        vtkimg = PNG.GetOutput()
        origin = vtkimg.GetOrigin()
        spacing = vtkimg.GetSpacing()
        extent = vtkimg.GetExtent()
        xc = origin[0] + 0.5*(extent[0] + extent[1]) * spacing[0]
        yc = origin[1] + 0.5*(extent[2] + extent[3]) * spacing[1]
        imageCenter = (xc, yc, 0)
        self.texscale = imageCenter[0] * 2

    def renderSetup(self):
        super(ForceOverlayWidget, self).renderSetup()
        self.arrowActor = makeArrow(tipLength=.15)
        self.vtkWidget.ren.AddActor(self.arrowActor)
        self.OrganObb = Obb(self.actor_moving.GetMapper().GetInput())
        self.MeshActor, self.ObbActor = self.OrganObb.Visualize()
        self.vtkWidget.ren.AddActor(self.MeshActor)
        self.vtkWidget.ren.AddActor(self.ObbActor)
        self.uvConverter = UVToWorldConverter(self.actor_moving.GetMapper().GetInput())
        self.poseSub = rospy.Subscriber('/dvrk/PSM2/position_cartesian_current', PoseStamped, self.robotPoseCb)
        #Markers to debug Start point and end Point of tool tip extension
        self.DebugActorStartPoint=createMarker(color=(1,0,0))
        self.vtkWidget.ren.AddActor(self.DebugActorStartPoint)
        self.DebugActorEndPoint=createMarker(color=(1,0,0))
        self.vtkWidget.ren.AddActor(self.DebugActorEndPoint)
        self.SurfaceTracker=createMarker(color=(0,1,0))
        self.vtkWidget.ren.AddActor(self.SurfaceTracker)

        self.textSource = vtk.vtkVectorText()
        self.textSource.SetText("??")
        self.textSource.Update()
        textMapper = vtk.vtkPolyDataMapper()
        textMapper.SetInputConnection(self.textSource.GetOutputPort())
        self.textActor = vtk.vtkFollower()
        self.textActor.SetMapper(textMapper)
        self.textActor.SetScale(.005)
        self.textActor.RotateZ(180)
        self.textActor.RotateY(180)
        self.vtkWidget.ren.AddActor(self.textActor)
        self.intensityMap = self.image[:,:,0] + self.image[:,:,1] + self.image[:,:,2]
        self.intensityMap = self.intensityMap.astype(np.float)/(255*3)
        self.annotatedTexture = np.copy(self.image)
        self.debugActors(1)

    def debugActors(self, debug):
        if debug==0:
            self.DebugActorStartPoint.VisibilityOff()
            self.DebugActorEndPoint.VisibilityOff()
            self.ObbActor.VisibilityOff()
            self.MeshActor.VisibilityOff()
        if debug==1:
            self.DebugActorStartPoint.VisibilityOn()
            self.DebugActorEndPoint.VisibilityOn()
            self.ObbActor.VisibilityOff()
            self.MeshActor.VisibilityOff()

        if debug==2:
            self.DebugActorStartPoint.VisibilityOff()
            self.DebugActorEndPoint.VisibilityOn()
            self.arrowActor.VisibilityOff()
    
    def robotPoseCb(self, data):
        if self.activeWindow==1:
            toolPosition = data.pose.position
            toolRotation = data.pose.orientation
            mat = transformations.quaternion_matrix([toolRotation.x,toolRotation.y,toolRotation.z,toolRotation.w])
            mat[0:3,3] = [toolPosition.x,toolPosition.y,toolPosition.z]
            pose = posemath.fromMatrix(mat)
            normalRotation = PyKDL.Frame(PyKDL.Rotation.RotY(-np.pi/2), PyKDL.Vector(0, 0, 0))   
            pose = pose * normalRotation
            mat = posemath.toMatrix(self.cameraTransform.Inverse() * pose)
            self.textActor.SetPosition(mat[0:3,3])
            startPoint = mat[0:3,3]
            endPoint = startPoint + mat[0:3,0]
            self.DebugActorStartPoint.SetPosition(startPoint)
            self.DebugActorEndPoint.SetPosition(endPoint)
            # Transform into organ frame
            organTransform = self.actor_moving.GetMatrix()
            organTransform.Invert()
            if organTransform==vtk.vtkMatrix4x4().Identity():
                print('Error: Organ transform not available')
                return
            endPoint = organTransform.MultiplyPoint(np.append(endPoint,1))[0:3]
            startPoint = organTransform.MultiplyPoint(np.append(startPoint,1))[0:3]
            intersection = self.OrganObb.FindIntersection(startPoint, endPoint)
            if len(intersection)<3:
                return
            uvPoint = self.uvConverter.toUVSpace(intersection[0])
            # Transform back into camera frame
            organTransform.Invert()
            intersectPoint = organTransform.MultiplyPoint(np.append(intersection[0],1))[0:3]
            self.SurfaceTracker.SetPosition(intersectPoint)
            mat[0:3,0] *= np.linalg.norm(np.subtract(mat[0:3,3], intersectPoint))
            mat[0:3,1:3] *= 0.2
            arrowTransform = vtk.vtkTransform()
            arrowTransform.Identity()
            arrowTransform.SetMatrix(mat.ravel())
            self.arrowActor.SetUserTransform(arrowTransform)
            uvPoint *= self.texscale
            if uvPoint[0]<0 or uvPoint[1]<0:
                color = vtk.vtkNamedColors().GetColor3d("turquoise")
            else:
                color = self.image[uvPoint[0]][uvPoint[1]]
                color = color/float(255)
                #start event
                self.annotatedTexture[uvPoint[0]][uvPoint[1]]=[255,255,255]
#            self.actor_moving.setTexture(self.annotatedTexture)
#end event        
#        self.actor_moving.setTexture(self.image)
            print(color)
            self.textActor.GetProperty().SetColor(color)
            color =[round(c,4) for c in color]
            self.textSource.SetText(str(color))
            self.textSource.Update()
            #cv2.imshow("Annotated Texture", cv2.resize(self.annotatedTexture, (500,500)))


    def arrayToPyKDLRotation(self, array):
        x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
        y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
        z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])

        return PyKDL.Rotation(x,y,z)

    def arrayToPyKDLFrame(self, array):
        rot = self.arrayToPyKDLRotation(array)
        pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
        return PyKDL.Frame(rot,pos)

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

#    texturePath = "package://oct_15_demo/resources/largeProstate.png"
    texturePath = "largeProstate.png"
    
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

    windowL = ForceOverlayWidget(cams.camL, texturePath, meshPath, scale=stlScale)
    windowL.show()
#    windowR = ForceOverlayWidget(cams.camR, texturePath, meshPath, scale=stlScale, cameraTransform=cameraTransform,  masterWidget=windowL)
#    windowR.show()
    rosThread.start()
    sys.exit(app.exec_())