from PyQt5.QtWidgets import QTableWidget
from multimodal_writer.msg import Wacom
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3


class TactileSurfaceArea(QTableWidget):
    def __init__(self, parent = None):
		QTableWidget.__init__(self, parent)
        self.deviceDown = False
		self.lastPoints = [QPoint(), QPoint()]
        self.wacom_msg = Wacom()
        self.frame_id = 0

    def tabletEvent(self, event):
		if event.type() == QTabletEvent.TabletPress:
			if self.deviceDown == False:
				self.deviceDown = True
				self.lastPoints[0] = event.pos()
				self.lastPoints[1] = event.pos()

		elif event.type() == QTabletEvent.TabletRelease:
			if self.deviceDown:
				self.deviceDown = False

		elif event.type() == QTabletEvent.TabletMove:
			self.lastPoints[1] = self.lastPoints[0]
			self.lastPoints[0] = event.pos()

            self.frame_id += 1
            self.wacom_msg.id = self.frame_id
            self.wacom_msg.time = rospy.get_rostime()
            self.wacom_msg.tip_position = Vector3(event.posF().x(), event.posF().y(), event.posF().z())
            self.wacom_msg.tilt = Point(event.xTilt(), event.yTilt())
            self.pressure = event.pressure()

if __name__ == '__main__':
	# init node
    rospy.init_node("wacom_client")

    actileSurface = TactileSurfaceArea()
    while True:
        rospy.spin()
t
