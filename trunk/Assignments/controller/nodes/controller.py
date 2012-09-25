#!/usr/bin/env python
import roslib; roslib.load_manifest('controller')
import rospy
try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program"

from std_msgs.msg import Int32


class controller(wx.Frame):

    def __init__(self,parent,id,title):
        wx.Frame.__init__(self,parent,id,title)
        self.parent = parent
        self.pub = rospy.Publisher('/action', Int32)

        sizer = wx.BoxSizer(wx.VERTICAL)

        self.SetSizer(sizer)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Controls"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
         
        move = wx.Button(self,wx.ID_ANY,label="Move forward")
        static_sizer.Add(move, 0)
        self.Bind(wx.EVT_BUTTON, self.move, move)
        
        turn = wx.Button(self,wx.ID_ANY,label="Turn 180 degrees")
        static_sizer.Add(turn, 0)
        self.Bind(wx.EVT_BUTTON, self.turn, turn)

        sense = wx.Button(self,wx.ID_ANY,label="Update sensor readings")
        static_sizer.Add(sense, 0)
        self.Bind(wx.EVT_BUTTON, self.sense, sense)

        movementnoise = wx.Button(self,wx.ID_ANY,label="movement noise")
        static_sizer.Add(movementnoise, 0)
        self.Bind(wx.EVT_BUTTON, self.movementnoise, movementnoise)

        measurementnoise = wx.Button(self,wx.ID_ANY,label="measurement noise")
        static_sizer.Add(measurementnoise, 0)
        self.Bind(wx.EVT_BUTTON, self.measurementnoise, measurementnoise)


        self.Layout()
        self.Fit()
        self.Show(True)

    def move(self, event):
        data = 0
        self.pub.publish(data)
	
    def turn(self, event):
        data = 1
        self.pub.publish(data)

    def sense(self, event):
        data = 2
        self.pub.publish(data)
   
    def movementnoise(self, event):
        data = 3
        self.pub.publish(data)

    def measurementnoise(self, event):
        data = 4
        self.pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('controller')
    app = wx.App()
    frame = controller(None,wx.ID_ANY,'Controller')

    app.MainLoop()
