#!/usr/bin/env python
import wx
import copy, sys
import os, os.path

p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

from specCompiler import SpecCompiler
import mapRenderer


#creating the main GUI
class Display(wx.Frame):
    '''The main application window'''
    
    def __init__(self, *args, **kwds):
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        
        #global variable, to be used later
        global PANEL
        PANEL = wx.Panel(self)
        PANEL.SetBackgroundColour('#4f5049')
        global BIG_BOX
        BIG_BOX = wx.BoxSizer(wx.HORIZONTAL)
        
        
        
        #opening the region file
        self.compiler = SpecCompiler(sys.argv[1])
        self.proj = copy.deepcopy(self.compiler.proj)
        self.proj.rfi = self.proj.loadRegionFile(decomposed= False)
        
        
        self.map()
        self.region_details()
        self.list_of_regions()
        self.control_panel()
        self.guidelines()
        
        
    def map(self):
        '''responsible for the box that displays the map'''
        global region_map
        region_map = wx.Panel(self, pos = (10, 10), size = (550, 380),
                              style = wx.SIMPLE_BORDER,name = 'Region Map')
        BIG_BOX.Add(region_map, proportion = 1,flag = wx.ALL|wx.EXPAND,
                border = 10)
        title = wx.StaticText(region_map,-1, 'Region Map', style = wx.ALIGN_TOP)
        
        
        region_map.Bind(wx.EVT_PAINT, self.drawMap)
        #import and display region file here
        #relevant regions will be greyed out or highlighted
    
    def drawMap(self, event):
        mapRenderer.drawMap(region_map, self.proj.rfi, scaleToFit=True)
        
    
    
    def control_panel(self):
        '''responsible for the control panel'''
        c_panel = wx.Panel(self, pos = (640, 10), size = (550, 380),
                              style = wx.SIMPLE_BORDER,name = 'Control Panel')
        BIG_BOX.Add(c_panel, proportion = 1,flag = wx.ALL|wx.EXPAND, border = 10)
        title = wx.StaticText(c_panel,-1, 'Control Panel', style = wx.ALIGN_TOP)
        
        #Create differnt movement related button here
        #these buttons control the Robot
        #it is the ONLY place where the robot can be controlled
        #relevant buttons will be greyed
    
    
    def region_details(self):
        '''responsible for region details box'''
        global r_details
        r_details = wx.Panel(self, pos = (10, 400), size = (390, 300),
                              style = wx.SIMPLE_BORDER, name = 'Region Details')
        BIG_BOX.Add(r_details, proportion = 1, flag = wx.ALL|wx.EXPAND,
                    border = 10)
        title = wx.StaticText(r_details,-1, 'Region Details', style = wx.ALIGN_TOP)
        
        
        #contains restrictions that the user has to follow while
        #in a/ clicking on a particular region
        #depends on the 'map' and 'list of regions'
    
    def list_of_regions(self):
        region_list = wx.Panel(self, pos = (410, 400), size = (390, 300),
                              style = wx.SIMPLE_BORDER,name = 'region_list')
        global list_box_regions
        list_box_regions = wx.ListBox(region_list, wx.ID_ANY, size = (390, 300),
                                      style = wx.LB_SINGLE)
        title = wx.StaticText(region_list,-1, 'List Of Regions', style = wx.ALIGN_TOP)
        BIG_BOX.Add(region_list, proportion = 1,flag = wx.ALL|wx.EXPAND,
                    border = 10)
        title = wx.StaticText(region_list,-1, 'List Of Regions', style = wx.ALIGN_TOP)
        
        for region in self.proj.rfi.regions:
            if not (region.isObstacle or region.name.lower() == "boundary"):
                list_box_regions.Append(region.name)
                
        #extract list of regions from the region file
    
    
    
    def guidelines(self):
        guidelines = wx.Panel(self, pos = (810, 400), size = (390, 300),
                              style = wx.SIMPLE_BORDER,name = 'Guidelines')
        BIG_BOX.Add(guidelines, proportion = 1,flag = wx.ALL|wx.EXPAND,
                    border = 10)
        title = wx.StaticText(guidelines,-1, 'Guidelines', style = wx.ALIGN_TOP)
    
    
if __name__ == '__main__':
    main_gui = wx.PySimpleApp()
    output = Display(None, -1, "Display Screen")
    output.Centre()
    output.Show()
    main_gui.MainLoop()
