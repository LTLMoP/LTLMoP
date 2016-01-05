#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ====================================
    specEditor.py - Specification Editor
    ====================================

    A development environment for specifications written in structured English,
    allowing for editing, compilation, and execution/simulation
"""

import re, sys, os, subprocess
import wx, wx.richtext, wx.stc
import threading
import time


# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))
sys.path.append(os.path.join(p,"lib","cores"))

from regions import *
import project
import strategy
import mapRenderer
from specCompiler import SpecCompiler
from asyncProcesses import AsynchronousProcessThread

from copy import deepcopy

import logging
import globalConfig


######################### WARNING! ############################
#         DO NOT EDIT GUI CODE BY HAND.  USE WXGLADE.         #
#   The .wxg file is located in the etc/wxglade/ directory.   #
###############################################################

class WxAsynchronousProcessThread(AsynchronousProcessThread):
    """ Make sure callbacks from AsynchronousProcessThreads are
        safe for threading in wx by wrapping with a wx.CallAfter. """

    def __init__(self, cmd, callback, logFunction):
        def wxSafeCallback(*args, **kwds):
            wx.CallAfter(callback, *args, **kwds)

        if logFunction is None:
            wxSafeLogger = None
        else:
            def wxSafeLogger(*args, **kwds):
                wx.CallAfter(logFunction, *args, **kwds)

        super(WxAsynchronousProcessThread, self).__init__(cmd, wxSafeCallback, wxSafeLogger)

class AnalysisResultsDialog(wx.Dialog):
    def __init__(self, parent, *args, **kwds):
        # begin wxGlade: AnalysisResultsDialog.__init__
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER | wx.THICK_FRAME
        wx.Dialog.__init__(self, *args, **kwds)
        self.label_3 = wx.StaticText(self, wx.ID_ANY, "Analysis Output:")
        self.text_ctrl_summary = wx.richtext.RichTextCtrl(self, wx.ID_ANY, "", style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.button_refine = wx.Button(self, wx.ID_ANY, "Refine analysis")
        self.label_traceback = wx.StaticText(self, wx.ID_ANY, "SLURP Traceback:")
        self.tree_ctrl_traceback = wx.TreeCtrl(self, wx.ID_ANY, style=wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_FULL_ROW_HIGHLIGHT | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER)
        self.button_1 = wx.Button(self, wx.ID_CLOSE, "")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.onClickRefineAnalysis, self.button_refine)
        self.Bind(wx.EVT_BUTTON, self.onButtonClose, self.button_1)
        # end wxGlade

        self.parent = parent
        self.statements = {}
        self.statements["env"] = []
        self.statements["sys"] = []

    def __set_properties(self):
        # begin wxGlade: AnalysisResultsDialog.__set_properties
        self.SetTitle("Analysis Results")
        self.SetSize((562, 602))
        self.label_traceback.Hide()
        self.tree_ctrl_traceback.Hide()
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: AnalysisResultsDialog.__do_layout
        sizer_11 = wx.BoxSizer(wx.VERTICAL)
        sizer_16 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_11.Add(self.label_3, 0, wx.ALL, 5)
        sizer_11.Add(self.text_ctrl_summary, 1, wx.ALL | wx.EXPAND, 5)
        sizer_11.Add(self.button_refine, 0, wx.ALL | wx.ALIGN_RIGHT | wx.ALIGN_CENTER_VERTICAL, 5)
        sizer_11.Add(self.label_traceback, 0, wx.ALL, 5)
        sizer_11.Add(self.tree_ctrl_traceback, 3, wx.ALL | wx.EXPAND, 5)
        sizer_16.Add((20, 20), 1, wx.EXPAND, 0)
        sizer_16.Add(self.button_1, 0, wx.ALL | wx.EXPAND, 5)
        sizer_11.Add(sizer_16, 0, wx.EXPAND, 0)
        self.SetSizer(sizer_11)
        self.Layout()
        # end wxGlade

    def populateTree(self, gentree):
        # Create the root
        self.statements["env"] = []
        self.statements["sys"] = []
        self.tree_ctrl_traceback.DeleteAllItems()
        root_node = self.tree_ctrl_traceback.AddRoot("Root")

        # Build the traceback tree
        for user_input, command_tree in gentree.items():
            # Add a node for each input line
            input_node = self.tree_ctrl_traceback.AppendItem(root_node, user_input)
            # Then fill in the tree for each command it generates
            for command, spec_chunks in command_tree.items():
                # Add a node fot the  command
                command_node = self.tree_ctrl_traceback.AppendItem(input_node, command)
                # Fill in the explanation and lines for each list of lines created
                for spec_chunk in spec_chunks:
                    explanation_node = \
                        self.tree_ctrl_traceback.AppendItem(command_node, spec_chunk.explanation)
                    for stmt, highlight in zip(spec_chunk.lines, spec_chunk.highlights):
                        stmt_node = self.tree_ctrl_traceback.AppendItem(explanation_node, stmt)
                        if spec_chunk.issys():
                            self.statements["sys"].append((stmt, stmt_node))
                        else:
                            self.statements["env"].append((stmt, stmt_node))

                        if highlight:
                            if "[]<>" in stmt:
                                # Goals colored differently than other statements
                                self.tree_ctrl_traceback.SetItemBackgroundColour(stmt_node, "#FA58D0") # pale pink
                            else:
                                self.tree_ctrl_traceback.SetItemBackgroundColour(stmt_node, "#FE9A2E") # pale orange
                            # Expand the relevant nodes
                            self.tree_ctrl_traceback.Expand(input_node)
                            self.tree_ctrl_traceback.Expand(command_node)
                            self.tree_ctrl_traceback.Expand(explanation_node)
                            self.tree_ctrl_traceback.Expand(stmt_node)

        self.Layout()


    def markFragments(self, agent, section, jx=None):
        jx_this = -1 # debug output is 0-indexed

        for f,obj in self.statements[agent]:
            if "[]<>" in f:
                ftype = "goals"
                jx_this += 1
            elif "[]" in f:
                ftype = "trans"
            else:
                ftype = "init"

            if ftype == section:
                if section == "goals" and jx_this == jx:
                    self.tree_ctrl_traceback.SetItemBackgroundColour(obj,"#FA58D0") # pale pink
                elif section != "goals":
                    self.tree_ctrl_traceback.SetItemBackgroundColour(obj,"#FE9A2E") # pale orange
                self.tree_ctrl_traceback.Expand(obj)

    def appendLog(self, text, color="BLACK"):
        self.text_ctrl_summary.BeginTextColour(color)
        self.text_ctrl_summary.WriteText(text)
        self.text_ctrl_summary.EndTextColour()
        self.text_ctrl_summary.ShowPosition(self.text_ctrl_summary.GetLastPosition())
        wx.GetApp().Yield(True) # Ensure update

    def onButtonClose(self, event): # wxGlade: AnalysisResultsDialog.<event_handler>
        self.Hide()
        event.Skip()

    def onClickRefineAnalysis(self, event):  # wxGlade: AnalysisResultsDialog.<event_handler>
        self.parent.refineAnalysis()
        event.Skip()

# end of class AnalysisResultsDialog

class MapDialog(wx.Dialog):
    """
    A simple little dialog that displays the regions on top of the map so that you can
    select a region visually instead of just choosing the name.
    """

    # FIXME: Doesn't scroll on Windows???

    def __init__(self, parent, *args, **kwds):
        # begin wxGlade: MapDialog.__init__
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE
        wx.Dialog.__init__(self, *args, **kwds)
        self.panel_2 = wx.ScrolledWindow(self, wx.ID_ANY, style=wx.TAB_TRAVERSAL)

        self.__set_properties()
        self.__do_layout()
        # end wxGlade

        self.parent = parent

    def __set_properties(self):
        # begin wxGlade: MapDialog.__set_properties
        self.SetTitle("Select Region...")
        self.SetSize((932, 709))
        self.panel_2.SetScrollRate(10, 10)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MapDialog.__do_layout
        sizer_10 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_10.Add(self.panel_2, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_10)
        self.Layout()
        self.Centre()
        # end wxGlade

        self.panel_2.SetBackgroundColour(wx.WHITE)
        self.panel_2.Bind(wx.EVT_PAINT, self.drawMap)

        # Bind to catch mouse clicks!
        self.panel_2.Bind(wx.EVT_LEFT_DOWN, self.onMapClick)

    def drawMap(self, event):
        mapRenderer.drawMap(self.panel_2, self.parent.proj.rfi, scaleToFit=False)

    def onMapClick(self, event):
        x, y = self.panel_2.CalcUnscrolledPosition(event.GetX(), event.GetY())
        for region in self.parent.proj.rfi.regions:
            if region.name.lower() != "boundary" and region.objectContainsPoint(x, y):
                self.parent.text_ctrl_spec.AppendText(region.name)
                self.Close()
                break

        event.Skip()

# end of class MapDialog


class SpecEditorFrame(wx.Frame):
    """
    The main application window!
    """


    def __init__(self, *args, **kwds):
        # begin wxGlade: SpecEditorFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)

        # Menu Bar
        self.frame_1_menubar = wx.MenuBar()
        global MENU_IMPORT_REGION; MENU_IMPORT_REGION = wx.NewId()
        global MENU_COMPILE; MENU_COMPILE = wx.NewId()
        global MENU_COMPILECONFIG; MENU_COMPILECONFIG = wx.NewId()
        global MENU_CONVEXIFY; MENU_CONVEXIFY = wx.NewId()
        global MENU_FASTSLOW; MENU_FASTSLOW = wx.NewId()
        global MENU_BITVECTOR; MENU_BITVECTOR = wx.NewId()
        global MENU_PARSERMODE; MENU_PARSERMODE = wx.NewId()
        global MENU_PARSERMODE_SLURP; MENU_PARSERMODE_SLURP = wx.NewId()
        global MENU_PARSERMODE_STRUCTURED; MENU_PARSERMODE_STRUCTURED = wx.NewId()
        global MENU_PARSERMODE_LTL; MENU_PARSERMODE_LTL = wx.NewId()
        global MENU_SYNTHESIZER; MENU_SYNTHESIZER = wx.NewId()
        global MENU_SYNTHESIZER_JTLV; MENU_SYNTHESIZER_JTLV = wx.NewId()
        global MENU_SYNTHESIZER_SLUGS; MENU_SYNTHESIZER_SLUGS = wx.NewId()
        global MENU_SYMBOLIC; MENU_SYMBOLIC = wx.NewId()
        global MENU_SIMULATE; MENU_SIMULATE = wx.NewId()
        global MENU_SIMCONFIG; MENU_SIMCONFIG = wx.NewId()
        global MENU_ANALYZE; MENU_ANALYZE = wx.NewId()
        global MENU_DOTTY; MENU_DOTTY = wx.NewId()
        global MENU_MOPSY; MENU_MOPSY = wx.NewId()
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_NEW, "&New\tCtrl-N", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_OPEN, "&Open...\tCtrl-O", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_IMPORT_REGION, "Import &Region File...\tCtrl-R", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_CLOSE, "&Close\tCtrl-W", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_SAVE, "&Save\tCtrl-S", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_SAVEAS, "Save &As...", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_EXIT, "&Quit\tCtrl-Q", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&File")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_UNDO, "&Undo\tCtrl-Z", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_REDO, "&Redo\tCtrl-Shift-Z", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.AppendSeparator()
        wxglade_tmp_menu.Append(wx.ID_CUT, "Cu&t\tCtrl-X", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_COPY, "&Copy\tCtrl-C", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(wx.ID_PASTE, "&Paste\tCtrl-V", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Edit")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(MENU_COMPILE, "&Compile\tF5", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu_sub = wx.Menu()
        wxglade_tmp_menu_sub.Append(MENU_CONVEXIFY, "Decompose workspace into convex regions", "", wx.ITEM_CHECK)
        wxglade_tmp_menu_sub.Append(MENU_FASTSLOW, "Enable \"fast-slow\" synthesis", "", wx.ITEM_CHECK)
        wxglade_tmp_menu_sub.Append(MENU_BITVECTOR, "Use bit-vector region encoding", "", wx.ITEM_CHECK)
        wxglade_tmp_menu_sub_sub = wx.Menu()
        wxglade_tmp_menu_sub_sub.Append(MENU_PARSERMODE_SLURP, "SLURP (NL)", "", wx.ITEM_RADIO)
        wxglade_tmp_menu_sub_sub.Append(MENU_PARSERMODE_STRUCTURED, "Structured English", "", wx.ITEM_RADIO)
        wxglade_tmp_menu_sub_sub.Append(MENU_PARSERMODE_LTL, "LTL", "", wx.ITEM_RADIO)
        wxglade_tmp_menu_sub.AppendMenu(MENU_PARSERMODE, "Parser mode", wxglade_tmp_menu_sub_sub, "")
        wxglade_tmp_menu_sub_sub = wx.Menu()
        wxglade_tmp_menu_sub_sub.Append(MENU_SYNTHESIZER_JTLV, "JTLV", "", wx.ITEM_RADIO)
        wxglade_tmp_menu_sub_sub.Append(MENU_SYNTHESIZER_SLUGS, "Slugs", "", wx.ITEM_RADIO)
        wxglade_tmp_menu_sub.AppendMenu(MENU_SYNTHESIZER, "Synthesizer", wxglade_tmp_menu_sub_sub, "")
        wxglade_tmp_menu_sub.Append(MENU_SYMBOLIC, "Use symbolic strategy", "", wx.ITEM_CHECK)
        wxglade_tmp_menu.AppendMenu(MENU_COMPILECONFIG, "Compilation options", wxglade_tmp_menu_sub, "")
        wxglade_tmp_menu.AppendSeparator()
        wxglade_tmp_menu.Append(MENU_SIMULATE, "&Simulate\tF6", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_SIMCONFIG, "Confi&gure Simulation...\tF7", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Run")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(MENU_ANALYZE, "&Analyze\tF8", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_DOTTY, "View &Automaton\tF9", "", wx.ITEM_NORMAL)
        wxglade_tmp_menu.Append(MENU_MOPSY, "&Visualize Counterstrategy...\tF10", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Debug")
        wxglade_tmp_menu = wx.Menu()
        wxglade_tmp_menu.Append(wx.ID_ABOUT, "&About Specification Editor...", "", wx.ITEM_NORMAL)
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Help")
        self.SetMenuBar(self.frame_1_menubar)
        # Menu Bar end
        self.window_1 = wx.SplitterWindow(self, wx.ID_ANY, style=wx.SP_3D | wx.SP_BORDER | wx.SP_LIVE_UPDATE)
        self.window_1_pane_1 = wx.Panel(self.window_1, wx.ID_ANY)
        self.panel_1 = wx.ScrolledWindow(self.window_1_pane_1, wx.ID_ANY, style=wx.TAB_TRAVERSAL)
        self.label_1 = wx.StaticText(self.panel_1, wx.ID_ANY, "Regions:")
        self.list_box_regions = wx.ListBox(self.panel_1, wx.ID_ANY, choices=[], style=wx.LB_SINGLE)
        self.button_map = wx.Button(self.panel_1, wx.ID_ANY, "Select from Map...")
        self.button_edit_regions = wx.Button(self.panel_1, wx.ID_ANY, "Edit Regions...")
        self.label_1_copy = wx.StaticText(self.panel_1, wx.ID_ANY, "Sensors:")
        self.list_box_sensors = wx.CheckListBox(self.panel_1, wx.ID_ANY, choices=[], style=wx.LB_SINGLE)
        self.button_sensor_add = wx.Button(self.panel_1, wx.ID_ADD, "")
        self.button_sensor_remove = wx.Button(self.panel_1, wx.ID_REMOVE, "")
        self.label_1_copy_1 = wx.StaticText(self.panel_1, wx.ID_ANY, "Actions:")
        self.list_box_actions = wx.CheckListBox(self.panel_1, wx.ID_ANY, choices=[], style=wx.LB_SINGLE)
        self.button_actuator_add = wx.Button(self.panel_1, wx.ID_ADD, "")
        self.button_actuator_remove = wx.Button(self.panel_1, wx.ID_REMOVE, "")
        self.label_1_copy_2 = wx.StaticText(self.panel_1, wx.ID_ANY, "Custom Propositions:")
        self.list_box_customs = wx.ListBox(self.panel_1, wx.ID_ANY, choices=[], style=wx.LB_SINGLE)
        self.button_custom_add = wx.Button(self.panel_1, wx.ID_ADD, "")
        self.button_custom_remove = wx.Button(self.panel_1, wx.ID_REMOVE, "")
        self.window_1_pane_2 = wx.Panel(self.window_1, wx.ID_ANY)
        self.notebook_1 = wx.Notebook(self.window_1_pane_2, wx.ID_ANY, style=0)
        self.notebook_1_pane_1 = wx.Panel(self.notebook_1, wx.ID_ANY)
        self.text_ctrl_log = wx.richtext.RichTextCtrl(self.notebook_1_pane_1, wx.ID_ANY, "", style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.notebook_1_pane_2 = wx.Panel(self.notebook_1, wx.ID_ANY)
        self.text_ctrl_LTL = wx.TextCtrl(self.notebook_1_pane_2, wx.ID_ANY, "", style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.notebook_1_pane_3 = wx.Panel(self.notebook_1, wx.ID_ANY)
        self.label_locphrases = wx.StaticText(self.notebook_1_pane_3, wx.ID_ANY, "Active locative phrases:")
        self.list_box_locphrases = wx.ListBox(self.notebook_1_pane_3, wx.ID_ANY, choices=[], style=wx.LB_ALWAYS_SB)
        self.checkbox_regionlabel = wx.CheckBox(self.notebook_1_pane_3, wx.ID_ANY, "Show region names")
        self.checkbox_regionlabelbits = wx.CheckBox(self.notebook_1_pane_3, wx.ID_ANY, "Include bit-vector representations")
        self.panel_locmap = wx.Panel(self.notebook_1_pane_3, wx.ID_ANY, style=wx.SUNKEN_BORDER | wx.TAB_TRAVERSAL | wx.FULL_REPAINT_ON_RESIZE)

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_MENU, self.onMenuNew, id=wx.ID_NEW)
        self.Bind(wx.EVT_MENU, self.onMenuOpen, id=wx.ID_OPEN)
        self.Bind(wx.EVT_MENU, self.onImportRegion, id=MENU_IMPORT_REGION)
        self.Bind(wx.EVT_MENU, self.doClose, id=wx.ID_CLOSE)
        self.Bind(wx.EVT_MENU, self.onMenuSave, id=wx.ID_SAVE)
        self.Bind(wx.EVT_MENU, self.onMenuSaveAs, id=wx.ID_SAVEAS)
        self.Bind(wx.EVT_MENU, self.onMenuQuit, id=wx.ID_EXIT)
        self.Bind(wx.EVT_MENU, self.onMenuUndo, id=wx.ID_UNDO)
        self.Bind(wx.EVT_MENU, self.onMenuRedo, id=wx.ID_REDO)
        self.Bind(wx.EVT_MENU, self.onMenuCut, id=wx.ID_CUT)
        self.Bind(wx.EVT_MENU, self.onMenuCopy, id=wx.ID_COPY)
        self.Bind(wx.EVT_MENU, self.onMenuPaste, id=wx.ID_PASTE)
        self.Bind(wx.EVT_MENU, self.onMenuCompile, id=MENU_COMPILE)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_CONVEXIFY)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_FASTSLOW)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_BITVECTOR)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_PARSERMODE_SLURP)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_PARSERMODE_STRUCTURED)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_PARSERMODE_LTL)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_SYNTHESIZER_JTLV)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_SYNTHESIZER_SLUGS)
        self.Bind(wx.EVT_MENU, self.onMenuSetCompileOptions, id=MENU_SYMBOLIC)
        self.Bind(wx.EVT_MENU, self.onMenuSimulate, id=MENU_SIMULATE)
        self.Bind(wx.EVT_MENU, self.onMenuConfigSim, id=MENU_SIMCONFIG)
        self.Bind(wx.EVT_MENU, self.onMenuAnalyze, id=MENU_ANALYZE)
        self.Bind(wx.EVT_MENU, self.onMenuViewAut, id=MENU_DOTTY)
        self.Bind(wx.EVT_MENU, self.onMenuMopsy, id=MENU_MOPSY)
        self.Bind(wx.EVT_MENU, self.onMenuAbout, id=wx.ID_ABOUT)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_regions)
        self.Bind(wx.EVT_BUTTON, self.onMapSelect, self.button_map)
        self.Bind(wx.EVT_BUTTON, self.onClickEditRegions, self.button_edit_regions)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_sensors)
        self.Bind(wx.EVT_BUTTON, self.onPropAdd, self.button_sensor_add)
        self.Bind(wx.EVT_BUTTON, self.onPropRemove, self.button_sensor_remove)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_actions)
        self.Bind(wx.EVT_BUTTON, self.onPropAdd, self.button_actuator_add)
        self.Bind(wx.EVT_BUTTON, self.onPropRemove, self.button_actuator_remove)
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.onPropositionDblClick, self.list_box_customs)
        self.Bind(wx.EVT_BUTTON, self.onPropAdd, self.button_custom_add)
        self.Bind(wx.EVT_BUTTON, self.onPropRemove, self.button_custom_remove)
        self.Bind(wx.EVT_LISTBOX, self.onLocPhraseSelect, self.list_box_locphrases)
        self.Bind(wx.EVT_CHECKBOX, self.onRegionLabelStyleChange, self.checkbox_regionlabel)
        self.Bind(wx.EVT_CHECKBOX, self.onRegionLabelStyleChange, self.checkbox_regionlabelbits)
        # end wxGlade

        # Listen for checkbox toggles
        self.Bind(wx.EVT_CHECKLISTBOX, self.onPropToggle)

        # Create the StyledTextControl for the specification area manually, since wxGlade doesn't support it
        self.text_ctrl_spec = wx.stc.StyledTextCtrl(self.window_1_pane_1, -1, style=wx.TE_PROCESS_ENTER|wx.TE_PROCESS_TAB|wx.TE_MULTILINE|wx.WANTS_CHARS)
        self.window_1_pane_1.GetSizer().Insert(0, self.text_ctrl_spec, 2, wx.EXPAND, 0)
        self.window_1_pane_1.GetSizer().Layout()

        self.text_ctrl_spec.SetMarginWidth(0, 40)
        self.text_ctrl_spec.SetMarginType(0, wx.stc.STC_MARGIN_NUMBER)
        self.text_ctrl_spec.SetMarginType(1, wx.stc.STC_MARGIN_SYMBOL)

        global MARKER_INIT, MARKER_SAFE, MARKER_LIVE, MARKER_PARSEERROR
        MARKER_INIT, MARKER_SAFE, MARKER_LIVE, MARKER_PARSEERROR = range(4)
        self.text_ctrl_spec.MarkerDefine(MARKER_INIT,wx.stc.STC_MARK_ARROW,"white","red")
        self.text_ctrl_spec.MarkerDefine(MARKER_SAFE,wx.stc.STC_MARK_ARROW,"white","blue")
        self.text_ctrl_spec.MarkerDefine(MARKER_LIVE,wx.stc.STC_MARK_ARROW,"white","green")
        self.text_ctrl_spec.MarkerDefine(MARKER_PARSEERROR,wx.stc.STC_MARK_BACKGROUND,"red","red")

        self.text_ctrl_spec.SetLexer(wx.stc.STC_LEX_CONTAINER)
        self.text_ctrl_spec.Bind(wx.stc.EVT_STC_STYLENEEDED, self.onStyleNeeded)
        self.text_ctrl_spec.StyleSetFont(wx.stc.STC_P_DEFAULT, wx.Font(12, wx.SWISS, wx.NORMAL, wx.NORMAL, False, u'Consolas'))
        self.text_ctrl_spec.StyleSetFont(wx.stc.STC_P_COMMENTLINE, wx.Font(12, wx.SWISS, wx.NORMAL, wx.BOLD, False, u'Consolas'))
        self.text_ctrl_spec.StyleSetForeground(wx.stc.STC_P_COMMENTLINE, wx.Colour(0, 200, 0))
        self.text_ctrl_spec.StyleSetFont(wx.stc.STC_P_WORD, wx.Font(12, wx.SWISS, wx.NORMAL, wx.BOLD, False, u'Consolas'))
        self.text_ctrl_spec.StyleSetForeground(wx.stc.STC_P_WORD, wx.BLUE)
        self.text_ctrl_spec.StyleSetFont(wx.stc.STC_P_STRING, wx.Font(12, wx.SWISS, wx.NORMAL, wx.BOLD, False, u'Consolas'))
        self.text_ctrl_spec.StyleSetForeground(wx.stc.STC_P_STRING, wx.Colour(200, 200, 0))

        # Don't enable tooltips on wx2.9.5 on OS X
        # Temporary workaround for http://trac.wxwidgets.org/ticket/15508
        if wx.version() != '2.9.5.0 osx-cocoa (classic)':
            self.text_ctrl_spec.SetMouseDwellTime(500)
            self.text_ctrl_spec.Bind(wx.stc.EVT_STC_DWELLSTART, self.onMouseDwellStart)
            self.text_ctrl_spec.Bind(wx.stc.EVT_STC_DWELLEND, self.onMouseDwellEnd)

        self.text_ctrl_spec.SetWrapMode(wx.stc.STC_WRAP_WORD)

        #self.text_ctrl_spec.SetEOLMode(wx.stc.STC_EOL_LF)

        # Listen for changes to the text (except for style and marker changes)
        self.text_ctrl_spec.SetModEventMask(self.text_ctrl_spec.GetModEventMask() \
                                            & ~(wx.stc.STC_MOD_CHANGESTYLE | wx.stc.STC_MOD_CHANGEMARKER))
        self.Bind(wx.stc.EVT_STC_CHANGE, self.onSpecTextChange, self.text_ctrl_spec)

        # Set up locative phrase map
        self.panel_locmap.SetBackgroundColour(wx.WHITE)
        self.panel_locmap.Bind(wx.EVT_PAINT, self.drawLocMap)

        # Set up extra event bindings
        self.Bind(wx.EVT_CLOSE, self.doClose)

        # Null the subprocess values
        self.subprocess = { "Region Editor": None,
                            "Dotty": None,
                            "Simulation Configuration": None }

        self.initializeNewSpec()

    def onMouseDwellStart(self, event):
        pos = event.GetPosition()
        line = self.text_ctrl_spec.LineFromPosition(pos) # 0-indexed

        if pos == -1:
            return

        if self.response is not None:
            self.text_ctrl_spec.CallTipShow( pos, "Response: {}".format(self.response[line]))

    def onMouseDwellEnd(self, event):
        if self.text_ctrl_spec.CallTipActive():
            self.text_ctrl_spec.CallTipCancel()

    def initializeNewSpec(self):
        # Initialize values
        self.mapDialog = None
        self.analysisDialog = None
        self.tracebackTree = None
        self.to_highlight = None
        self.response = None
        self.proj = project.Project()
        self.decomposedRFI = None

        # Reset GUI
        self.button_map.Enable(False)
        self.button_sensor_remove.Enable(False)
        self.button_actuator_remove.Enable(False)
        self.button_custom_remove.Enable(False)
        self.list_box_regions.Clear()
        self.list_box_sensors.Clear()
        self.list_box_actions.Clear()
        self.list_box_customs.Clear()
        self.list_box_locphrases.Clear()
        self.text_ctrl_spec.SetText("")
        self.text_ctrl_spec.EmptyUndoBuffer()
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_INIT)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_SAFE)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_LIVE)
        self.text_ctrl_log.Clear()

        # Set options menu status based on proj options
        self.updateMenusFromProjectOptions()

        self.SetTitle("Specification Editor - Untitled")

        self.dirty = False

    def onStyleNeeded(self, e):
        start = self.text_ctrl_spec.GetEndStyled()    # this is the first character that needs styling
        end = e.GetPosition()          # this is the last character that needs styling

        # Move back to the beginning of the line because apparently we aren't guaranteed to process line-wise chunks
        while start > 0 and chr(self.text_ctrl_spec.GetCharAt(start-1)) != "\n":
            start -= 1

        # Set everything to the default style
        self.text_ctrl_spec.StartStyling(start, 31)  # 31 is the default mask
        self.text_ctrl_spec.SetStyling(end-start, wx.stc.STC_P_DEFAULT)

        # Find propositions
        text = self.text_ctrl_spec.GetTextRange(start,end)
        for m in re.finditer("|".join(map(lambda s: "\\b%s\\b"%s, self.proj.enabled_sensors + self.proj.enabled_actuators + self.proj.all_customs + self.list_box_regions.GetItems())), text, re.MULTILINE):
            self.text_ctrl_spec.StartStyling(start+m.start(), 31)
            self.text_ctrl_spec.SetStyling(m.end()-m.start(), wx.stc.STC_P_WORD)

        # Find groups
        # TODO: Don't search the whole document each time...
        all_text = self.text_ctrl_spec.GetText()
        group_names = re.findall("^group\s+(\w*?)\s+is", all_text, re.MULTILINE | re.IGNORECASE)
        # Allow for singular references too
        group_names += [n[:-1] for n in group_names if n.endswith("s")]
        for m in re.finditer("|".join(map(lambda s: "\\b%s\\b"%s, group_names)), text, re.MULTILINE):
            self.text_ctrl_spec.StartStyling(start+m.start(), 31)
            self.text_ctrl_spec.SetStyling(m.end()-m.start(), wx.stc.STC_P_STRING)

        # Find comment lines
        text = self.text_ctrl_spec.GetTextRange(start,end)
        for m in re.finditer("^#.*?$", text, re.MULTILINE):
            self.text_ctrl_spec.StartStyling(start+m.start(), 31)
            self.text_ctrl_spec.SetStyling(m.end()-m.start(), wx.stc.STC_P_COMMENTLINE)

        # Perform a no-op style operation on the entire section of text to ensure that
        # Scintilla knows we've styled to the end of the section.  It's not clear why this is
        # necessary, as it should have been taken care of by the STC_P_DEFAULT styling above
        # but if this is not here this event can get thrown over and over, causing flickering.
        # (It appears that only the end-point of the last set-styling call affects Scintilla's internal
        # style-needed pointer?)
        self.text_ctrl_spec.StartStyling(start, 0)
        self.text_ctrl_spec.SetStyling(end-start, 0)

    def __set_properties(self):
        # begin wxGlade: SpecEditorFrame.__set_properties
        self.SetTitle("Specification Editor - Untitled")
        self.SetSize((929, 700))
        self.button_map.Enable(False)
        self.list_box_sensors.SetMinSize((123, 75))
        self.button_sensor_remove.Enable(False)
        self.list_box_actions.SetMinSize((123, 75))
        self.button_actuator_remove.Enable(False)
        self.list_box_customs.SetMinSize((123, 75))
        self.button_custom_remove.Enable(False)
        self.panel_1.SetScrollRate(10, 10)
        self.checkbox_regionlabel.SetValue(1)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: SpecEditorFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_14 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_15 = wx.BoxSizer(wx.VERTICAL)
        sizer_9 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_3 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.VERTICAL)
        sizer_8 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_11 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_6 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_7 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5.Add(self.label_1, 0, wx.LEFT | wx.TOP | wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_regions, 2, wx.LEFT | wx.EXPAND, 4)
        sizer_7.Add(self.button_map, 0, wx.TOP, 5)
        sizer_7.Add((5, 20), 0, 0, 0)
        sizer_7.Add(self.button_edit_regions, 0, wx.TOP, 5)
        sizer_5.Add(sizer_7, 0, wx.LEFT | wx.EXPAND, 4)
        sizer_5.Add(self.label_1_copy, 0, wx.LEFT | wx.TOP | wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_sensors, 2, wx.LEFT | wx.EXPAND, 4)
        sizer_6.Add(self.button_sensor_add, 0, wx.TOP, 5)
        sizer_6.Add((5, 20), 0, 0, 0)
        sizer_6.Add(self.button_sensor_remove, 0, wx.TOP, 5)
        sizer_5.Add(sizer_6, 0, wx.LEFT | wx.EXPAND, 4)
        sizer_5.Add(self.label_1_copy_1, 0, wx.LEFT | wx.TOP | wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_actions, 2, wx.LEFT | wx.EXPAND, 4)
        sizer_11.Add(self.button_actuator_add, 0, wx.TOP, 5)
        sizer_11.Add((5, 20), 0, 0, 0)
        sizer_11.Add(self.button_actuator_remove, 0, wx.TOP, 5)
        sizer_5.Add(sizer_11, 0, wx.LEFT | wx.EXPAND, 6)
        sizer_5.Add(self.label_1_copy_2, 0, wx.LEFT | wx.TOP | wx.BOTTOM, 4)
        sizer_5.Add(self.list_box_customs, 2, wx.LEFT | wx.EXPAND, 4)
        sizer_8.Add(self.button_custom_add, 0, wx.TOP, 5)
        sizer_8.Add((5, 20), 0, 0, 0)
        sizer_8.Add(self.button_custom_remove, 0, wx.TOP, 5)
        sizer_5.Add(sizer_8, 0, wx.LEFT | wx.EXPAND, 4)
        self.panel_1.SetSizer(sizer_5)
        sizer_4.Add(self.panel_1, 1, wx.EXPAND, 0)
        self.window_1_pane_1.SetSizer(sizer_4)
        sizer_3.Add(self.text_ctrl_log, 1, wx.ALL | wx.EXPAND, 2)
        self.notebook_1_pane_1.SetSizer(sizer_3)
        sizer_9.Add(self.text_ctrl_LTL, 1, wx.EXPAND, 0)
        self.notebook_1_pane_2.SetSizer(sizer_9)
        sizer_14.Add((5, 20), 0, 0, 0)
        sizer_15.Add((20, 20), 0, 0, 0)
        sizer_15.Add(self.label_locphrases, 0, 0, 0)
        sizer_15.Add(self.list_box_locphrases, 1, wx.EXPAND, 0)
        sizer_15.Add((20, 20), 0, 0, 0)
        sizer_15.Add(self.checkbox_regionlabel, 0, wx.EXPAND, 0)
        sizer_15.Add(self.checkbox_regionlabelbits, 0, wx.LEFT | wx.EXPAND, 20)
        sizer_15.Add((20, 20), 0, 0, 0)
        sizer_14.Add(sizer_15, 1, wx.EXPAND, 0)
        sizer_14.Add((5, 20), 0, 0, 0)
        sizer_14.Add(self.panel_locmap, 2, wx.EXPAND, 0)
        self.notebook_1_pane_3.SetSizer(sizer_14)
        self.notebook_1.AddPage(self.notebook_1_pane_1, "Compiler Log")
        self.notebook_1.AddPage(self.notebook_1_pane_2, "LTL Output")
        self.notebook_1.AddPage(self.notebook_1_pane_3, "Workspace Decomposition")
        sizer_2.Add(self.notebook_1, 1, wx.EXPAND, 0)
        self.window_1_pane_2.SetSizer(sizer_2)
        self.window_1.SplitHorizontally(self.window_1_pane_1, self.window_1_pane_2, 425)
        sizer_1.Add(self.window_1, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        self.Centre()
        # end wxGlade

        # Make it so that the log window doesn't change height when the window is resized
        # NOTE: May not work on older versions of wxWidgets
        self.window_1.SetSashGravity(1.0)

        self.window_1.SetMinimumPaneSize(100)

    def drawLocMap(self, event):
        """ Respond to a request to redraw the contents of the decomposed map
        """

        # Nothing to draw if no decomposed regions have been generated
        if self.decomposedRFI is None:
            return

        highlightList = self.proj.regionMapping[self.list_box_locphrases.GetStringSelection()]

        # TODO: Advise the user that the decomposed map may be inaccurate if
        #  mtime(spec)>mtime(aut) or spectext is dirty

        mapRenderer.drawMap(self.panel_locmap, self.decomposedRFI, scaleToFit=True, drawLabels=self.checkbox_regionlabel.GetValue(), highlightList=highlightList, showBits=self.checkbox_regionlabelbits.GetValue())

    def onPropositionDblClick(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Add the proposition name to the spec when you double click the name
        """

        caller = event.GetEventObject()
        if caller in [self.list_box_sensors, self.list_box_actions] \
           and not caller.IsChecked(caller.GetSelection()):
            # Only allow adding of enabled propositions
            return

        self.text_ctrl_spec.InsertText(self.text_ctrl_spec.GetCurrentPos(), caller.GetStringSelection())

        event.Skip()

    def onPropToggle(self, event):
        caller = event.GetEventObject()

        if caller is self.list_box_sensors:
            self.proj.enabled_sensors = list(self.list_box_sensors.GetCheckedStrings())
        elif caller is self.list_box_actions:
            self.proj.enabled_actuators = list(self.list_box_actions.GetCheckedStrings())

        self.dirty = True

        # Force document relexing
        self.text_ctrl_spec.Colourise(0,self.text_ctrl_spec.GetTextLength())

    def onSpecTextChange(self, event):
        # If there are any error markers, clear them
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_INIT)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_SAFE)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_LIVE)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_PARSEERROR)

        # Stop showing response tooltips, too
        self.response = None

        self.proj.specText = self.text_ctrl_spec.GetText()

        self.dirty = True

    def onMapSelect(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Show the map with overlayed regions so that the user can select a region name visually.
        """

        self.mapDialog.Show()

        event.Skip()

    def onImportRegion(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a region file and then import it.
        """
        filename = wx.FileSelector("Import Region File", default_extension="regions",
                                  default_path=".",
                                  wildcard="Region files (*.regions)|*.regions",
                                  flags = wx.OPEN | wx.FILE_MUST_EXIST)
        if filename == "": return

        rfi = RegionFileInterface()

        # Try loading the file
        if not rfi.readFile(filename):
            wx.MessageBox("Cannot open region file %s" % (filename), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            self.rfi = None
            return

        # Loading succeeded, so bring the rfi into our project
        self.proj.rfi = rfi

        # If we are working with an unsaved spec, assume everything is in the same dir
        # as the regions file
        if self.proj.project_root is None:
            self.proj.project_root = os.path.dirname(os.path.abspath(filename))

        self.dirty = True

        self.updateFromRFI()

    def updateFromRFI(self):
        """
        Update the GUI to reflect a newly loaded region file
        """

        # Add the regions to the region listbox
        self.list_box_regions.Clear()
        for region in self.proj.rfi.regions:
            if not (region.isObstacle or region.name.lower() == "boundary"):
                self.list_box_regions.Append(region.name)

        # Create the map selection dialog
        if self.mapDialog is not None:
            self.mapDialog.Destroy()

        self.mapDialog = MapDialog(frame_1, frame_1)
        frame_1.button_map.Enable(True)

        # Force document relexing
        self.text_ctrl_spec.Colourise(0,self.text_ctrl_spec.GetTextLength())


    def onMenuNew(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Create a new specification.
        """

        if self.dirty:
            if not self.askIfUserWantsToSave("starting a new specification"):
                return

        self.initializeNewSpec()

        #event.Skip()

    def onMenuOpen(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a specification file to open, and then open it.
        """

        if self.dirty:
            if not self.askIfUserWantsToSave("opening a different specification"):
                return

        filename = wx.FileSelector("Open File", default_extension="spec", default_path=".",
                                  wildcard="Specification files (*.spec)|*.spec",
                                  flags = wx.OPEN | wx.FILE_MUST_EXIST)
        if filename == "": return

        self.openFile(filename)

        #event.Skip()

    def onMenuSave(self, event=None): # wxGlade: SpecEditorFrame.<event_handler>
        """
        If the file has been saved already, save it quietly.
        Else, ask for a filename and then save it.
        """

        if self.proj.project_basename is None:
            self.onMenuSaveAs(event)
        else:
            self.proj.writeSpecFile()
            self.dirty = False

    def onMenuSaveAs(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Ask the user for a filename to save the specification as, and then save it.
        """

        # If this isn't the first save, set the current filename as default
        if self.proj.project_basename is None:
            default = ""
        else:
            default = self.proj.getFilenamePrefix() + ".spec"

        # Get a filename
        filename = wx.FileSelector("Save File As",
                                  default_path=os.path.dirname(default),
                                  default_filename=os.path.basename(default),
                                  default_extension="spec",
                                  wildcard="Specification files (*.spec)|*.spec",
                                  flags = wx.SAVE | wx.OVERWRITE_PROMPT)

        if filename == "": return # User cancelled.

        # Force a .spec extension.  How mean!!!
        if os.path.splitext(filename)[1] != ".spec":
            filename = filename + ".spec"

        # Save data to the file
        self.proj.writeSpecFile(filename)
        self.dirty = False

        # Update the window title
        self.SetTitle("Specification Editor - " + self.proj.project_basename + ".spec")

    def openFile(self, filename):
        proj = project.Project()

        # If necessary, try appending ".spec"
        if not os.path.exists(filename):
            filename = os.path.splitext(filename)[0] + ".spec"

        if not proj.loadProject(filename):
            wx.MessageBox("Cannot open specification file %s" % (filename), "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        self.initializeNewSpec()
        self.proj = proj

        # Suppress any errors
        self.proj.setSilent(True)
        self.decomposedRFI = self.proj.loadRegionFile(decomposed=True)
        self.proj.setSilent(False)

        if self.decomposedRFI is None and self.proj.regionMapping is not None:
            print "WARNING: Region mapping exists but failed to load decomposed region file"

        # TODO: Advise the user that these may be inaccurate if
        #  mtime(spec)>mtime(aut) or spectext is dirty!!!

        if self.proj.regionMapping is not None:
            # Update workspace decomposition listbox
            self.list_box_locphrases.Set(self.proj.regionMapping.keys())
            self.list_box_locphrases.Select(0)

        # Load in LTL file to the LTL tab
        if os.path.exists(self.proj.getFilenamePrefix()+".ltl"):
            f = open(self.proj.getFilenamePrefix()+".ltl","r")
            ltl = "".join(f.readlines())
            f.close()
            self.text_ctrl_LTL.SetValue(ltl)

        #####################################

        self.text_ctrl_spec.AppendText(self.proj.specText)
        #self.text_ctrl_spec.ConvertEOLs(wx.stc.STC_EOL_LF)

        if self.proj.rfi is not None:
            self.updateFromRFI()

        self.list_box_actions.Set(self.proj.all_actuators)
        self.list_box_actions.SetCheckedStrings(self.proj.enabled_actuators)
        if len(self.proj.all_actuators) > 0:
            self.button_actuator_remove.Enable(True)

        self.list_box_sensors.Set(self.proj.all_sensors)
        self.list_box_sensors.SetCheckedStrings(self.proj.enabled_sensors)
        if len(self.proj.all_sensors) > 0:
            self.button_sensor_remove.Enable(True)

        self.list_box_customs.Set(self.proj.all_customs)
        if len(self.proj.all_customs) > 0:
            self.button_custom_remove.Enable(True)

        # Update the window title
        self.SetTitle("Specification Editor - " + self.proj.project_basename + ".spec")

        self.text_ctrl_spec.EmptyUndoBuffer()

        # Set compilation option checkboxes
        self.updateMenusFromProjectOptions()

        self.dirty = False

    def updateMenusFromProjectOptions(self):
        self.frame_1_menubar.Check(MENU_CONVEXIFY, self.proj.compile_options["convexify"])
        self.frame_1_menubar.Check(MENU_BITVECTOR, self.proj.compile_options["use_region_bit_encoding"])
        self.frame_1_menubar.Check(MENU_FASTSLOW, self.proj.compile_options["fastslow"])
        self.frame_1_menubar.Check(MENU_SYMBOLIC, self.proj.compile_options["symbolic"])

        if self.proj.compile_options["parser"] == "slurp":
            self.frame_1_menubar.Check(MENU_PARSERMODE_SLURP, True)
        elif self.proj.compile_options["parser"] == "structured":
            self.frame_1_menubar.Check(MENU_PARSERMODE_STRUCTURED, True)
        elif self.proj.compile_options["parser"] == "ltl":
            self.frame_1_menubar.Check(MENU_PARSERMODE_LTL, True)

        if self.proj.compile_options["synthesizer"] == "jtlv":
            self.frame_1_menubar.Check(MENU_SYNTHESIZER_JTLV, True)
        elif self.proj.compile_options["synthesizer"] == "slugs":
            self.frame_1_menubar.Check(MENU_SYNTHESIZER_SLUGS, True)

    def doClose(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Respond to the "Close" menu command.
        """

        if self.dirty:
            if not self.askIfUserWantsToSave("closing"): return

        # Kill any remaining subprocesses
        for n, p in self.subprocess.iteritems():
            if p is not None:
                response = wx.MessageBox("Quitting SpecEditor will also close %s.\nContinue anyways?" % n,
                                        "Subprocess still running", wx.YES_NO, self)

                if response == wx.YES:
                    p.kill()
                    p.join()
                elif response == wx.NO:
                    return

        self.Destroy()

    def askIfUserWantsToSave(self, action):
        """ Give the user the opportunity to save the current document.

            'action' is a string describing the action about to be taken.  If
            the user wants to save the document, it is saved immediately.  If
            the user cancels, we return False.

            From pySketch example
        """

        response = wx.MessageBox("Save changes before " + action + "?",
                                "Confirm", wx.YES_NO | wx.CANCEL, self)

        if response == wx.YES:
            self.onMenuSave()
            return True
        elif response == wx.NO:
            return True # User doesn't want changes saved.
        elif response == wx.CANCEL:
            return False # User cancelled.

    def onMenuUndo(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanUndo():
            self.text_ctrl_spec.Undo()

        event.Skip()

    def onMenuRedo(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanRedo():
            self.text_ctrl_spec.Redo()

        event.Skip()

    def onMenuCut(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.text_ctrl_spec.Cut()
        event.Skip()

    def onMenuCopy(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.text_ctrl_spec.Copy()
        event.Skip()

    def onMenuPaste(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if self.text_ctrl_spec.CanPaste():
            self.text_ctrl_spec.Paste()

        #event.Skip()

    def onMenuCompile(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Clear the error markers
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_INIT)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_SAFE)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_LIVE)
        self.text_ctrl_spec.MarkerDeleteAll(MARKER_PARSEERROR)

		# Let's make sure we have everything!
        if self.proj.rfi is None:
            wx.MessageBox("Please define regions before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        if self.proj.specText.strip() == "":
            wx.MessageBox("Please write a specification before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        if self.proj.project_basename is None:
            wx.MessageBox("Please save your project before compiling.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # Check that there's a boundary region
        if self.proj.compile_options["decompose"] and self.proj.rfi.indexOfRegionWithName("boundary") < 0:
            wx.MessageBox("Please define a boundary region before compiling.\n(Just add a region named 'boundary' in RegionEditor.)", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return


        # TODO: we could just pass the proj object
        self.proj.writeSpecFile()
        self.dirty = False
        compiler = SpecCompiler(self.proj.getFilenamePrefix() + ".spec")

        # Clear the log so we can start fresh grocer
        self.text_ctrl_log.Clear()

        # Redirect all output to the log
        redir = RedirectText(self,self.text_ctrl_log)

        sys.stdout = redir
        sys.stderr = redir

        self.appendLog("Decomposing map into convex regions...\n", "BLUE")

        compiler._decompose()
        self.proj = compiler.proj
        self.decomposedRFI = compiler.parser.proj.rfi

        # Update workspace decomposition listbox
        if self.proj.regionMapping is not None:
            self.list_box_locphrases.Set(self.proj.regionMapping.keys())
            self.list_box_locphrases.Select(0)

        self.appendLog("Creating LTL...\n", "BLUE")

        spec, self.tracebackTree, self.response = compiler._writeLTLFile()

        # Add any auto-generated propositions to the list
        # TODO: what about removing old ones?
        for p in compiler.proj.internal_props:
            if p not in self.list_box_customs.GetItems():
                self.list_box_customs.AppendAndEnsureVisible(p)

        # Add any auto-generated sensor propositions to the list
        for s in compiler.proj.enabled_sensors:
            if s not in self.list_box_sensors.GetItems():
                self.list_box_sensors.Insert(s,0)
                self.list_box_sensors.Check(0)


        if self.tracebackTree is None:
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__
            self.appendLog("ERROR: Aborting compilation due to syntax error.\n", "RED")
            return

        self.appendLog("Creating SMV file...\n", "BLUE")

        compiler._writeSMVFile()

        # Load in LTL file to the LTL tab
        if os.path.exists(self.proj.getFilenamePrefix()+".ltl"):
            f = open(self.proj.getFilenamePrefix()+".ltl","r")
            ltl = "".join(f.readlines())
            f.close()
            self.text_ctrl_LTL.SetValue(ltl)


        self.appendLog("Creating automaton...\n", "BLUE")

        # Disable console redirection
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

        # Put up a busy dialog
        busy_dialog = wx.ProgressDialog("Synthesizing...", "Please wait, synthesizing a strategy...",
                                        style=wx.PD_APP_MODAL | wx.PD_CAN_ABORT | wx.PD_ELAPSED_TIME)

        self.badInit = ""
        def onLog(text):
            # Check for bad initial conditions list in unsynth case
            if "For example" in text:
                self.badInit = text.split('\t')[-1].strip()

            # Display output realtime in the log
            wx.CallAfter(self.appendLog, "\t"+text)

        # Kick off the synthesis
        try:
            compiler._synthesizeAsync(onLog)
        except RuntimeError as e:
            wx.MessageBox(e.message, "Error",
                        style = wx.OK | wx.ICON_ERROR)
            busy_dialog.Destroy()
            return

        while not compiler.synthesis_complete.isSet():
            # Keep the progress bar spinning and check if the Abort button has been pressed
            keep_going = busy_dialog.UpdatePulse()[0]

            if not keep_going:
                compiler.abortSynthesis()
                break

            # Let wx and the OS have some time
            # We are updating here instead of in the log callback because log output
            # may be very infrequent
            wx.GetApp().Yield(True)
            time.sleep(0.1)

        # Done! Close the dialog.
        busy_dialog.Destroy()

        # If the user aborted, we should just stop here
        if not keep_going:
            self.appendLog("\tSynthesis aborted by user!\n", "RED")
            return compiler, None

        if self.proj.compile_options['fastslow']:
            if compiler.realizableFS:
                self.appendLog("Automaton successfully synthesized for slow and fast actions.\n", "GREEN")
            elif compiler.realizable:
                self.appendLog("Specification is unsynthesizable for slow and fast actions.\n Automaton successfully synthesized for instantaneous actions.\n", "GREEN")
            else:
                self.appendLog("ERROR: Specification was unsynthesizable (unrealizable/unsatisfiable) for instantaneous actions.\n", "RED")
        else:
            if compiler.realizable:
                self.appendLog("Automaton successfully synthesized for instantaneous actions.\n", "GREEN")
            else:
                self.appendLog("ERROR: Specification was unsynthesizable (unrealizable/unsatisfiable) for instantaneous actions.\n", "RED")

        # Check for trivial aut
        if compiler.realizable or compiler.realizableFS:
            if not compiler._autIsNonTrivial():
                self.appendLog("\tWARNING: Automaton is trivial.  Further analysis is recommended.\n", "RED")

        return compiler, self.badInit

    def appendLog(self, text, color="BLACK"):
        self.text_ctrl_log.BeginTextColour(color)
        #self.text_ctrl_log.BeginBold()
        self.text_ctrl_log.WriteText(text)
        #self.text_ctrl_log.EndBold()
        self.text_ctrl_log.EndTextColour()
        self.text_ctrl_log.ShowPosition(self.text_ctrl_log.GetLastPosition())
        wx.GetApp().Yield(True) # Ensure update

    def onMenuSimulate(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """ Run the simulation with current experiment configuration. """

        if not self.proj.compile_options["use_region_bit_encoding"]:
            wx.MessageBox("Execution requires bit-vector region encoding.\nPlease enable it and recompile.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # TODO: or check mtime
        if self.dirty:
            response = wx.MessageBox("Specification may have changed since last compile.\nContinue anyways, without recompiling?",
                                    "Warning", wx.YES_NO | wx.CANCEL, self)

            if response != wx.YES:
                return

        if not os.path.isfile(self.proj.getFilenamePrefix()+".aut"):
            # TODO: Deal with case where aut file exists but is lame
            wx.MessageBox("Cannot find automaton for simulation.  Please make sure compilation completed successfully.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        redir = RedirectText(self,self.text_ctrl_log)

        sys.stdout = redir
        sys.stderr = redir

        subprocess.Popen([sys.executable, "-u", "-m", "lib.execute", "-a", self.proj.getStrategyFilename(), "-s", self.proj.getFilenamePrefix() + ".spec"])

        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__


    def onClickEditRegions(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Provide visual feedback and make sure multiple copies of RegionEditor
        # can't be open at the same time
        self.button_edit_regions.Enable(False)
        self.button_edit_regions.SetLabel("Starting Region Editor...")

        # Force window update to fit new text length in button
        # FIXME: Doesn't change size on Windows??
        self.Layout()

        def regedCallback():
            # Re-enable the Edit Regions button
            self.button_edit_regions.SetLabel("Edit Regions...")
            self.Layout()
            self.button_edit_regions.Enable(True)

            # If we were editing a new region file
            if self.proj.rfi is None:
                fileName = self.proj.getFilenamePrefix()+".regions"
                # Check whether the user actually saved or not
                if os.path.isfile(fileName):
                    rfi = RegionFileInterface()

                    # Try loading the file
                    if not rfi.readFile(fileName):
                        print "ERROR: Could not load newly created region file"
                        return

                    self.proj.rfi = rfi
                    self.dirty = True
                    self.updateFromRFI()

            # Or if it was a pre-existing region file
            else:
                fileName = self.proj.rfi.filename
                # Check to see if its modification time has changed; no point in reloading otherwise
                if os.path.getmtime(fileName) != self.lastRegionModTime:
                    rfi = RegionFileInterface()

                    # Try loading the file
                    if not rfi.readFile(fileName):
                        print "ERROR: Could not reload region file"
                        return

                    self.proj.rfi = rfi
                    self.dirty = True
                    self.updateFromRFI()

            self.subprocess["Region Editor"] = None

        # Spawn asynchronous subprocess
        if self.proj.rfi is not None:
            # If we already have a region file defined, open it up for editing
            fileName = self.proj.rfi.filename
            self.lastRegionModTime = os.path.getmtime(fileName)
            self.subprocess["Region Editor"] = WxAsynchronousProcessThread([sys.executable, "-u", "regionEditor.py", fileName], regedCallback, None)
        else:
            # Otherwise let's create a new region file
            if self.proj.project_basename is None:
                # First we need a project name, though
                wx.MessageBox("Please save first to give the project a name.", "Error",
                            style = wx.OK | wx.ICON_ERROR)
                self.onMenuSave()
                if self.proj.project_basename is None:
                    # If the save was cancelled, forget it
                    self.button_edit_regions.SetLabel("Edit Regions...")
                    self.Layout()
                    self.button_edit_regions.Enable(True)
                    return

            # We'll name the region file with the same name as our project
            fileName = self.proj.getFilenamePrefix()+".regions"
            self.subprocess["Region Editor"] = WxAsynchronousProcessThread([sys.executable, "-u", "regionEditor.py", fileName], regedCallback, None)

    def onMenuConfigSim(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Launch the config editor
        # TODO: Discourage editing of spec while it's open?

        if self.subprocess["Simulation Configuration"] is not None:
            wx.MessageBox("Simulation Config is already running.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        if self.proj.project_basename is None:
            # First we need a project name, though
            wx.MessageBox("Please save first to give the project a name.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            self.onMenuSave()
            if self.proj.project_basename is None:
                # If the save was cancelled, forget it
                return

        def simConfigCallback():
            # Reload just the current config object
            # (no more is necessary because this is the only part of the spec file
            # that configEditor could modify)
            other_proj = project.Project()
            other_proj.loadProject(self.proj.getFilenamePrefix()+".spec")
            self.proj.current_config= other_proj.current_config
            self.subprocess["Simulation Configuration"] = None

        self.subprocess["Simulation Configuration"] = WxAsynchronousProcessThread([sys.executable, "-u", "-m", "lib.configEditor", self.proj.getFilenamePrefix()+".spec"], simConfigCallback, None)

    def _exportDotFile(self):
        region_domain = strategy.Domain("region",  self.decomposedRFI.regions, strategy.Domain.B0_IS_MSB)
        strat = strategy.createStrategyFromFile(self.proj.getStrategyFilename(),
                                                self.proj.enabled_sensors,
                                                self.proj.enabled_actuators + self.proj.all_customs +  [region_domain])

        strat.exportAsDotFile(self.proj.getFilenamePrefix()+".dot", self.proj.regionMapping)

    def _exportSMVFile(self):
        aut.writeSMV(self.proj.getFilenamePrefix()+"MC.smv")





    def onMenuViewAut(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        if not os.path.isfile(self.proj.getFilenamePrefix()+".aut"):
            wx.MessageBox("Cannot find automaton for viewing.  Please make sure compilation completed successfully.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        if self.subprocess["Dotty"] is not None:
            wx.MessageBox("Dotty is already running.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # TODO: Advise the user that the exported automaton may be inaccurate if
        #  mtime(spec)>mtime(aut) or spectext is dirty

        self.appendLog("Generating PDF file from automaton...\n", "BLUE")

        self._exportDotFile()

        def dottyCallback():
            # TODO: Check mtime to make sure it didn't die
            if os.path.isfile(self.proj.getFilenamePrefix()+".pdf"):
                self.appendLog("Export complete!\n", "GREEN")
            else:
                self.appendLog("Export failed.\n", "RED")

            self.subprocess["Dotty"] = None

        self.subprocess["Dotty"] = WxAsynchronousProcessThread(["dot","-Tpdf","-o%s.pdf" % self.proj.getFilenamePrefix(),"%s.dot" % self.proj.getFilenamePrefix()], dottyCallback, None)

        self.subprocess["Dotty"].startComplete.wait()
        if not self.subprocess["Dotty"].running:
            wx.MessageBox("Dotty could not be executed.\nAre you sure Graphviz is correctly installed?", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

    def onMenuQuit(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.doClose(event)
        #event.Skip()

    def onMenuAbout(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        wx.MessageBox("Specification Editor is part of the LTLMoP Toolkit.\n" + \
                      "For more information, please visit http://ltlmop.github.com", "About Specification Editor",
                      style = wx.OK | wx.ICON_INFORMATION)
        #event.Skip()

    def onLocPhraseSelect(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        self.panel_locmap.Refresh()
        event.Skip()

    def onMenuAnalyze(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        #TODO: check to see if we need to recompile
        self.compiler, self.badInit = self.onMenuCompile(event)

        # instantiate if necessary
        if self.analysisDialog is None:
            self.analysisDialog = AnalysisResultsDialog(self, self)

        # Clear dialog to make way for new info
        self.analysisDialog.text_ctrl_summary.Clear()

        # Populate tree based on traceback data

        if self.proj.compile_options["parser"] == "slurp":
            self.analysisDialog.label_traceback.Show()
            self.analysisDialog.tree_ctrl_traceback.Show()
            if self.tracebackTree is not None:
                self.analysisDialog.populateTree(self.tracebackTree)

            self.analysisDialog.tree_ctrl_traceback.ExpandAll()
        else:
            self.analysisDialog.label_traceback.Hide()
            self.analysisDialog.tree_ctrl_traceback.Hide()

        self.appendLog("Running analysis...\n","BLUE")

        # Redirect all output to the log
        redir = RedirectText(self, self.text_ctrl_log)
        sys.stdout = redir
        sys.stderr = redir

        try:
            (realizable, self.unsat, nonTrivial, self.to_highlight, output) = self.compiler._analyze()
        except RuntimeError as e:
            wx.MessageBox(e.message, "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return
        finally:
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__

        # Remove lines about garbage collection from the output and remove extraenous lines
        output_lines = [line for line in output.split('\n') if line.strip() and
                        "Garbage collection" not in line and
                        "Resizing node table" not in line]

        if realizable:
            # Strip trailing \n from output so it doesn't scroll past it
            self.analysisDialog.appendLog('\n'.join(output_lines), "BLACK")
            if nonTrivial:
                self.analysisDialog.appendLog("\nSynthesized automaton is non-trivial.", "BLACK")
            else:
                self.analysisDialog.appendLog("\nSynthesized automaton is trivial.", "RED")
        else:
            self.analysisDialog.appendLog(output.rstrip(), "RED")
        self.analysisDialog.appendLog('\n')

        #highlight guilty sentences
        #special treatment for goals: we already know which one to highlight
        if self.proj.compile_options["parser"] == "structured":
            for h_item in self.to_highlight:
                tb_key = h_item[0].title() + h_item[1].title()
                if h_item[1] == "goals":
                    self.text_ctrl_spec.MarkerAdd(self.tracebackTree[tb_key][h_item[2]]-1, MARKER_LIVE)
        elif self.proj.compile_options["parser"] == "slurp":
            for frag in self.to_highlight:
                self.analysisDialog.markFragments(*frag)
        else:
            print "Fragment marking not yet supported for this parser type"

        self.analysisDialog.Show()

        self.appendLog("Initial analysis complete.\n\n", "BLUE")

        if (not realizable or not nonTrivial):
            self.appendLog("Further analysis is possible.\n", "BLUE")
            self.analysisDialog.button_refine.Enable(True)
            self.analysisDialog.button_refine.SetLabel("Refine analysis...")
            self.analysisDialog.Layout()
        else:
            self.appendLog("No further analysis needed.\n", "BLUE")
            self.analysisDialog.button_refine.Enable(False)
            self.analysisDialog.button_refine.SetLabel("No further analysis available.")
            self.analysisDialog.Layout()

    def refineAnalysis(self):
        self.appendLog("Please wait; refining analysis...\n", "BLUE")

        guilty = self.compiler._coreFinding(self.to_highlight, self.unsat, self.badInit)

        self.highlightCores(guilty, self.compiler)
        
        if not self.unsat:
            to_highlight = self.compiler._iterateCores()
        
            #highlight guilty transitions
            if self.proj.compile_options["parser"] == "structured":
                for h_item in to_highlight:
                    tb_key = h_item[0].title() + h_item[1].title()
                    if h_item[2] < len(self.tracebackTree[tb_key]):
                        self.text_ctrl_spec.MarkerAdd(self.tracebackTree[tb_key][h_item[2]]-1, MARKER_INIT)

        self.appendLog("Final analysis complete.\n", "BLUE")

        self.analysisDialog.button_refine.Enable(False)
        self.analysisDialog.button_refine.SetLabel("No further analysis available.")
        self.analysisDialog.Layout()

    def highlightCores(self, guilty, compiler):
        if self.proj.compile_options["parser"] == "structured":
            if guilty is not None:
                #highlight system initial condition
                for l in self.tracebackTree['SysInit']:
                    self.highlight(l, "init")
                #look up the line number corresponding to each guilty LTL formula
                for k,v in compiler.LTL2SpecLineNumber.iteritems():
                    newCs = k.replace("\t","\n").split('\n')
                    if not set(guilty).isdisjoint(newCs):
                        #for now, just highlight with the colour originally used for initial conditions
                        self.highlight(v, 'init')
        else:
            guilty_clean = []
            for ltl_frag in guilty:
                canonical_ltl_frag = ltl_frag.lstrip().rstrip("\n\t &")
                try:
                    guilty_clean.append(compiler.reversemapping[canonical_ltl_frag])
                except KeyError:
                    if "[]" not in canonical_ltl_frag:
                        # Special case: we were given an initial condition that wasn't explicitly created by the user
                        # This means it came from the GROne code, so let's just highlight all initial conditions
                        # TODO: we ought to be able to narrow this down more
                        guilty_clean.extend([v for k,v in compiler.reversemapping.iteritems() if "[]" not in v])
                    else:
                        print "WARNING: LTL fragment {!r} not found in spec->LTL mapping".format(canonical_ltl_frag)

            # Add SLURP to path for import
            p = os.path.dirname(os.path.abspath(__file__))
            sys.path.append(os.path.join(p, "..", "etc", "SLURP"))
            from ltlbroom.specgeneration import explain_conflict
            msg, highlight_tree = explain_conflict(guilty_clean, self.tracebackTree)
            # Reprocess the traceback tree
            self.analysisDialog.populateTree(highlight_tree)
            self.analysisDialog.appendLog(msg)

    def highlight(self, l, type):
        if type == "init":
           self.text_ctrl_spec.MarkerAdd(l-1, MARKER_INIT)
        elif type == "trans":
           self.text_ctrl_spec.MarkerAdd(l-1, MARKER_SAFE)


    def onMenuMopsy(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        # Opens the counterstrategy visualization interfacs ("Mopsy")

        if not self.proj.compile_options["use_region_bit_encoding"]:
            wx.MessageBox("Mopsy currently requires bit-vector region encoding.\nPlease enable it and recompile.", "Error",
                        style = wx.OK | wx.ICON_ERROR)
            return

        # TODO: check for failed compilation before allowing this
        if self.to_highlight is None:
            wx.MessageBox("Please run analysis before trying to run Mopsy.", "Analysis required",
                        style = wx.OK | wx.ICON_ERROR)
            return

        desired_jx = None
        for frag in self.to_highlight:
            if frag[0] == 'sys' and frag[1] == 'goals':
                desired_jx = frag[2]
                break

        if desired_jx is None:
            wx.MessageBox("No goal was found to be involved in unrealizability.\nThere is no need to run Mopsy.", "Mopsy irrelevant",
                        style = wx.OK | wx.ICON_ERROR)
            return

        print "Calling mopsy with desired_jx={}...".format(desired_jx)

        subprocess.Popen([sys.executable, os.path.join(self.proj.ltlmop_root, "etc", "utils", "mopsy.py"), self.proj.getFilenamePrefix()+".spec", str(desired_jx)])

    def onPropAdd(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Display a dialog asking for a proposition name and then
        add it to the appropriate proposition list.
        """

        if event.GetEventObject() is self.button_sensor_add:
            prop_prefix = "sensor"
            lb = self.list_box_sensors
            remove_button = self.button_sensor_remove
            append_lists = [self.proj.all_sensors, self.proj.enabled_sensors]
        elif event.GetEventObject() is self.button_actuator_add:
            prop_prefix = "actuator"
            lb = self.list_box_actions
            remove_button = self.button_actuator_remove
            append_lists = [self.proj.all_actuators, self.proj.enabled_actuators]
        elif event.GetEventObject() is self.button_custom_add:
            prop_prefix = "custom"
            lb = self.list_box_customs
            remove_button = self.button_custom_remove
            append_lists = [self.proj.all_customs]

        # Find any existing propositions of name sensor/actuator/customX
        nums = []
        p = re.compile(r"^%s(?P<num>\d+)$" % prop_prefix)
        for name in lb.GetItems():
            m = p.match(name)
            if not m: continue
            nums.append(int(m.group('num')))
        nums.sort()

        # Find smallest available number
        last = 0
        for num in nums:
            if num == last + 1:
                last = num
            elif num == last:
                #print "Warning: Multiple propositions with name 'r%d'." % num
                continue
            else:
                break

        default_name = prop_prefix + str(last + 1)

        # Ask the user for a proposition name, suggesting the next available one as default
        name = wx.GetTextFromUser("Name:", "New %s Proposition" % prop_prefix.title(), default_name)

        if name != "":
            if name[0].isdigit():
                wx.MessageBox("Propositions must begin with a letter.", "Invalid proposition name",
                            style = wx.OK | wx.ICON_ERROR)
                return

            # If it's valid, add it, select it and enable it
            lb.Insert(name, lb.GetCount())
            lb.Select(lb.GetCount()-1)
            if prop_prefix != "custom":
                lb.Check(lb.GetCount()-1)
            remove_button.Enable(True)

            for l in append_lists:
                l.append(name)

        self.dirty = True

        event.Skip(False)

        # Force document relexing
        self.text_ctrl_spec.Colourise(0,self.text_ctrl_spec.GetTextLength())

    def onPropRemove(self, event): # wxGlade: SpecEditorFrame.<event_handler>
        """
        Remove the selected proposition from the list.
        """

        # Figure out which listbox called us
        if event.GetEventObject() is self.button_sensor_remove:
            lb = self.list_box_sensors
            remove_lists = [self.proj.all_sensors, self.proj.enabled_sensors]
        elif event.GetEventObject() is self.button_actuator_remove:
            lb = self.list_box_actions
            remove_lists = [self.proj.all_actuators, self.proj.enabled_actuators]
        elif event.GetEventObject() is self.button_custom_remove:
            lb = self.list_box_customs
            remove_lists = [self.proj.all_customs]

        # Remove the prop from the appropriate list(s) in proj
        for l in remove_lists:
            if lb.GetStringSelection() in l:
                l.remove(lb.GetStringSelection())

        selection = lb.GetSelection()
        lb.Delete(selection)

        # Select something reasonable now that the old item is gone
        if selection > 0:
            lb.Select(selection-1)
        else:
            if lb.GetCount() != 0:
                lb.Select(0)

        # Disable the Delete button if there's nothing to delete
        if lb.GetCount() == 0:
            event.GetEventObject().Enable(False)

        self.dirty = True

        event.Skip(False)

        # Force document relexing
        self.text_ctrl_spec.Colourise(0,self.text_ctrl_spec.GetTextLength())

    def onMenuSetCompileOptions(self, event):  # wxGlade: SpecEditorFrame.<event_handler>
        self.proj.compile_options["convexify"] = self.frame_1_menubar.IsChecked(MENU_CONVEXIFY)
        self.proj.compile_options["fastslow"] = self.frame_1_menubar.IsChecked(MENU_FASTSLOW)
        self.proj.compile_options["use_region_bit_encoding"] = self.frame_1_menubar.IsChecked(MENU_BITVECTOR)
        self.proj.compile_options["symbolic"] = self.frame_1_menubar.IsChecked(MENU_SYMBOLIC)

        if self.frame_1_menubar.IsChecked(MENU_PARSERMODE_SLURP):
            self.proj.compile_options["parser"] = "slurp"
        elif self.frame_1_menubar.IsChecked(MENU_PARSERMODE_STRUCTURED):
            self.proj.compile_options["parser"] = "structured"
        elif self.frame_1_menubar.IsChecked(MENU_PARSERMODE_LTL):
            self.proj.compile_options["parser"] = "ltl"

        if self.frame_1_menubar.IsChecked(MENU_SYNTHESIZER_JTLV):
            self.proj.compile_options["synthesizer"] = "jtlv"
        elif self.frame_1_menubar.IsChecked(MENU_SYNTHESIZER_SLUGS):
            self.proj.compile_options["synthesizer"] = "slugs"

        self.dirty = True

    def onRegionLabelStyleChange(self, event):  # wxGlade: SpecEditorFrame.<event_handler>
        if self.checkbox_regionlabel.GetValue():
            self.checkbox_regionlabelbits.Enable(True)
        else:
            self.checkbox_regionlabelbits.Enable(False)

        self.panel_locmap.Refresh()
        event.Skip()

# end of class SpecEditorFrame

class RedirectText:
    """
    A class that lets the output of a stream be directed into a text box.

    http://mail.python.org/pipermail/python-list/2007-June/445795.html
    """

    def __init__(self,parent,aWxTextCtrl):
        self.out=aWxTextCtrl
        self.parent=parent

    def write(self,string):
        self.out.BeginTextColour("BLACK")
        self.out.WriteText("\t"+string)
        self.out.EndTextColour()
        self.out.ShowPosition(self.out.GetLastPosition())

        m = re.search(r"Could not parse the sentence in line (\d+)", string)
        if m:
            self.parent.text_ctrl_spec.MarkerAdd(int(m.group(1))-1,MARKER_PARSEERROR)


if __name__ == "__main__":
    SpecEditor = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    frame_1 = SpecEditorFrame(None, -1, "")
    SpecEditor.SetTopWindow(frame_1)
    frame_1.Show()

    if len(sys.argv) > 1:
        frame_1.openFile(sys.argv[1])

    SpecEditor.MainLoop()
