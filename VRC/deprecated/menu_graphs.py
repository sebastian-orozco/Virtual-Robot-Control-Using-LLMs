# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pathlib import Path

import omni.graph.core as og
import omni.ui as ui
import omni.usd
from omni.isaac.core.utils.prims import get_all_matching_child_prims, get_prim_at_path
from omni.isaac.core.utils.stage import get_next_free_path
from omni.isaac.ui.callbacks import on_docs_link_clicked, on_open_IDE_clicked
from omni.isaac.ui.style import get_style
from omni.isaac.ui.widgets import ParamWidget, SelectPrimWidget
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.window.extensions import SimpleCheckBox
from pxr import OmniGraphSchema, UsdPhysics

OG_DOCS_LINK = (
    "https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_omnigraph_shortcuts.html"
)


class DifferentialRobotGraph:
    def __init__(self):
        # have a place to save variables so there's default when creating new graphs in the same session
        self._og_path = "/Graphs/differential_controller"
        self._art_root_path = ""
        self._robot_prim_path = ""
        self._add_to_existing_graph = False
        self._wheel_radius = 0.0
        self._wheel_distance = 0.0
        self._left_joint_name = ""
        self._right_joint_name = ""
        self._left_joint_index = 0
        self._right_joint_index = 0
        self._use_keyboard = False

    def make_graph(self):
        # stop the simulation before adding nodes
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.stop()

        keys = og.Controller.Keys

        # if starting from a new graph, start it with just a tick, the rest is the same for adding to exsiting graph
        if not self._add_to_existing_graph:
            self._og_path = get_next_free_path(self._og_path, "")
            graph_handle = og.Controller.create_graph({"graph_path": self._og_path, "evaluator_name": "execution"})
            og.Controller.create_node(self._og_path + "/OnPlaybackTick", "omni.graph.action.OnPlaybackTick")
        else:
            graph_handle = og.get_graph_by_path(self._og_path)

        # to an existin graph
        # traverse through the graph
        all_nodes = graph_handle.get_nodes()
        tick_node = None
        diff_node = None
        art_node = None
        diff_node_name = "DifferentialController"
        art_node_name = "ArticulationController"
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if node_type == "omni.graph.action.OnPlaybackTick" or node_type == "omni.graph.action.OnTick":
                tick_node = node_path
            elif node_type == "omni.isaac.wheeled_robots.DifferentialController":
                # if there already exist a differential controller, add a new one
                diff_node = get_next_free_path(node_path, "")
                diff_node_name = Path(diff_node).name
            elif node_type == "omni.isaac.core_nodes.IsaacArticulationController":
                # if there already exist an articulation controller, add a new one
                art_node = get_next_free_path(node_path, "")
                art_node_name = Path(art_node).name

        # rest of the nodes
        og.Controller.edit(
            graph_handle,
            {
                keys.CREATE_NODES: [
                    (diff_node_name, "omni.isaac.wheeled_robots.DifferentialController"),
                    (art_node_name, "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    (diff_node_name + ".inputs:wheelRadius", self._wheel_radius),
                    (diff_node_name + ".inputs:wheelDistance", self._wheel_distance),
                    (art_node_name + ".inputs:targetPrim", self._art_root_path),
                ],
                keys.CONNECT: [
                    (tick_node + ".outputs:tick", diff_node_name + ".inputs:execIn"),
                    (tick_node + ".outputs:tick", art_node_name + ".inputs:execIn"),
                    (diff_node_name + ".outputs:velocityCommand", art_node_name + ".inputs:velocityCommand"),
                ],
            },
        )

        # if user input used joint indices
        if self._left_joint_index and self._right_joint_index:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [("ArrayNames", "omni.graph.nodes.ConstructArray")],
                    keys.CREATE_ATTRIBUTES: [
                        ("ArrayNames.inputs:input1", "int"),
                    ],
                    keys.SET_VALUES: [
                        ("ArrayNames.inputs:arrayType", "int[]"),
                        ("ArrayNames.inputs:input0", self._right_joint_index),
                        ("ArrayNames.inputs:input1", self._left_joint_index),
                        ("ArrayNames.inputs:arraySize", 2),
                    ],
                },
            )
            og.Controller.connect(
                og.Controller.attribute(self._og_path + "/ArrayNames.outputs:array"),
                og.Controller.attribute(self._og_path + "/" + art_node_name + ".inputs:jointIndices"),
            )

        # if user input used joint names
        elif self._left_joint_name and self._right_joint_name:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [("ArrayNames", "omni.graph.nodes.ConstructArray")],
                    keys.CREATE_ATTRIBUTES: [
                        ("ArrayNames.inputs:input1", "token"),
                    ],
                    keys.SET_VALUES: [
                        ("ArrayNames.inputs:arrayType", "token[]"),
                        ("ArrayNames.inputs:input0", self._right_joint_name),
                        ("ArrayNames.inputs:input1", self._left_joint_name),
                        ("ArrayNames.inputs:arraySize", 2),
                    ],
                },
            )
            og.Controller.connect(
                og.Controller.attribute(self._og_path + "/ArrayNames.outputs:array"),
                og.Controller.attribute(self._og_path + "/" + art_node_name + ".inputs:jointNames"),
            )

        else:
            msg = "using all joints in the articulation"
            post_notification("Differential Robot Graph", msg, NotificationStatus.INFO)

        # if user wants to use keyboard control
        if self._use_keyboard:
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        ("W", "omni.graph.nodes.ReadKeyboardState"),
                        ("A", "omni.graph.nodes.ReadKeyboardState"),
                        ("S", "omni.graph.nodes.ReadKeyboardState"),
                        ("D", "omni.graph.nodes.ReadKeyboardState"),
                        ("ToDoubleW", "omni.graph.nodes.ToDouble"),
                        ("ToDoubleA", "omni.graph.nodes.ToDouble"),
                        ("ToDoubleS", "omni.graph.nodes.ToDouble"),
                        ("ToDoubleD", "omni.graph.nodes.ToDouble"),
                        ("NegateLinear", "omni.graph.nodes.Multiply"),
                        ("NegateAngular", "omni.graph.nodes.Multiply"),
                        ("AddLinear", "omni.graph.nodes.Add"),
                        ("AddAngular", "omni.graph.nodes.Add"),
                        ("SpeedLinear", "omni.graph.nodes.Multiply"),
                        ("ScaleLinear", "omni.graph.nodes.ConstantDouble"),
                        ("SpeedAngular", "omni.graph.nodes.Multiply"),
                        ("ScaleAngular", "omni.graph.nodes.ConstantDouble"),
                        ("NegOne", "omni.graph.nodes.ConstantInt"),
                    ],
                    keys.SET_VALUES: [
                        ("W.inputs:key", "W"),
                        ("A.inputs:key", "A"),
                        ("S.inputs:key", "S"),
                        ("D.inputs:key", "D"),
                        ("NegOne.inputs:value", -1),
                        ("ScaleLinear.inputs:value", 5),
                        ("ScaleAngular.inputs:value", 6),
                    ],
                    keys.CONNECT: [
                        ("W.outputs:isPressed", "ToDoubleW.inputs:value"),
                        ("A.outputs:isPressed", "ToDoubleA.inputs:value"),
                        ("S.outputs:isPressed", "ToDoubleS.inputs:value"),
                        ("D.outputs:isPressed", "ToDoubleD.inputs:value"),
                        ("ToDoubleS.outputs:converted", "NegateLinear.inputs:a"),
                        ("NegOne.inputs:value", "NegateLinear.inputs:b"),
                        ("ToDoubleD.outputs:converted", "NegateAngular.inputs:a"),
                        ("NegOne.inputs:value", "NegateAngular.inputs:b"),
                        ("ToDoubleW.outputs:converted", "AddLinear.inputs:a"),
                        ("NegateLinear.outputs:product", "AddLinear.inputs:b"),
                        ("AddLinear.outputs:sum", "SpeedLinear.inputs:a"),
                        ("ScaleLinear.inputs:value", "SpeedLinear.inputs:b"),
                        ("ToDoubleA.outputs:converted", "AddAngular.inputs:a"),
                        ("NegateAngular.outputs:product", "AddAngular.inputs:b"),
                        ("AddAngular.outputs:sum", "SpeedAngular.inputs:a"),
                        ("ScaleAngular.inputs:value", "SpeedAngular.inputs:b"),
                    ],
                },
            )
            og.Controller.connect(
                og.Controller.attribute(self._og_path + "/SpeedLinear.outputs:product"),
                og.Controller.attribute(self._og_path + "/" + diff_node_name + ".inputs:linearVelocity"),
            )
            og.Controller.connect(
                og.Controller.attribute(self._og_path + "/SpeedAngular.outputs:product"),
                og.Controller.attribute(self._og_path + "/" + diff_node_name + ".inputs:angularVelocity"),
            )

    def create_differential_robot_graph(self):
        og_path_def = ParamWidget.FieldDef(
            name="og_path", label="graph path", type=ui.StringField, default=self._og_path
        )
        wheel_radius_def = ParamWidget.FieldDef(
            name="wheel_radius",
            label="wheel radius",
            type=ui.FloatField,
            default=self._wheel_radius,
            tooltip="in meters",
        )
        wheel_distance_def = ParamWidget.FieldDef(
            name="wheel_distance",
            label="distance between wheels",
            type=ui.FloatField,
            default=self._wheel_distance,
            tooltip="in meters",
        )
        left_joint_name_def = ParamWidget.FieldDef(
            name="left_joint_name", label="Left Joint Name", type=ui.StringField, default=self._left_joint_name
        )
        right_joint_name_def = ParamWidget.FieldDef(
            name="right_joint_name", label="Right Joint Name", type=ui.StringField, default=self._right_joint_name
        )
        left_joint_index_def = ParamWidget.FieldDef(
            name="left_joint_index", label="Left Joint Index", type=ui.IntField, default=self._left_joint_index
        )
        right_joint_index_def = ParamWidget.FieldDef(
            name="right_joint_index", label="Right Joint Index", type=ui.IntField, default=self._right_joint_index
        )

        ## populate the popup window
        self._window = ui.Window("Differential Controller Inputs", width=400, height=500)
        with self._window.frame:
            with ui.VStack(spacing=4):
                with ui.HStack(height=40):
                    ui.Label("Add to an existing graph?", width=ui.Percent(30))
                    cb = ui.SimpleBoolModel(default_value=self._add_to_existing_graph)
                    SimpleCheckBox(self._add_to_existing_graph, self._on_use_existing_graph, model=cb)
                self.og_path_input = ParamWidget(field_def=og_path_def)
                self.robot_prim_input = SelectPrimWidget(label="Robot Prim", default=self._art_root_path)
                self.wheel_radius_input = ParamWidget(field_def=wheel_radius_def)
                self.wheel_distance_input = ParamWidget(field_def=wheel_distance_def)
                ui.Spacer(height=2)

                ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), height=2)
                ui.Label(
                    "If robot has more than two controllable joints:",
                    height=30,
                    style_type_name_override="Label.Label",
                    style={"font_size": 18, "color": 0xFFA8A8A8},
                )
                with ui.VStack(spacing=4):
                    self.right_joint_name_input = ParamWidget(field_def=right_joint_name_def)
                    self.left_joint_name_input = ParamWidget(field_def=left_joint_name_def)
                ui.Label("    OR", height=0)
                with ui.VStack(spacing=4):
                    self.right_joint_index_input = ParamWidget(field_def=right_joint_index_def)
                    self.left_joint_index_input = ParamWidget(field_def=left_joint_index_def)
                ui.Spacer(height=5)
                with ui.HStack():
                    ui.Label("Use Keyboard Control (WASD)", width=ui.Percent(30))
                    cb = ui.SimpleBoolModel(default_value=self._use_keyboard)
                    SimpleCheckBox(self._use_keyboard, self._on_use_keyboard, model=cb)
                with ui.HStack():
                    ui.Spacer(width=ui.Percent(10))
                    ui.Button("OK", height=40, width=ui.Percent(30), clicked_fn=self._on_ok)
                    ui.Spacer(width=ui.Percent(20))
                    ui.Button("Cancel", height=40, width=ui.Percent(30), clicked_fn=self._on_cancel)
                    ui.Spacer(width=ui.Percent(10))
                with ui.Frame(height=30):
                    with ui.VStack():
                        with ui.HStack():
                            ui.Label("Python Script for Graph Generation", width=ui.Percent(30))
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_open_IDE_clicked("", __file__),
                                style=get_style()["IconButton.Image::OpenConfig"],
                            )
                        with ui.HStack():
                            ui.Label("Documentations", width=0, word_wrap=True)
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_docs_link_clicked(OG_DOCS_LINK),
                                style=get_style()["IconButton.Image::OpenLink"],
                            )

        return self._window

    def _on_ok(self):
        self._og_path = self.og_path_input.get_value()
        self._robot_prim_path = self.robot_prim_input.get_value()
        self._wheel_radius = self.wheel_radius_input.get_value()
        self._wheel_distance = self.wheel_distance_input.get_value()
        self._right_joint_name = self.right_joint_name_input.get_value()
        self._left_joint_name = self.left_joint_name_input.get_value()
        self._right_joint_index = self.right_joint_index_input.get_value()
        self._left_joint_index = self.left_joint_index_input.get_value()

        param_check = self._check_params()
        if param_check:
            self.make_graph()
            self._window.visible = False
        else:
            post_notification("Parameter check failed", status=NotificationStatus.WARNING)

    def _on_cancel(self):
        self._window.visible = False

    def _check_params(self):
        stage = omni.usd.get_context().get_stage()

        if self._add_to_existing_graph:
            # make sure the "existing" graph exist
            og_prim = stage.GetPrimAtPath(self._og_path)
            if og_prim.IsValid() and og_prim.IsA(OmniGraphSchema.OmniGraph):
                pass
            else:
                msg = self._og_path + " is not an existing graph, check the og path"
                post_notification(msg, status=NotificationStatus.WARNING)
                return False

        # from the robot parent prim, find the prim that contains the articulation root API
        art_root_prim = get_all_matching_child_prims(
            self._robot_prim_path, predicate=lambda path: get_prim_at_path(path).HasAPI(UsdPhysics.ArticulationRootAPI)
        )
        if len(art_root_prim) == 0:
            msg = "No articulation root prim found under robot parent prim, check if you need to give a different prim for robot"
            post_notification(msg, status=NotificationStatus.WARNING)
            return False
        if len(art_root_prim) > 1:
            msg = "More than one articulation root prim found under robot parent prim, check if you need to give a different prim for robot"
            post_notification(msg, status=NotificationStatus.WARNING)
            return False
        self._art_root_path = art_root_prim[0].GetPath().pathString

        return True

    def _on_use_existing_graph(self, check_state):
        self._add_to_existing_graph = check_state

    def _on_use_keyboard(self, check_state):
        self._use_keyboard = check_state
