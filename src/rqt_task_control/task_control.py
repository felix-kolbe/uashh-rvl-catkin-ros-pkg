import os

import rospy
import rospkg

from actionlib import SimpleActionClient

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QWidget, QPushButton, QBoxLayout, QSizePolicy

from std_msgs.msg import String
from task_msgs.msg import TaskActivationAction, TaskActivationGoal
from actionlib_msgs.msg import GoalStatus



class TaskInfo:
    """self.status being an integer from actionlib_msgs/GoalStatus"""
    def __init__(self, name, button, status=None):
        self.name = name
        self.button = button
        self.status = status


class TaskControl(Plugin):


    _receive_task_list_signal = Signal(list)
    _refresh_button_highlighting_signal = Signal()

    state_colors = {GoalStatus.PENDING: 'greenyellow',
                    GoalStatus.ACTIVE: 'yellow',
                    GoalStatus.PREEMPTED: 'orangered',
                    GoalStatus.SUCCEEDED: 'lightgreen',
                    GoalStatus.ABORTED: 'red',
                    GoalStatus.REJECTED: 'darkred',
                    GoalStatus.PREEMPTING: 'darkorange',
                    GoalStatus.RECALLING: 'violet',
                    GoalStatus.RECALLED: 'darkviolet',
                    GoalStatus.LOST: 'MediumSlateBlue'}

    def __init__(self, context):
        super(TaskControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TaskControl')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
#        if not args.quiet:
#            print 'arguments: ', args
#            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_task_control'), 'resource', 'task_control.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('TaskControlWidget')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)


        # Data

        self._task_map = {} # mapping id strings to TaskInfo objects
        self._active_task_name = None

        # Additional widgets

        self._layout = QBoxLayout(QBoxLayout.TopToBottom)
        self._widget.scrollAreaWidgetContents.setLayout(self._layout)

        self._cancel_button = QPushButton('cancel current task', self._widget.scrollAreaWidgetContents)
        self._cancel_button.clicked.connect(self.cancel_button_click_slot)
        self._cancel_button.setSizePolicy(QSizePolicy.MinimumExpanding,
                                          QSizePolicy.MinimumExpanding)
        self._layout.addWidget(self._cancel_button)


        # Signals

        self._receive_task_list_signal.connect(self.receive_task_list_slot)
        self._refresh_button_highlighting_signal.connect(self.refresh_button_highlighting)

        # ROS

        self._task_list_sub = rospy.Subscriber('/task/available_tasks', String, self.task_list_callback)
        self._task_pub = rospy.Publisher('/task', String)

        self._action_client = SimpleActionClient('activate_task', TaskActivationAction)
#        self._action_client.wait_for_server()


    def task_list_callback(self, msg):
        task_list = [task.strip() for task in msg.data.split(',')]
        rospy.logdebug("received tasks list: %s", msg)
        self._receive_task_list_signal.emit(task_list)

    @Slot(list)
    def receive_task_list_slot(self, task_list):
        rospy.logdebug("slot received task list: %s", task_list)
        self.generate_task_buttons(task_list)

    def generate_task_buttons(self, task_list):
        # remove existing tasks that aren't in received list
        # two steps to avoid modifying the dict while iterating
        tasks_to_remove = [name
                           for name in self._task_map
                           if name not in task_list]
        for name in tasks_to_remove:
            self._task_map[name].button.setParent(None)
            del self._task_map[name]

        # add received tasks that we don't have yet
        for name in task_list:
            if name not in self._task_map:
                button = QPushButton(name, self._widget.scrollAreaWidgetContents)
                button.clicked.connect(self.task_buttons_click_slot)
                button.setSizePolicy(QSizePolicy.MinimumExpanding,
                                     QSizePolicy.MinimumExpanding)
                self._layout.addWidget(button)
                self._task_map[name] = TaskInfo(name, button)

        self.refresh_button_highlighting()

    @Slot()
    def task_buttons_click_slot(self):

        clicked_button = self.sender()
        task_name = clicked_button.text()
        rospy.loginfo("activating task %s", task_name)
        self._task_pub.publish(task_name)
        goal = TaskActivationGoal()
        goal.task_id = task_name
        self._action_client.send_goal(goal, self.task_done_cb,
                                      self.task_active_cb, self.task_feedback_cb)
        if self._active_task_name is not None:
            # set active task status to unknown as we cannot track it anymore
            self._task_map[self._active_task_name].status = GoalStatus.LOST
        self._active_task_name = task_name
        self.refresh_button_highlighting()

    @Slot()
    def cancel_button_click_slot(self):
        rospy.loginfo("cancelling goal")
        self._action_client.cancel_goal()
        self.refresh_button_highlighting()


    def task_feedback_cb(self, feedback):
        rospy.loginfo("task feedback callback: %S", feedback)
        print feedback
        self._task_map[self._active_task_name].status = feedback.status.status
        self._refresh_button_highlighting_signal.emit()

    def task_done_cb(self, status, status_text):
        """Status being integer from actionlib_msgs/GoalStatus"""
        rospy.loginfo("task done callback: %s %r", status, status_text)
        self._task_map[self._active_task_name].status = status
        self._active_task_name = None
        self._refresh_button_highlighting_signal.emit()

    def task_active_cb(self):
        rospy.loginfo("task active callback: %s" %
                      self._task_map[self._active_task_name].name)
        self._task_map[self._active_task_name].status = GoalStatus.ACTIVE
        self._refresh_button_highlighting_signal.emit()


    @Slot()
    def refresh_button_highlighting(self):
        for task in self._task_map.itervalues():
            #print 'task: ', task
            if task.status is not None:
                color = TaskControl.state_colors[task.status]
                task.button.setStyleSheet('background-color: %s' % color)


        # color current button
        if self._active_task_name is not None:
            button = self._task_map[self._active_task_name].button
            color = TaskControl.state_colors[self._action_client.get_state()]
            button.setStyleSheet('background-color: %s' % color)



    def shutdown_plugin(self):
        self._task_list_sub.unregister()
        self._action_client.stop_tracking_goal()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('button_layout',
                    'horizontal'
                    if self._layout.direction() == QBoxLayout.LeftToRight
                    else 'vertical')

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value('button_layout') == 'horizontal':
            self._layout.setDirection(QBoxLayout.LeftToRight)
        else:
            self._layout.setDirection(QBoxLayout.TopToBottom)

    def trigger_configuration(self):
        if self._layout.direction() == QBoxLayout.TopToBottom:
            self._layout.setDirection(QBoxLayout.LeftToRight)
        else:
            self._layout.setDirection(QBoxLayout.TopToBottom)
