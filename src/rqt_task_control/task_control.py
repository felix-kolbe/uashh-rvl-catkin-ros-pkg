import os

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QWidget, QPushButton, QGridLayout, QSizePolicy

from std_msgs.msg import String

class TaskControl(Plugin):

    _task_list_receiver_signal = Signal(list)

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


        self._task_list_sub = rospy.Subscriber('/task/available_tasks', String, self.task_list_callback)
        self._task_pub = rospy.Publisher('/task', String)

        self._task_list_receiver_signal.connect(self.receive_task_list_slot)

        self._task_buttons = []

        self._layout = QGridLayout()
        self._widget.scrollAreaWidgetContents.setLayout(self._layout)



    def task_list_callback(self, msg):
        task_list = [task.strip() for task in msg.data.split(',')]
        #print "received tasks list: ", msg
        #self.generate_task_buttons(task_list)
        self._task_list_receiver_signal.emit(task_list)

    @Slot(list)
    def receive_task_list_slot(self, task_list):
        print "slot received task list:", task_list
        self.generate_task_buttons(task_list)

    def generate_task_buttons(self, task_list):
        for button in self._task_buttons:
            button.setParent(None)
        self._task_buttons = []

        for task in task_list:
            button = QPushButton(task, self._widget.scrollAreaWidgetContents)
            button.clicked.connect(self.task_buttons_click_slot)
            button.setSizePolicy(QSizePolicy.MinimumExpanding,
                                 QSizePolicy.MinimumExpanding)
            self._layout.addWidget(button)
            self._task_buttons.append(button)


    @Slot()
    def task_buttons_click_slot(self):
        clicked_button = self.sender()
        clicked_task = clicked_button.text()
        print clicked_task
        self._task_pub.publish(clicked_task)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
