# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'controlsUi.ui'
#
# Created: Mon Feb 12 16:59:47 2018
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(QtGui.QWidget):
    def __init__(self):
        super(Ui_Form, self).__init__()
        self.setupUi(self)

    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.setEnabled(True)
        Form.resize(1000, 480)
        self.btnStart = QtGui.QPushButton(Form)
        self.btnStart.setGeometry(QtCore.QRect(736, 420, 98, 27))
        self.btnStart.setObjectName(_fromUtf8("btnStart"))
        self.btnStop = QtGui.QPushButton(Form)
        self.btnStop.setGeometry(QtCore.QRect(852, 420, 98, 27))
        self.btnStop.setObjectName(_fromUtf8("btnStop"))
        self.groupBoxTrajectory = QtGui.QGroupBox(Form)
        self.groupBoxTrajectory.setGeometry(QtCore.QRect(40, 22, 191, 201))
        self.groupBoxTrajectory.setObjectName(_fromUtf8("groupBoxTrajectory"))
        self.formLayoutWidget = QtGui.QWidget(self.groupBoxTrajectory)
        self.formLayoutWidget.setGeometry(QtCore.QRect(10, 27, 160, 175))
        self.formLayoutWidget.setObjectName(_fromUtf8("formLayoutWidget"))
        self.formLayout = QtGui.QFormLayout(self.formLayoutWidget)
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setMargin(0)
        self.formLayout.setVerticalSpacing(5)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.label_mx = QtGui.QLabel(self.formLayoutWidget)
        self.label_mx.setObjectName(_fromUtf8("label_mx"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label_mx)
        self.lineEdit_mx = QtGui.QLineEdit(self.formLayoutWidget)
        self.lineEdit_mx.setObjectName(_fromUtf8("lineEdit_mx"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.lineEdit_mx)
        self.label_my = QtGui.QLabel(self.formLayoutWidget)
        self.label_my.setObjectName(_fromUtf8("label_my"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_my)
        self.lineEdit_my = QtGui.QLineEdit(self.formLayoutWidget)
        self.lineEdit_my.setObjectName(_fromUtf8("lineEdit_my"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.lineEdit_my)
        self.label_mz = QtGui.QLabel(self.formLayoutWidget)
        self.label_mz.setObjectName(_fromUtf8("label_mz"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_mz)
        self.lineEdit_mz = QtGui.QLineEdit(self.formLayoutWidget)
        self.lineEdit_mz.setObjectName(_fromUtf8("lineEdit_mz"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.lineEdit_mz)
        self.line = QtGui.QFrame(self.formLayoutWidget)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.formLayout.setWidget(4, QtGui.QFormLayout.FieldRole, self.line)
        self.label_4 = QtGui.QLabel(self.formLayoutWidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.formLayout.setWidget(5, QtGui.QFormLayout.LabelRole, self.label_4)
        self.lineEdit_veces = QtGui.QLineEdit(self.formLayoutWidget)
        self.lineEdit_veces.setObjectName(_fromUtf8("lineEdit_veces"))
        self.formLayout.setWidget(5, QtGui.QFormLayout.FieldRole, self.lineEdit_veces)
        self.lineEdit_myaw = QtGui.QLineEdit(self.formLayoutWidget)
        self.lineEdit_myaw.setObjectName(_fromUtf8("lineEdit_myaw"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.lineEdit_myaw)
        self.label_mz_2 = QtGui.QLabel(self.formLayoutWidget)
        self.label_mz_2.setObjectName(_fromUtf8("label_mz_2"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_mz_2)
        self.groupBoxTest = QtGui.QGroupBox(Form)
        self.groupBoxTest.setGeometry(QtCore.QRect(250, 22, 121, 111))
        self.groupBoxTest.setObjectName(_fromUtf8("groupBoxTest"))
        self.verticalLayoutWidget = QtGui.QWidget(self.groupBoxTest)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 111, 80))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.radioButton_sim = QtGui.QRadioButton(self.verticalLayoutWidget)
        self.radioButton_sim.setChecked(True)
        self.radioButton_sim.setObjectName(_fromUtf8("radioButton_sim"))
        self.verticalLayout.addWidget(self.radioButton_sim)
        self.radioButton_real = QtGui.QRadioButton(self.verticalLayoutWidget)
        self.radioButton_real.setObjectName(_fromUtf8("radioButton_real"))
        self.verticalLayout.addWidget(self.radioButton_real)
        self.groupBoxFiles = QtGui.QGroupBox(Form)
        self.groupBoxFiles.setGeometry(QtCore.QRect(40, 250, 551, 251))
        self.groupBoxFiles.setObjectName(_fromUtf8("groupBoxFiles"))
        self.gridLayoutWidget = QtGui.QWidget(self.groupBoxFiles)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(9, 29, 531, 91))
        self.gridLayoutWidget.setObjectName(_fromUtf8("gridLayoutWidget"))
        self.layout_files = QtGui.QGridLayout(self.gridLayoutWidget)
        self.layout_files.setMargin(0)
        self.layout_files.setHorizontalSpacing(4)
        self.layout_files.setObjectName(_fromUtf8("layout_files"))
        self.label_2 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.layout_files.addWidget(self.label_2, 0, 0, 1, 1)
        self.btn_launch = QtGui.QPushButton(self.gridLayoutWidget)
        self.btn_launch.setObjectName(_fromUtf8("btn_launch"))
        self.layout_files.addWidget(self.btn_launch, 0, 2, 1, 1)
        self.lineEdit_launch = QtGui.QLineEdit(self.gridLayoutWidget)
        self.lineEdit_launch.setObjectName(_fromUtf8("lineEdit_launch"))
        self.layout_files.addWidget(self.lineEdit_launch, 0, 1, 1, 1)
        self.btn_control = QtGui.QPushButton(self.gridLayoutWidget)
        self.btn_control.setObjectName(_fromUtf8("btn_control"))
        self.layout_files.addWidget(self.btn_control, 1, 2, 1, 1)
        self.lineEdit_control = QtGui.QLineEdit(self.gridLayoutWidget)
        self.lineEdit_control.setObjectName(_fromUtf8("lineEdit_control"))
        self.layout_files.addWidget(self.lineEdit_control, 1, 1, 1, 1)
        self.label_3 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.layout_files.addWidget(self.label_3, 1, 0, 1, 1)
        self.groupBoxTest_2 = QtGui.QGroupBox(Form)
        self.groupBoxTest_2.setGeometry(QtCore.QRect(250, 140, 91, 111))
        self.groupBoxTest_2.setObjectName(_fromUtf8("groupBoxTest_2"))
        self.verticalLayoutWidget_2 = QtGui.QWidget(self.groupBoxTest_2)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 20, 71, 80))
        self.verticalLayoutWidget_2.setObjectName(_fromUtf8("verticalLayoutWidget_2"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setMargin(0)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.checkBox_save = QtGui.QCheckBox(self.verticalLayoutWidget_2)
        self.checkBox_save.setObjectName(_fromUtf8("checkBox_save"))
        self.verticalLayout_2.addWidget(self.checkBox_save)
        self.checkBox_show = QtGui.QCheckBox(self.verticalLayoutWidget_2)
        self.checkBox_show.setObjectName(_fromUtf8("checkBox_show"))
        self.verticalLayout_2.addWidget(self.checkBox_show)
        self.groupBox = QtGui.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(420, 20, 201, 91))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.formLayoutWidget_2 = QtGui.QWidget(self.groupBox)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(10, 29, 181, 51))
        self.formLayoutWidget_2.setObjectName(_fromUtf8("formLayoutWidget_2"))
        self.formLayout_2 = QtGui.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout_2.setMargin(0)
        self.formLayout_2.setObjectName(_fromUtf8("formLayout_2"))
        self.label_5 = QtGui.QLabel(self.formLayoutWidget_2)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.formLayout_2.setWidget(0, QtGui.QFormLayout.LabelRole, self.label_5)
        self.label_status = QtGui.QLabel(self.formLayoutWidget_2)
        self.label_status.setText(_fromUtf8(""))
        self.label_status.setObjectName(_fromUtf8("label_status"))
        self.formLayout_2.setWidget(0, QtGui.QFormLayout.FieldRole, self.label_status)
        self.label_8 = QtGui.QLabel(self.formLayoutWidget_2)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.formLayout_2.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_8)
        self.label_battery = QtGui.QLabel(self.formLayoutWidget_2)
        self.label_battery.setText(_fromUtf8(""))
        self.label_battery.setObjectName(_fromUtf8("label_battery"))
        self.formLayout_2.setWidget(1, QtGui.QFormLayout.FieldRole, self.label_battery)
        self.groupBox_console = QtGui.QGroupBox(Form)
        self.groupBox_console.setGeometry(QtCore.QRect(620, 250, 341, 121))
        self.groupBox_console.setObjectName(_fromUtf8("groupBox_console"))
        self.label_console = QtGui.QLabel(self.groupBox_console)
        self.label_console.setGeometry(QtCore.QRect(0, 30, 331, 81))
        self.label_console.setFrameShape(QtGui.QFrame.StyledPanel)
        self.label_console.setLineWidth(2)
        self.label_console.setText(_fromUtf8(""))
        self.label_console.setObjectName(_fromUtf8("label_console"))
        self.btnReset = QtGui.QPushButton(Form)
        self.btnReset.setGeometry(QtCore.QRect(620, 420, 98, 27))
        self.btnReset.setObjectName(_fromUtf8("btnReset"))

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.btnStart.setText(_translate("Form", "Start", None))
        self.btnStop.setText(_translate("Form", "Stop", None))
        self.groupBoxTrajectory.setTitle(_translate("Form", "Trajectory", None))
        self.label_mx.setText(_translate("Form", "X:", None))
        self.label_my.setText(_translate("Form", "Y:", None))
        self.label_mz.setText(_translate("Form", "Z:", None))
        self.label_4.setText(_translate("Form", "n:", None))
        self.label_mz_2.setText(_translate("Form", "Yaw:", None))
        self.groupBoxTest.setTitle(_translate("Form", "Test", None))
        self.radioButton_sim.setText(_translate("Form", "Simulation", None))
        self.radioButton_real.setText(_translate("Form", "Experiment", None))
        self.groupBoxFiles.setTitle(_translate("Form", "Files", None))
        self.label_2.setText(_translate("Form", "launch file:", None))
        self.btn_launch.setText(_translate("Form", "Open", None))
        self.btn_control.setText(_translate("Form", "Open", None))
        self.label_3.setText(_translate("Form", "control file:", None))
        self.groupBoxTest_2.setTitle(_translate("Form", "Results", None))
        self.checkBox_save.setText(_translate("Form", "Save", None))
        self.checkBox_show.setText(_translate("Form", "Show", None))
        self.groupBox.setTitle(_translate("Form", "Drone Info", None))
        self.label_5.setText(_translate("Form", "Status", None))
        self.label_8.setText(_translate("Form", "Battery", None))
        self.groupBox_console.setTitle(_translate("Form", "Console Info", None))
        self.btnReset.setText(_translate("Form", "Reset", None))
