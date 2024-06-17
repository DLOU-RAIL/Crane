# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'fuxian.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from LiftSceneRepresentationConverter import *

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(623, 502)
        self.txtbtn = QtWidgets.QPushButton(Form)
        self.txtbtn.setGeometry(QtCore.QRect(30, 60, 89, 25))
        self.txtbtn.setObjectName("txtbtn")
        self.scenebtn = QtWidgets.QPushButton(Form)
        self.scenebtn.setGeometry(QtCore.QRect(30, 100, 89, 25))
        self.scenebtn.setObjectName("scenebtn")
        lsrc = LiftSceneRepresentationConverter("./kyw_data/obs_kyw.xlsx")
        self.scenebtn.clicked.connect(lambda:lsrc.generateSceneFile("./kyw_data/obs"))
        self.obsbtn = QtWidgets.QPushButton(Form)
        self.obsbtn.setGeometry(QtCore.QRect(30, 140, 131, 31))
        self.obsbtn.setObjectName("obsbtn")
        self.loctxtbtn = QtWidgets.QPushButton(Form)
        self.loctxtbtn.setGeometry(QtCore.QRect(30, 180, 131, 31))
        self.loctxtbtn.setObjectName("loctxtbtn")
        self.tosonbtn = QtWidgets.QPushButton(Form)
        self.tosonbtn.setGeometry(QtCore.QRect(30, 230, 89, 25))
        self.tosonbtn.setObjectName("tosonbtn")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.txtbtn.setText(_translate("Form", "地图txt生成"))
        self.scenebtn.setText(_translate("Form", "场景生成"))
        self.obsbtn.setText(_translate("Form", "障碍物位置生成"))
        self.loctxtbtn.setText(_translate("Form", "局部地图txt生成"))
        self.tosonbtn.setText(_translate("Form", "跳转页面"))
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_Form()

    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

