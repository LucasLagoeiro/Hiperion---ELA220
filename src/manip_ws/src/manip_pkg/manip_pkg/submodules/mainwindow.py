import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QIntValidator

from thread import ThreadClass

import sys, time

from ui_mainwindow import Ui_MainWindow
from robot import Robot

class MainWindow:
    def __init__(self):
        self.main_win = QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.main_win)

        self.ui_joints = [self.ui.joint1_pos, self.ui.joint2_pos, self.ui.joint3_pos, self.ui.ee_pos]

        validator = QIntValidator(0, 180)

        for i in self.ui_joints:
            i.setValidator(validator)

        self.ui.set_hp_button.clicked.connect(self.set_hp_button_clicked)
        self.ui.set_all_button.clicked.connect(self.set_all_button_clicked)

        self.ui.set_j1_button.clicked.connect(self.set_j1_button_clicked)
        self.ui.joint1_pos.returnPressed.connect(self.set_j1_button_clicked)

        self.ui.set_j2_button.clicked.connect(self.set_j2_button_clicked)
        self.ui.joint2_pos.returnPressed.connect(self.set_j2_button_clicked)

        self.ui.set_j3_button.clicked.connect(self.set_j3_button_clicked)
        self.ui.joint3_pos.returnPressed.connect(self.set_j3_button_clicked)

        self.ui.set_ee_button.clicked.connect(self.set_ee_button_clicked)
        self.ui.ee_pos.returnPressed.connect(self.set_ee_button_clicked)

        self.ui.new_mov_button.clicked.connect(self.new_move)
        self.ui.exclude_mov_button.clicked.connect(self.exclude_move)
        self.ui.previous_pos_button.clicked.connect(self.previous_position)
        self.ui.next_pos_button.clicked.connect(self.next_position)
        self.ui.save_pos_button.clicked.connect(self.save_position)

        self.ui.play_move_button.clicked.connect(self.play_move)
        self.ui.stop_move_button.clicked.connect(self.stop_move)

        self.robot = Robot()

        self.thread = ThreadClass()

        self.movements = list()
        self.atual_move = 0
        self.edit_mode = False
        self.playing = False

    def play_move(self):
        if not self.edit_mode:
            return
        
        for moves in self.movements:
            self.robot.set_all_joints_angle(moves)
        

    def stop_move(self):
        pass

    def next_position(self):
        if self.atual_move == len(self.movements):
            return
        else:
            self.atual_move += 1
        
        self.ui_atualize_move(self.atual_move)

    def previous_position(self):
        if self.atual_move <= 1:
            return
        else:
            self.atual_move -= 1

        self.ui_atualize_move(self.atual_move)
            
    def save_position(self):
        self.movements[self.atual_move-1] = self.ui_get_all_joint_angle(self.atual_move)
        print(self.movements)

    def show(self):
        self.main_win.show()

    def set_hp_button_clicked(self):
        self.robot.home_position()
        joint_angles = list()
        for angles in self.robot.atual_angles:
            joint_angles.append(angles)
        self.ui_atualize_joints_angles(joint_angles)

    def set_all_button_clicked(self):
        list_angles = self.ui_get_all_joint_angle(0)
        self.robot.set_all_joints_angle(list_angles)

    def set_j1_button_clicked(self):

        joint1_angle = self.ui_get_joint_angle(1, 0)

        self.robot.set_single_joint_angle(1, joint1_angle)
        self.ui_joints[0].setText(str(self.robot.atual_angles[0]))

    def set_j2_button_clicked(self):

        joint2_angle = self.ui_get_joint_angle(2, 0)

        self.robot.set_single_joint_angle(2, joint2_angle)
        self.ui_joints[1].setText(str(self.robot.atual_angles[1]))

    def set_j3_button_clicked(self):

        joint3_angle = self.ui_get_joint_angle(3, 0)

        self.robot.set_single_joint_angle(3, joint3_angle)
        self.ui_joints[2].setText(str(self.robot.atual_angles[2]))

    def set_ee_button_clicked(self):
        ee_angle = self.ui_get_joint_angle(4, 0)

        self.robot.set_single_joint_angle(4, ee_angle)
        self.ui_joints[3].setText(str(self.robot.atual_angles[3]))

    def ui_atualize_joints_angles(self, joint_angles):
        for i in range(4):
            self.ui_joints[i].setText(str(joint_angles[i]))

    def new_move(self):
        if not self.edit_mode:
            self.atual_move += 1
            self.edit_mode = True
            self.movements.append(self.robot.home_angles)
        else:
            #salvar os valores escritos no index do atual move
            self.movements[self.atual_move-1] = self.ui_get_all_joint_angle(self.atual_move)
            self.movements.insert(self.atual_move, self.movements[self.atual_move-1])
            self.atual_move += 1

        self.ui_atualize_move(self.atual_move)
        
        #print(self.movements)

    def exclude_move(self):
        qnt_moves = len(self.movements)
        if qnt_moves == 0:
            return
        
        elif qnt_moves == 1:
            self.atual_move -= 1
            self.movements.pop()
            self.edit_mode = False

        elif self.atual_move == 1:
            self.movements.pop(0)
        
        else:
            self.movements.pop(self.atual_move-1)
            self.atual_move -= 1
        
        self.ui_atualize_move(self.atual_move)

        print(self.movements)
    
    def clear_all_ui_joint_angles(self):
        for i in range(4):
            self.ui_joints[i].setText('')



            
        

        # if len(self.movement) != 0 :
        #     self.movement.pop()
        #     self.ui.move_label.setText(f'Move {len(self.movement)+1}')
        # else:
        #     self.ui.move_label.setText('')
        #     self.edit_mode = False

    def ui_get_joint_angle(self, joint_number, move): #se move = 0 ou 1 usa angulos atuais
        joint_number -= 1
        if move <= 1:
            joint_angle = self.robot.atual_angles[joint_number]
        else:
            joint_angle = self.movements[move-1]

        text = self.ui_joints[joint_number].text()
        if text.isnumeric():
            joint_angle = int(text)
        print(f'Joint {joint_number}: {joint_angle}')
        return joint_angle    

    def ui_get_all_joint_angle(self, move):
        joint_angles = list()

        for i in range(1,5):
            joint_angles.append(self.ui_get_joint_angle(i, move))


        print(joint_angles)

        return joint_angles   

    def ui_atualize_move(self, move):
        if move == 0:
            self.ui.move_label.setText('')
            self.clear_all_ui_joint_angles()
        else:
            self.ui.move_label.setText(f'Move {self.atual_move}')
            self.ui_atualize_joints_angles(self.movements[self.atual_move-1])

        print(self.movements)




    




if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())
