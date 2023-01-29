#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: xartiou
"""

from matplotlib import pyplot as plt

des_pos = 30.0  # целевое положение
init_pos = 0.0  # начальное положение
init_vel = 0.0  # начальная скорость
init_acc = 0.0  # начальное ускорение

# e_teta      ошибка по положению
# teta_1_cmd  целевая скорость
# teta_2_cmd целевое ускорение

pos = init_pos  # текущее положеие
vel = init_vel  # текущая скорость

m_inert = 7.16914e-05  # момент инерции
kb = 3.9865e-08  # коэфф тяги
len_base = 0.17  # длина базы

dt = 0.01  # шаг интегрирования
t_end = 10  # время моделирования
init_time = 0  # начало моделирования
time = init_time  # время
req_thrust = 100  # целевая тяга
lim_vel = 1000  # ограничение по скорости
lim_acc = 10000  # ограничение по ускорению

kP_pos = 120  # коэффициент Пропорционального регулирования отвечает время достижения результата
kI_pos = 3  # коэффициент Интегрального регулирования отвечает за статическую ошибку
kD_pos = 17  # коэффициент Дифференциального регулирования отвечает за колебания системы

kP_vel = 40
kI_vel = 10
kD_vel = 0.8

err_pos_past = 0
err_vel_past = 0
integral = 0
integral1 = 0

accList = []
velList = []
posList = []
timeList = []


# расчет ошибки
def error(input1, input2):
    return input1 - input2


# PID вычислитель
def PID(error, Kp, Ki, Kd, error_past, Lim, integral):
    P = Kp * error
    I = Ki * integral
    D = Kd * ((error - error_past) / dt)
    PID = P + I + D

    if (PID > Lim):
        PID = Lim
    elif (PID < -Lim):
        PID = -Lim

    # print(f"P:{P}, I:{I}, D: {D}")

    return PID


# модель
def model(des_acc, req_thrust, kb, len_b, m_inert):
    vel_1 = des_acc + req_thrust  # угловая скорость 1 мотора
    vel_2 = -des_acc + req_thrust  # угловая скорость 2 мотора
    m_force = kb * len_b * (vel_1 ** 2 - vel_2 ** 2)
    ang_acc = m_force / m_inert
    # print(My)
    return ang_acc


def plots():
    f = plt.figure(constrained_layout=True)
    gs = f.add_gridspec(3, 5)
    ax1 = f.add_subplot(gs[0, :-1])
    ax1.plot(posList)
    ax1.grid()
    str_title = 'position' + ' P=' + str(kP_pos) + ' I=' + str(kI_pos) + ' D=' + str(kD_pos)
    ax1.set_title(str_title)

    ax2 = f.add_subplot(gs[1, :-1])
    ax2.plot(velList, "g")
    ax2.grid()
    ax2.set_title('velocity')

    ax3 = f.add_subplot(gs[2, :-1])
    ax3.plot(accList, "r")
    ax3.grid()
    ax3.set_title('acceleration')

    plt.show()


while time <= t_end:
    err_pos = error(des_pos, pos)  # расчет ошибки по положению
    des_vel = PID(err_pos, kP_pos, kI_pos, kD_pos, err_pos_past, lim_vel, integral)  # расчет  целевой скорости
    err_pos_past = err_pos
    integral += err_pos * dt

    err_vel = error(des_vel, vel)  # расчет ошибки по скорости
    des_acc = PID(err_vel, kP_vel, kI_vel, kD_vel, err_vel_past, lim_acc, integral1)  # расчет  целевого ускорения
    err_vel_past = err_vel
    integral1 += err_vel * dt

    acc = model(des_acc, req_thrust, kb, len_base, m_inert)
    vel += acc * dt
    pos += vel * dt

    accList.append(acc)
    velList.append(vel)
    posList.append(pos)
    timeList.append(time)

    time += dt

plots()
