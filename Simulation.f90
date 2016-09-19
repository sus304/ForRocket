!========================================================================
! ForRocket    (Rocket Flight Simulater For Fortran)
!
!The MIT License (MIT)
!Copyright (c) 2015  Susumu Tanaka
!This program is released under the MIT License.
!
!Permission is hereby granted, free of charge, to any person obtaining a copy of
!this software and associated documentation files (the "Software"),
!to deal in the Software without restriction, including without limitation
!the rights to use, copy, modify, merge, publish, distribute, sublicense,
!and/or sell copies of the Software, and to permit persons to whom
!the Software is furnished to do so, subject to the following conditions:
!
!The above copyright notice and this permission notice shall be included
!in all copies or substantial portions of the Software.
!
!THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
!EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
!MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
!IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
!DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
!TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
!THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
!                  
!========================================================================

!
! Coordinate : East - North - Up
!

program Simulation
use standard_collection
use Rocket_Class
use Rocket_Dynamics
use atmosphere1976 
use environment
implicit none

!**********************************************************************
!                        Program Variable
!**********************************************************************

real :: t1,t2
!計算時間測定用
real(8) :: dummy1,dummy2
!機体データスキップ用

integer :: i

!**********************************************************************
!                        Rocket Variable
!**********************************************************************

type(Rocket_Type) :: Rocket
  
integer :: Flight_Status = 0

integer :: code = 0,eof = 0
!   codeは開傘判定用
!   code = 0 : 弾道
!   code = 1 : 開傘

!-Mach-Cdスプライン用
real(8),allocatable :: Mach_base(:),Cd_base(:)
integer :: n_base = 0

real(8) :: t,dt

!------------------------------------
!-            Condition
!------------------------------------
real(8) :: Vw_abs,Vw_angle,Vw(3) !-風速,風向
real(8) :: Hw,Wh !-風速測定高度,高度係数
real(8) :: Pa,Ta,rho,g,Cs
real(8) :: Ta_0,Pa_0,rho_0,g0
real(8) :: LL !-ランチャレール長[m]

!------------------------------------
!-              Save
!------------------------------------
real(8) :: t_top,Position_top(3),Va_top,Ve_top(3),m_top
real(8) :: V_lc,Va_max,max_Q,Mach_max
real(8) :: t_sepa2


!**********************************************************************
!                        Switch Variable
!**********************************************************************
real(8) :: Vw_min,Vw_max,Vw_delta,Vw_angle_min,Vw_angle_max,Vw_angle_delta
integer :: Landing_Range_table_sw,Va_top_table_sw,Z_max_table_sw,Vlc_table_sw,t_table_sw,Va_max_table_sw
integer :: Decent_sw,time_sw,position_sw,Attitude_sw,Va_sw,acc_sw


!**********************************************************************

call cpu_time(t1)

!**********************************************************************
!                Data Input(Read) & Pre Setting
!**********************************************************************
!------------------------------------
!-             Rocket
!------------------------------------

open (100,file='rocket_param.dat',status='old')
read (100,*) Rocket%l ; read (100,*) Rocket%d ; read (100,*) Rocket%ms
read (100,*) Rocket%lcgs ; read (100,*) Rocket%lcgox_0 ; read (100,*) Rocket%lcgf ; read (100,*) Rocket%ltank
read (100,*) Rocket%Is ; read (100,*) Rocket%Ir
read (100,*) Rocket%lcp ; read (100,*) Rocket%Clp ; read (100,*) Rocket%Cmq ; read (100,*) Rocket%Cd ; read (100,*) Rocket%CNa
read (100,*) Rocket%M_fin
read (100,*) Rocket%CdS1 ; read (100,*) Rocket%CdS2
read (100,*) dt ; read (100,*) Rocket%de ; read (100,*) Rocket%Isp ; read (100,*) Rocket%mox_0
read (100,*) Rocket%mf_b ; read (100,*) Rocket%mf_a ; read (100,*) Rocket%lf ; read (100,*) Rocket%df1 ; read (100,*) Rocket%df2
read (100,*) Rocket%mox_dot ; read (100,*) Rocket%mf_dot
read (100,*) dummy1 ; read (100,*) dummy2 !------------Space Skip---------------
read (100,*) Pa_0 ; read (100,*) Ta_0 ; read (100,*) rho ; read (100,*) g0
read (100,*) Hw ; read (100,*) Wh ; read (100,*) Rocket%T_sepa ; read (100,*) Rocket%H_sepa
read (100,*) Rocket%theta_0 ; read (100,*) Rocket%psi_0 ; read (100,*) LL
close(100)

call sign_check(Rocket%Clp,-1)
call sign_check(Rocket%Cmq,-1)
call sign_check(Rocket%theta_0,-1)
!-よくあるエラー回避

!------------------------------------
!-           Mach-Cd
!------------------------------------

open (103,file='Mach_Cd.dat',status='old')
read (103,*) !１行目のスキップ
do
  read (103,'()',end=80)
  n_base = n_base + 1
end do
80 close(103)

allocate(Mach_base(n_base),Cd_base(n_base))

open (103,file='Mach_Cd.dat',status='old')
read (103,*) !1行目のスキップ
do i = 1,n_base
  read (103,*) Mach_base(i),Cd_base(i)
end do
close(103)

call spline(Mach_base,Cd_base,n_base-1,Rocket%Mach,Rocket%Cd,0)

!------------------------------------
!-          Pre Setting
!------------------------------------
dt = 1.0d0 / dt
Rocket%Cnr = Rocket%Cmq

!--------Diameter to Area---------
Rocket%S = Rocket%d**2 * PI / 4.0d0
Rocket%Ae = (Rocket%de / 1000.0d0)**2 * PI / 4.0d0

!----------kitai to nose------------
Rocket%lcgf = Rocket%l - Rocket%lcgf
Rocket%lcgox_0 = Rocket%l - Rocket%lcgox_0


Pa_0 = Pa_0 * 1000.0d0 !---------kPa to Pa---------
Ta_0 = Ta_0 + 273.15d0
rho_0 = Pa_0 / (R*Ta_0)

!------deg to rad--------
Rocket%theta_0 = deg2rad(Rocket%theta_0)
Rocket%psi_0 = deg2rad(Rocket%psi_0)

!---------Zero_data_Save----------
Rocket%lcgp_0 = (Rocket%lcgox_0 * Rocket%mox_0 + Rocket%lcgf * Rocket%mf_b) / (Rocket%mox_0 + Rocket%mf_b)
Rocket%lcg_0 = (((Rocket%mox + Rocket%mf_b) * Rocket%lcgp_0) + (Rocket%ms * Rocket%lcgs))&
                 / (Rocket%ms + (Rocket%mf_b + Rocket%mox_0))


!**********************************************************************
!                        Output File Open
!**********************************************************************
!------------------------------------
!-            Switch
!------------------------------------

open (101,file='switch.dat',status='old')

read (101,*) Vw_min ; read (101,*) Vw_max ; read (101,*) Vw_delta
read (101,*) Vw_angle_min ; read (101,*) Vw_angle_max ; read (101,*) Vw_angle_delta
read (101,*) Landing_Range_table_sw ; read (101,*) Vlc_table_sw ; read (101,*) Va_max_table_sw ; read (101,*) Va_top_table_sw
read (101,*) t_table_sw ; read (101,*) Z_max_table_sw

read (101,*) Decent_sw
read (101,*) time_sw ; read (101,*) position_sw ; read (101,*) Attitude_sw ; read (101,*) Va_sw ; read (101,*) acc_sw

close(101)

!------------------------------------
!-      For Excel Landing Range
!------------------------------------

if (Landing_Range_table_sw == 1) then
  open (210,file='Output/Landing_Range_v2.csv')
end if

!------------------------------------
!-        Table for Analysis
!------------------------------------

if (Vlc_table_sw == 1) open (201,file='Output/Velocity_of_Launch_Clear_table.csv')
if (Va_max_table_sw == 1) then
  open (202,file='Output/Va_max_table.csv')
  open (203,file='Output/Mach_max_table.csv')
  open (204,file='Output/Q_max_table.csv')
end if
if (Va_top_table_sw == 1) open (205,file='Output/Va_Top_table.csv')
if (t_table_sw == 1) then
  open (206,file='Output/time_top_table.csv')
  open (211,file='Output/time_Second_Para_table.csv')
  open (207,file='Output/time_Hard_Landing_table.csv')
  open (208,file='Output/time_Para_Landing_table.csv')
end if
if (Z_max_table_sw == 1) open (209,file='Output/Max_Altitude_table.csv')

!------------------------------------
!-        Log for Analysis
!------------------------------------

if (time_sw == 1) then
  open (250,file='Output_Log/time_log.csv')
  write (250,*) 't'
end if
if (position_sw == 1) then
  open (251,file='Output_Log/Position_log.csv')
  write (251,*) 'X',',','Y',',','Z'
end if
if (Attitude_sw == 1) then
  open (252,file='Output_Log/Attitude_log.csv')
  write (252,*) 'theta',',','psi',',','fai',',','alpha',',','beta'
end if
if (Va_sw == 1) then
  open (253,file='Output_Log/Va_log.csv')
  write (253,*) 'Vax',',','Vay',',','Vaz',',','Va',',','Mach'
end if
if (acc_sw == 1) then
  open (254,file='Output_Log/Acc_log.csv')
  write (254,*) 'accx',',','accy',',','accz',',','acc'
end if 

!open (900,file='debug.csv')

!****************************************************************************************************
!                                       Calculate Process
!****************************************************************************************************

print *, 'Calculation Start!!'
print '(a10,a10,a10,a10,a10)', 'Vw','Vw_angle','Top','Altitude','Landing'

!**********************************************************************
!                        Wind Velocity Loop
!**********************************************************************
Vw_abs = Vw_min
do
!**********************************************************************
!                         Wind Angle Loop
!**********************************************************************
Vw_angle = Vw_angle_min
do
!**********************************************************************
!                     Open the Parachute Loop
!**********************************************************************

do code=0,1
if (code == 1) goto 400 !----------descend of Parachute ShortCut------------
!--------------------------------------
!-            Initialize
!--------------------------------------
Flight_Status = 0 ; eof = 0
t = 0.0d0

Ta = Ta_0 ; Pa = Pa_0 ; rho = rho_0

call Initialize(Rocket)

Position_top = 0.0d0
Ve_top = 0.0d0
t_top = 0.0d0 ; Va_top = 0.0d0 ; m_top = 0.0d0
V_lc = 0.0d0 ; Va_max = 0.0d0 ; max_Q = 0.0d0 ; Mach_max = 0.0d0

open (62,file='thrust.dat',status='old') !----------Thrust Data Open----------
!推力読み取りはメインループの中

400 if (code == 1) then
  t = t_top
  Rocket%Position = Position_top
  Rocket%Ve = Ve_top
  Rocket%Va_abs = Va_top
  Rocket%m = m_top
  Flight_Status = 3
end if

!**********************************************************************
!
!                           Main Calculation
!
!**********************************************************************

do

  if (eof >= 0) then
    read (62,*,iostat = eof) Rocket%thrust
  else
    eof = -1
    Rocket%thrust = 0.0d0
  end if

call VelocityOfWind_Vecter(Vw_abs,Vw_angle,Rocket%Position(3),Hw,Wh,Vw)
call Atmosphere_Setting(Ta_0,Pa_0,rho_0,g0,Rocket%Position(3),g,Cs,Ta,Pa,rho)

if (code == 1) goto 500 !----------descend of Parachute ShortCut------------

if (Flight_Status > 0) then
  call Airspeed_Calc(Rocket,Vw,Cs,rho)
  if (Va_max < Rocket%Va_abs) then
    Va_max = Rocket%Va_abs
  end if
  if (Mach_max < Rocket%Mach) then
    Mach_max = Rocket%Mach
  end if
  if (max_Q < Rocket%Q) then
    max_Q = Rocket%Q
  end if
end if

call Mass_Calc(Rocket,g0,dt)
call MI_Calc(Rocket)
call spline(Mach_base,Cd_base,n_base-1,Rocket%Mach,Rocket%Cd,1)
call Load_Calc(Rocket,Pa,Pa_0,eof)
call Moment_Calc(Rocket)

call Acceleration_Integral(Rocket,g,dt)

!-----------Parachute Descent----------
500 if (code == 1) then
  if (Rocket%H_sepa > 0.0d0 .and. Rocket%Position(3) <= Rocket%H_sepa) then !TSSS ON and Under Altitude  
    Rocket%CdS = Rocket%CdS2 + Rocket%CdS1
  else !TSSS OFF / Over Altitude on TSSS Altitude
    Rocket%CdS = Rocket%CdS1
    t_sepa2 = t
  end if  
  call Parachute_Aerodynamics(Rocket,rho,g,Vw,dt)  
end if
!--------------------------------------

call Velocity_Integral(Rocket,dt)

if (Flight_Status <= 1) then
  Rocket%omega = 0.0d0
else
  call ODE_Solve(Rocket,dt)
end if
call Quat2Euler(Rocket%Ceb,Rocket%theta,Rocket%psi,Rocket%fai)

!-----------------------------------
!-         Flight State Check
!-
!- 0 : ~Lift Off
!- 1 : ~Launch Clear
!- 2 : ~Max Altitude
!- 3 : ~Landing
!- 4 : Landed
!-----------------------------------

select case (Flight_Status)
  case (0)
    if (Rocket%Position(3) > Rocket%Position_pre(3)) then
      Flight_Status = 1
    else
      t = 0.0d0
      Rocket%Position(3) = Rocket%Position_pre(3)
    end if
  
  case (1)
    if ((Rocket%Position(3) / sin(deg2rad(abs(Rocket%theta)))) > LL) then
      V_lc = Rocket%Ve_abs
      Flight_Status = 2
    end if
  
  case (2)
    if (Rocket%Position(3) < Rocket%Position_pre(3)) then
      Position_top = Rocket%Position_pre
      Ve_top = Rocket%Ve_pre
      Va_top = Abs_Vector(Rocket%Va_pre)
      m_top = Rocket%m ; t_top = t - dt
      Flight_Status = 3
    end if
  
  case (3)
   if (Rocket%Position(3) <= 0.0d0) then
    Flight_Status = 4
  end if
end select

!------------------------------------
!-           Debug Code
!------------------------------------

!write (900,*) t,',',Flight_Status,',',eof,',',mp,',',m,',',lcg,',',rho,',',g,',',thrust,',',F(1),',',F(2),',',F(3),',',Ve(1)&
  !,',',Ve(2),',',Ve(3),',',Va(1),',',Va(2),',',Va(3),',',acc(1),',',acc(2),',',acc(3),',',Position(1),',',Position(2)&
  !,',',Position(3),',',omega(1),',',omega(2),',',omega(3),',',alpha,',',beta,',',theta,',',psi,',',fai,',',quat(1),',',quat(2)&
  !,',',quat(3),',',quat(4)
!write (900,*) t,',',Rocket%thrust,',',Pa_0,',',Pa!,',',mf,',',Ipp,',',ms,',',m,',',lcgp,',',lcgs!,',',Kj(3)


!**********************************************************************
!                      Output Result Log
!**********************************************************************

if (Decent_sw == 0) then
  if (code == 0) then
    if (time_sw == 1) write (250,*) t
    if (position_sw == 1) write (251,*) Rocket%Position(1),',',Rocket%Position(2),',',Rocket%Position(3)
    if (Attitude_sw == 1) write(252,*) Rocket%theta,',',Rocket%psi,',',Rocket%fai,',',rad2deg(Rocket%alpha),',',rad2deg(Rocket%beta)
    if (Va_sw == 1) write (253,*) Rocket%Va(1),',',Rocket%Va(2),',',Rocket%Va(3),',',Rocket%Va_abs,',',Rocket%Mach
    if (acc_sw == 1) write (254,*) Rocket%acce(1),',',Rocket%acce(2),',',Rocket%acce(3),',',Rocket%acce_abs
  end if
else !------ code == 1 -------
  if (code == 1) then
    if (time_sw == 1) write (250,*) t
    if (position_sw == 1) write (251,*) Rocket%Position(1),',',Rocket%Position(2),',',Rocket%Position(3)
    if (Va_sw == 1) write (253,*) Rocket%Ve(1),',',Rocket%Ve(2),',',Rocket%Ve(3),',',Rocket%Ve_abs
  end if
end if

!**********************************************************************
!               Output Result Landing Range & Table
!**********************************************************************

if (Flight_Status == 4) then
  if (code == 0) then
    if (Landing_Range_table_sw == 1) then
      write (210,'(2(f0.3,a)$)') Rocket%Position(1),',',Rocket%Position(2),','
    end if
    if (Vlc_table_sw == 1) write (201,'(f0.3,a,$)') V_lc,','
    if (Va_max_table_sw == 1) then
      write (202,'(f0.3,a,$)') Va_max,','
      write (203,'(f0.3,a,$)') Mach_max,','
      write (204,'(f0.3,a,$)') max_Q,','
    end if
    if (Va_top_table_sw == 1) write (205,'(f0.3,a,$)') Va_top,','
    if (t_table_sw == 1) then
      write (206,'(f0.3,a,$)') t_top,','
      write (207,'(f0.3,a,$)') t,','
    end if
    if (Z_max_table_sw == 1) write (209,'(f0.3,a,$)') Position_top(3),','
      
  else !------ code == 1 -------
    
    if (Landing_Range_table_sw == 1) then
      write (210,*) Rocket%Position(1),',',Rocket%Position(2)
    end if
    if (t_table_sw == 1) write (208,'(f0.3,a,$)') t,','
    if (t_table_sw == 1) write (211,'(f0.3,a,$)') t_sepa2,','
  end if

  exit !********** Main Calculation Loop exit ************
  
end if

t = t + dt

end do
!************** End of Main Calculate *****************

end do
!************** End of Open of parachute? *****************
close(62)
print '(f10.2,f10.2,f10.2,f10.2,f10.2)',Vw_abs,Vw_angle,t_top,Position_top(3),t

Vw_angle = Vw_angle + Vw_angle_delta
if (Vw_angle > Vw_angle_max .or. Vw_angle >= 360.0d0) exit

end do
!************** End of Wind_Angle *****************


!------------------------------------
!-         Output Adjustment
!------------------------------------

if (Vlc_table_sw == 1) write (201,*)
if (Va_max_table_sw == 1) then
  write (202,*) ; write (203,*) ; write (204,*)
end if
if (Va_top_table_sw == 1) write (205,*)
if (t_table_sw == 1) then
  write (206,*) ; write (211,*) ; write (207,*) ; write (208,*)
end if
if (Z_max_table_sw == 1) write (209,*)

Vw_abs = Vw_abs + Vw_delta
if (Vw_abs > Vw_max) exit

end do
!************* End of Wind_Velocity ****************


call cpu_time(t2)

print *, "CPU_Time:",(t2-t1)*1000.0d0," msec"

!**********************************************************************
!                       Output File Close
!**********************************************************************

if (Landing_Range_table_sw == 1) then
  close(210)
end if
if (Vlc_table_sw == 1) close(201)
if (Va_max_table_sw == 1) then
  close(202) ; close(203) ; close(204)
end if
if (Va_top_table_sw == 1) close(205)
if (t_table_sw == 1) then
  close(206) ; close(211) ; close(207) ; close(208)
end if
if (Z_max_table_sw == 1) close(209)
if (time_sw == 1) close(250)
if (position_sw == 1) close(251)
if (Attitude_sw == 1) close(252)
if (Va_sw == 1) close(253)
if (acc_sw == 1) close(254)

contains

subroutine Initialize(Rocket)
  type(Rocket_Type) :: Rocket
    Rocket%mox = Rocket%mox_0 ; Rocket%mf = Rocket%mf_b
    Rocket%Va = 0.0d0 ; Rocket%Va_pre = 0.0d0 ; Rocket%Va_abs = 0.0d0
    Rocket%Mach = 0.0d0 ; Rocket%Mach_pre = 0.0d0 ; Rocket%Q = 0.0d0
    Rocket%alpha = 0.0d0 ; Rocket%beta = 0.0d0
    Rocket%thrust = 0.0d0
    Rocket%fai = 0.0d0 ; Rocket%theta = Rocket%theta_0 ; Rocket%psi = Rocket%psi_0
    Rocket%omega = 0.0d0 ; Rocket%omega_pre = 0.0d0
    call Euler2Quat(Rocket%theta,Rocket%psi,Rocket%fai,Rocket%Ceb,Rocket%Cbe,Rocket%quat)
    
    Rocket%acce = 0.0d0 ; Rocket%acce_pre = 0.0d0
    Rocket%Ve = 0.0d0 ; Rocket%Ve_pre = 0.0d0
    Rocket%Position = 0.0d0
    Rocket%Position(2) = (Rocket%l - Rocket%lcg_0) * cos(abs(Rocket%theta_0))
    Rocket%Position(3) = (Rocket%l - Rocket%lcg_0) * sin(abs(Rocket%theta_0))
    
end subroutine Initialize


end program Simulation