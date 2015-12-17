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
!the Software is furnished to do so, subject to the following conditions:!
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

program Simlation
use standard_collection
use atmosphere1976 
use environment
use airframe
use translation
use rotation
implicit none

!**********************************************************************
!                        Program Variable
!**********************************************************************

integer :: Flight_Status = 0
! 0 : ~Lift Off
! 1 : ~Launch Clear
! 2 : ~Max Altitude
! 3 : ~Landing
! 4 : Landed

integer :: code = 0,eof = 0
!   codeは開傘判定用
!   code = 0 : 弾道
!   code = 1 : 開傘
!   eofはend of fileの略　推力データの終端検知

real :: t1,t2
!計算時間測定用

real(8) :: dummy1,dummy2
!機体データスキップ用

integer :: i

real(8) :: t,dt,range,compass

!**********************************************************************
!                        Rocket Variable
!**********************************************************************
!------------------------------------
!-            Structure
!------------------------------------
real(8) :: ms,m
real(8) :: d,S
real(8) :: l,lcgs,lcg,lcp,ltank,lcg_0
real(8) :: Is,Ir,Ib(3)
real(8) :: Clp,Cmq,Cnr,Cd,CNa,M_fin,CdS1,CdS2,CdS

!------------------------------------
!-             Engine
!------------------------------------
real(8) :: mox,mox_0,mf,mf_b,mf_a,mox_dot,mf_dot,mp,mp_dot
real(8) :: lcgp,lcgox,lcgf,lcgox_0,lcgp_0
real(8) :: Ifp,Ifr,lf,df1,df2
real(8) :: Isp
real(8) :: thrust = 0.0d0

!------------------------------------
!-           Translation
!------------------------------------
real(8) :: Dx,Ny,Nz,F(3)
real(8) :: Position(3),Ve(3),Ve_syn,Va(3),Va_syn,acc(3),acc_syn
real(8) :: Position_pre(3),Ve_pre(3),Va_pre(3),acc_pre(3),Q,Q_pre
real(8) :: Mach,Cs
real(8) :: Mach_pre
real(8),allocatable :: Mach_base(:),Cd_base(:)
integer :: n_base = 0

!------------------------------------
!-             Rotation
!------------------------------------
real(8) :: Ma(3),Ka(3),Kj(3)
real(8) :: omega(3),omega_pre(3),theta,psi,fai,alpha,beta,dY(4),dZ(4)
real(8) :: theta_0,psi_0

!------------------------------------
!-            Quaternion
!------------------------------------
real(8) :: quat(4),quat_pre(4)
real(8),dimension(3,3) :: Cbe,Ceb

!------------------------------------
!-            Condition
!------------------------------------
real(8) :: Vw_syn,Vw_angle,Vw(3),Hw,Wh
real(8) :: P,Ta,rho,Ta_0,P_0,rho_0,g,g0
real(8) :: T_sepa,H_sepa,LL

!------------------------------------
!-              Save
!------------------------------------
real(8) :: Position_top(3),t_top,Va_top,Ve_top(3),m_top
real(8) :: V_lc,Va_max,max_Q,Mach_max
real(8) :: t_sepa2


!**********************************************************************
!                     Release Object Variable
!**********************************************************************
real(8) :: t_release

type Release
  real(8) :: m,CdS,Position(3),Position_pre(3),Ve(3),Ve_syn,Va_syn,Ve_pre(3),acc(3)
end type Release

type(Release) :: nose

!構 造 体 の 代 入 の 仕 方 : name%element = value

!**********************************************************************
!                        Switch Variable
!**********************************************************************
real(8) :: Vw_min,Vw_max,Vw_delta,Vw_angle_min,Vw_angle_max,Vw_angle_delta
integer :: Landing_Range_table_sw,Va_top_table_sw,Z_max_table_sw,Vlc_table_sw,t_table_sw,Va_max_table_sw
integer :: time_sw,position_sw,Attitude_sw,Va_sw,acc_sw


!**********************************************************************

call cpu_time(t1)

!**********************************************************************
!                Data Input(Read) & Pre Setting
!**********************************************************************
!------------------------------------
!-             kitai
!------------------------------------

open (100,file='rocket_param.dat',status='old')
read (100,*) l ; read (100,*) d ; read (100,*) ms ; read (100,*) lcgs ; read (100,*) lcgox_0 ; read (100,*) lcgf
read (100,*) ltank ; read (100,*) Is ; read (100,*) Ir ; read (100,*) nose%m
read (100,*) lcp ; read (100,*) Clp ; read (100,*) Cmq ; read (100,*) Cd ; read (100,*) CNa ; read (100,*) M_fin
read (100,*) CdS1 ; read (100,*) CdS2 ; read (100,*) t_release ; read (100,*) nose%CdS
read (100,*) dt ; read (100,*) Isp ; read (100,*) mox_0 ; read (100,*) mf_b ; read (100,*) mf_a
read (100,*) lf ; read (100,*) df1 ; read (100,*) df2 ; read (100,*) mox_dot ; read (100,*) mf_dot
read (100,*) dummy1 ; read (100,*) dummy2 !------------Space Skip---------------
read (100,*) P_0 ; read (100,*) Ta_0 ; read (100,*) rho ; read (100,*) g0
read (100,*) Hw ; read (100,*) Wh ; read (100,*) T_sepa ; read (100,*) H_sepa
read (100,*) theta_0 ; read (100,*) psi_0 ; read (100,*) LL
close(100)


!------------------------------------
!-            Switch
!------------------------------------

open (101,file='switch.dat',status='old')
read (101,*) Vw_min ; read (101,*) Vw_max ; read (101,*) Vw_delta
read (101,*) Vw_angle_min ; read (101,*) Vw_angle_max ; read (101,*) Vw_angle_delta
read (101,*) Landing_Range_table_sw ; read (101,*) Vlc_table_sw ; read (101,*) Va_max_table_sw ; read (101,*) Va_top_table_sw
read (101,*) t_table_sw ; read (101,*) Z_max_table_sw
read (101,*) time_sw ; read (101,*) position_sw ; read (101,*) Attitude_sw ; read (101,*) Va_sw
read (101,*) acc_sw
close(101)

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

call spline(Mach_base,Cd_base,n_base-1,Mach,Cd,0)

!------------------------------------
!-          Pre Setting
!------------------------------------
dt = 1.0d0 / dt
Cnr = Cmq

!--------Diameter to Area---------
S = d**2*PI/4.0d0

!----------kitai to nose------------
lcgf = l - lcgf
lcgox_0 = l - lcgox_0

!---------kPa to Pa---------
P_0 = P_0*1000.0d0
Ta_0 = Ta_0 + 273.15d0
rho_0 = P_0 / (R*Ta_0)

!------deg to rad--------
theta_0 = deg2rad(theta_0)
psi_0 = deg2rad(psi_0)

!---------Zero_data_Save----------
lcgp_0 = (lcgox_0*mox_0 + lcgf*mf_b) / (mox_0 + mf_b)
lcg_0 = (((mox + mf_b)*lcgp_0) + (ms*lcgs)) / (ms + (mf_b + mox_0))


!**********************************************************************
!                        Output File Open
!**********************************************************************
!------------------------------------
!-      For Excel Landing Range
!------------------------------------

if (Landing_Range_table_sw == 1) then
  open (200,file='Output/Landing_Range.csv')
  open (210,file='Output/Landing_Range_v2(Release_and_High).csv')
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
Vw_syn = Vw_min
do
if (Landing_Range_table_sw == 1) write (200,*) 'Wind_Velocity = ',',',Vw_syn,',','m/s'

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
!-           Initialize
!--------------------------------------
Flight_Status = 0 ; eof = 0
t = 0.0d0

Ta = Ta_0 ; P = P_0 ; rho = rho_0
mox = mox_0 ; mf = mf_b
Va = 0.0d0 ; Va_pre = 0.0d0 ; Mach_pre = 0.0d0
thrust = 0.0d0
fai = 0.0d0 ; theta = theta_0 ; psi = psi_0
omega = 0.0d0 ; omega_pre = 0.0d0
call Euler2Quat(theta,psi,fai,Ceb,Cbe,quat)

acc = 0.0d0 ; acc_pre = 0.0d0
Ve = 0.0d0 ; Ve_pre = 0.0d0
Position = 0.0d0
Position(3) = (l - lcg_0)*sin(abs(theta_0))

nose%Position = 0.0d0

Position_top = 0.0d0
Ve_top = 0.0d0
t_top = 0.0d0 ; Va_top = 0.0d0 ; m_top = 0.0d0
V_lc = 0.0d0 ; Va_max = 0.0d0 ; max_Q = 0.0d0 ; Mach_max = 0.0d0

open (62,file='thrust.dat',status='old') !----------Thrust Data Open----------
!推 力 読 み 取 り は メ イ ン ル ー プ の 中

400 if (code == 1) then
  t = t_top
  Position = Position_top
  Ve = Ve_top
  Va_syn = Va_top
  m = m_top
  Flight_Status = 3
end if


!**********************************************************************
!
!                           Main Calculation
!
!**********************************************************************

do

call VelocityOfWind_Vecter(Vw_syn,Vw_angle,Position(3),Hw,Wh,Vw)

call Atmosphere_Setting(Ta_0,P_0,rho_0,g0,Position(3),g,Cs,Ta,P,rho)

if (code == 1) goto 500 !----------descend of Parachute ShortCut------------

call Mass_Calc(thrust,Isp,g0,mf_a,mf_dot,ms,mp_dot,mox_dot,mp,m,mf,mox,dt)

call MI_Calc(lcgox_0,mox,mox_0,ltank,mf,lcgf,mp,ms,lcgs,m,lf,df1,df2,mf_b,Is,Ir,lcgox,lcgp,lcg,Ifp,Ifr,Ib)

call Airspeed_Calc(Ceb,Ve,Vw,Cs,rho,Va_pre,Va,alpha,beta,Mach,Q,Va_syn)

if (Va_max < Va_syn) then
  Va_max = Va_syn
end if
if (Mach_max < Mach) then
  Mach_max = Mach
end if
if (max_Q < Q) then
  max_Q = Q
end if

call spline(Mach_base,Cd_base,n_base-1,Mach,Cd,1)

call Load_Calc(Q,Cd,S,CNa,alpha,beta,eof,Dx,Ny,Nz,thrust,F)

call Moment_Calc(Q,S,M_fin,CNa,lcp,lcg,F,Clp,Cmq,Cnr,d,l,Va_syn,mp_dot,lcgp,Ma,Ka,Kj)

if (Flight_Status <= 1) then
  omega = 0.0d0
else
  call ODE_Solve(Ib,Ma,Ka,Kj,omega,omega_pre,Ceb,Cbe,quat,quat_pre,dt)
end if

call Quat2Euler(Ceb,theta,psi,fai)

call Acceleration_Integral(F,Cbe,m,g,acc_pre,acc_syn,Ve_pre,Ve_syn,acc,Ve,dt)

if (Flight_Status == 0) then
  Va = 0.0d0 ; Va_pre = 0.0d0
end if

!-----------Parachute Descent----------
500 if (code == 1) then

  if (H_sepa >= 0.0d0 .and. Position(3) <= H_sepa) then !TSSS ON and Under Altitude
    if (nose%m > 0.0d0) then !Release ON
      if (t >= t_sepa2 + t_release) then
        CdS = CdS2
        nose%CdS = CdS1
        m = m_top - nose%m
      end if
    else !Release OFF
      CdS = CdS2 + CdS1
    end if
    
  else !TSSS OFF / Over Altitude on TSSS Altitude
    
    CdS = CdS1
    
    if (nose%m > 0.0d0) then !Release ON Override
      nose%CdS = CdS1
      nose%Position = Position ; nose%Position_pre = Position_pre
      nose%Ve = Ve ; nose%Va_syn = Va_syn
    end if
    t_sepa2 = t
  end if  

  call Parachute_Aerodynamics(rho,CdS,m,g,Vw,acc,Ve,dt)
  
  if (nose%m > 0.0d0) then
    call Parachute_Aerodynamics(rho,nose%CdS,nose%m,g,Vw,nose%acc,nose%Ve,dt)
  end if
  
end if
!--------------------------------------

call Velocity_Integral(Ve,Ve_pre,Position_pre,Position,dt)
if (nose%m > 0.0d0) then
  call Velocity_Integral(nose%Ve,nose%Ve_pre,nose%Position_pre,nose%Position,dt)
end if

!-----------------------------------
!-         Flight State Check
!-----------------------------------

select case (Flight_Status)
  case (0)
    if (Position(3) > Position_pre(3)) then
      Flight_Status = 1
    else
      Position(3) = Position_pre(3)
    end if
  
  case (1)
    if ((Position(3) / sin(deg2rad(abs(theta)))) > LL) then
      V_lc = Ve_syn
      Flight_Status = 2
    end if
  
  case (2)
    if (Position(3) < Position_pre(3)) then
      Position_top = Position_pre
      Ve_top = Ve_pre
      Va_top = sqrt((Va_pre(1)*Va_pre(1)) + (Va_pre(2)*Va_pre(2)) + (Va_pre(3)*Va_pre(3)))
      m_top = m ; t_top = t - dt
      Flight_Status = 3
    end if
  
  case (3)
   if (Position(3) <= 0.0d0) then
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
!write (900,*) t,',',Mach,',',Cd!,',',Position(3)!,',',mf,',',Ipp,',',ms,',',m,',',lcgp,',',lcgs!,',',Kj(3)


!**********************************************************************
!                      Output Result Log
!**********************************************************************

if (code == 0) then !------ こ　こ　の　codeで　弾　道　/　開　傘　を　変　更 --------
  
  if (time_sw == 1) write (250,*) t
  if (position_sw == 1) write (251,*) Position(1),',',Position(2),',',Position(3)
  if (Attitude_sw == 1) write (252,*) theta,',',psi,',',fai,',',rad2deg(alpha),',',rad2deg(beta)
  if (Va_sw == 1) write (253,*) Va(1),',',Va(2),',',Va(3),',',Va_syn,',',Mach
  if (Flight_Status <= 2) then
    if (acc_sw == 1) write (254,*) acc(1),',',acc(2),',',acc(3),',',acc_syn
  end if
  
else !------ code == 1 -------

  if (acc_sw == 1) write (254,*) acc(1),',',acc(2),',',acc(3),',',acc_syn

end if

!**********************************************************************
!               Output Result Landing Range & Table
!**********************************************************************

if (Flight_Status == 4) then
  if (code == 0) then
    range = sqrt(Position(1)**2 + Position(2)**2) ; compass = atan(Position(2)/Position(1))*180.0d0/PI
    if (Landing_Range_table_sw == 1) then
      write (200,'(f0.3,a,f0.3,a,f0.3,a,f0.3,a,f0.3,a,f0.3,a,f0.3,a,$)') Position_top(3),',',Position(1)&
        ,',',Position(2),',',range,',',compass,',',Position_top(1),',',Position_top(2),','
      write (210,'(f0.3,a,f0.3,a,$)') Position(1),',',Position(2),','
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
    
    range = sqrt(Position(1)**2 + Position(2)**2) ; compass = atan(Position(2)/Position(1))*180.0d0/PI
    if (Landing_Range_table_sw == 1) then
      write (200,*) Position(1),',',Position(2),',',range,',',compass,',',t_top,',',Va_top
      write (210,*) Position(1),',',Position(2),',',nose%Position(1),',',nose%Position(2)
    end if
    if (t_table_sw == 1) write (208,'(f0.3,a,$)') t,','
  end if

  exit !********** Main Calculation Loop exit ************
  
end if

t = t + dt
end do
!************** End of Main Calculate *****************

end do
!************** End of Open of parachute? *****************
close(62)
print '(f10.2,f10.2,f10.2,f10.2,f10.2)',Vw_syn,Vw_angle,t_top,Position_top(3),t

Vw_angle = Vw_angle+Vw_angle_delta
if (Vw_angle > Vw_angle_max .or. Vw_angle >= 360.0d0) exit

end do
!************** End of Wind_Angle *****************


!------------------------------------
!-         Output Adjustment
!------------------------------------

if (Landing_Range_table_sw == 1) then
  write (200,*) ; write (200,*) ; write (200,*) ; write (200,*) ; write (200,*) ; write (200,*) ; 
end if
if (Vlc_table_sw == 1) write (201,*)
if (Va_max_table_sw == 1) then
  write (202,*) ; write (203,*) ; write (204,*)
end if
if (Va_top_table_sw == 1) write (205,*)
if (t_table_sw == 1) then
  write (206,*) ; write (207,*) ; write (208,*)
end if
if (Z_max_table_sw == 1) write (209,*)

Vw_syn = Vw_syn + Vw_delta
if (Vw_syn > Vw_max) exit

end do
!************* End of Wind_Velocity ****************


call cpu_time(t2)

print *, "CPU_Time:",(t2-t1)*1000.0d0," msec"

!**********************************************************************
!                       Output File Close
!**********************************************************************

if (Landing_Range_table_sw == 1) then
  close(200)
  close(210)
end if
if (Vlc_table_sw == 1) close(201)
if (Va_max_table_sw == 1) then
  close(202) ; close(203) ; close(204)
end if
if (Va_top_table_sw == 1) close(205)
if (t_table_sw == 1) then
  close(206) ; close(207) ; close(208)
end if
if (Z_max_table_sw == 1) close(209)
if (time_sw == 1) close(250)
if (position_sw == 1) close(251)
if (Attitude_sw == 1) close(252)
if (Va_sw == 1) close(253)
if (acc_sw == 1) close(254)


end program Simlation