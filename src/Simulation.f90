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

program Simulation
use standard_collection
use Rocket_Class
use Rocket_Dynamics
use atmosphere1976 
use environment
implicit none


!**********************************************************************

call cpu_time(t1)

!------------------------------------
!-            Switch
!------------------------------------

open (101,file='switch.dat',status='old')
read (101,*) LogTable_sw
read (101,*) Vw_min ; read (101,*) Vw_max ; read (101,*) Vw_delta
read (101,*) Vw_angle_min ; read (101,*) Vw_angle_max ; read (101,*) Vw_angle_delta
read (101,*) Decent_sw
close(101)

if (LogTable_sw == 0) then
! Log出力時は強制単一条件
  Vw_max = Vw_min
  Vw_angle_max = Vw_angle_min
  if (Decent_sw == 0) then
    open (251,file='OutputLog/Position_log.dat',status="replace",form="unformatted")
    open (252,file='OutputLog/Attitude_log.dat',status="replace",form="unformatted")
    open (253,file='OutputLog/Velocity_log.dat',status="replace",form="unformatted")
    open (254,file='OutputLog/Acceleration_log.dat',status="replace",form="unformatted")
    open (255,file='OutputLog/Force_log.dat',status="replace",form="unformatted")
    open (256,file='OutputLog/Structure_log.dat',status="replace",form="unformatted")
    open (257,file='OutputLog/Atmosphere_log.dat',status="replace",form="unformatted")
  else if (Decent_sw == 1) then
    open (251,file='OutputLog/Position_log.dat',status="replace",form="unformatted")
    open (253,file='OutputLog/Velocity_log.dat',status="replace",form="unformatted")
    open (257,file='OutputLog/Atmosphere_log.dat',status="replace",form="unformatted")
  end if
else
  LandingRange = 0.0d0
  AltitudeTable = 0.0d0
  MachTable = 0.0d0
  VaMaxTable = 0.0d0
  VaApogeeTable = 0.0d0
  TimeApogeeTable = 0.0d0
  TimeHardTable = 0.0d0
  TimeDecentTable = 0.0d0
  TimeTSSSTable = 0.0d0
end if

!**********************************************************************
!                Data Input(Read) & Pre Setting
!**********************************************************************
!------------------------------------
!-             Rocket
!------------------------------------
open (100,file='rocket_param.dat',status='old')
read (100,*) Rocket%l
read (100,*) Rocket%d
read (100,*) Rocket%ms
read (100,*) Rocket%lcgs
read (100,*) Rocket%lcgox_0
read (100,*) Rocket%lcgf
read (100,*) Rocket%ltank
read (100,*) Rocket%Is
read (100,*) Rocket%Ir
read (100,*) Rocket%lcp
read (100,*) Rocket%Clp
read (100,*) Rocket%Cmq
read (100,*) Rocket%CNa
read (100,*) Rocket%M_fin
read (100,*) Rocket%CdS1
read (100,*) Rocket%CdS2
read (100,*) freq
read (100,*) Rocket%de
read (100,*) Rocket%Isp
read (100,*) Rocket%It
read (100,*) Rocket%mox_0
read (100,*) Rocket%mf_b
read (100,*) Rocket%mf_a
read (100,*) Rocket%lf
read (100,*) Rocket%df1
read (100,*) Rocket%df2
read (100,*) Rocket%mf_dot
read (100,*) Pa_0
read (100,*) Ta_0
read (100,*) g0
read (100,*) Hw
read (100,*) Wh
read (100,*) Rocket%T_sepa
read (100,*) Rocket%H_sepa
read (100,*) Rocket%theta_0
read (100,*) Rocket%psi_0
read (100,*) LL
close(100)

call sign_check(Rocket%Clp,-1)
call sign_check(Rocket%Cmq,-1)
call sign_check(Rocket%theta_0,-1)
!-よくあるエラー回避


!------------------------------------
!-          Pre Setting
!- 単位変換や以降変更のないものの計算
!------------------------------------
dt = 1.0d0 / freq
Rocket%Cnr = Rocket%Cmq ! Pitch == Yaw(軸対称のロケット)

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

call setMachCd() ! Mach-Cdスプラインのセット

call setThrust(Rocket) ! 推力のRead

!****************************************************************************************************
!                                       Calculate Process
!****************************************************************************************************
print *, 'Calculation Start!!'
print '(a10,a10,a10,a10,a10)', 'Vw','Vw_angle','Top','Altitude','Landing'

!**********************************************************************
!                        Wind Velocity Loop
!**********************************************************************
index_wind = 1
Vw_abs = Vw_min
do while (Vw_abs <= Vw_max)
  !**********************************************************************
  !                         Wind Angle Loop
  !**********************************************************************
  index_wangle = 1
  Vw_angle = Vw_angle_min
  do while (Vw_angle <= Vw_angle_max .and. Vw_angle < 360.0d0)
    !--------------------------------------
    !-            Initialize
    !--------------------------------------
    Flight_Status = 0
    call Initialize(Rocket)

    !**********************************************************************
    !
    !                     Main Calculation (Ballistic)
    !
    !**********************************************************************
    i = 1
    do while (Rocket%Position(3) > 0.0d0)
      call VelocityOfWind_Vecter(Vw_abs,Vw_angle,Rocket%Position(3),Hw,Wh,Vw)
      call Atmosphere_Setting(Ta_0,Pa_0,rho_0,g0,Rocket%Position(3),g,Cs,Ta,Pa,rho)
      if (Flight_Status > 0) then
        call Airspeed_Calc(Rocket,Vw,Cs,rho)
      end if
      if (i <= index_burn) then
        call Mass_Calc(Rocket,g0,dt)
        call MI_Calc(Rocket)
      end if
      call Load_Calc(Rocket,Pa,Pa_0)
      call Moment_Calc(Rocket)
      call Acceleration_Integral(Rocket,g,dt)
      call Velocity_Integral(Rocket,dt)
      if (Flight_Status <= 1) then
        Rocket%omega = 0.0d0
      else
        call ODE_Solve(Rocket,dt)
      end if
      call Quat2Euler(Rocket%Ceb,Rocket%theta,Rocket%psi,Rocket%fai)

      !-----------------------------------
      !-        Flight State Check
      !- Ballistic
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
            Acc_lc = (Rocket%acce_abs - g0) / g0
            Flight_Status = 2
          end if
        case (2)
          if (Rocket%Mach > Mach_max) then
            Mach_max = Rocket%Mach
            Va_max = Rocket%Va_abs
          end if
          if (Rocket%Position(3) < Rocket%Position_pre(3)) then
            Position_top = Rocket%Position_pre
            Ve_top = Rocket%Ve_pre
            Va_top = Abs_Vector(Rocket%Va_pre)
            m_top = Rocket%m ; t_top = t - dt
            Flight_Status = 3
          end if
      end select
      
      !--------------------------------------
      !-            Log Output
      !--------------------------------------
      if (LogTable_sw == 0 .and. Decent_sw == 0) then
        write (251) t,Rocket%Position(1),Rocket%Position(2),Rocket%Position(3)
        write (252) t,Rocket%theta,Rocket%psi,Rocket%fai,rad2deg(Rocket%alpha),rad2deg(Rocket%beta)
        write (253) t,Rocket%Va(1),Rocket%Va(2),Rocket%Va(3),Rocket%Va_abs,Rocket%Mach,Rocket%Ve(1),Rocket%Ve(2),Rocket%Ve(3)
        write (254) t,Rocket%acce(1),Rocket%acce(2),Rocket%acce(3)
        write (255) t,Rocket%Dx,Rocket%F(1),Rocket%F(2),Rocket%F(3)
        write (256) t,Rocket%mf,Rocket%mox,Rocket%mp,Rocket%m,Rocket%lcgox,Rocket%lcgp,Rocket%lcg
        write (257) t,Pa,Ta,rho,g,Cs,Vw(1),Vw(2)
      end if
      !--------------------------------------
      
      t = t + dt
      i = i + 1
    end do
    if (LogTable_sw == 1) then
      LandingRange((index_wind-1)*8+index_wangle,1) = Rocket%Position(1)
      LandingRange((index_wind-1)*8+index_wangle,2) = Rocket%Position(2)
      TimeHardTable(index_wind,index_wangle) = t
    end if
    !************** End of Main Calculate *****************

    
    !**********************************************************************
    !
    !                Main Calculation (Parachute Decent)
    ! 
    !**********************************************************************
    t = t_top
    Rocket%Position = Position_top
    Rocket%Ve = Ve_top
    Rocket%Va_abs = Va_top
    Rocket%m = m_top
    Flight_Status = 3

    do while (Rocket%Position(3) > 0.0d0)
      call VelocityOfWind_Vecter(Vw_abs,Vw_angle,Rocket%Position(3),Hw,Wh,Vw)
      call Atmosphere_Setting(Ta_0,Pa_0,rho_0,g0,Rocket%Position(3),g,Cs,Ta,Pa,rho)
      !-----------Parachute Descent----------
      if (Rocket%H_sepa > 0.0d0 .and. Rocket%Position(3) <= Rocket%H_sepa) then !TSSS ON and Under Altitude  
        Rocket%CdS = Rocket%CdS2 + Rocket%CdS1
      else !TSSS OFF / Over Altitude on TSSS Altitude
        Rocket%CdS = Rocket%CdS1
        t_sepa2 = t
      end if  
      call Parachute_Aerodynamics(Rocket,rho,g,Vw,dt)  
      !--------------------------------------
      call Velocity_Integral(Rocket,dt)
      
      !--------------------------------------
      !-            Log Output
      !--------------------------------------
      if (LogTable_sw == 0 .and. Decent_sw == 1) then
        write (251) t,Rocket%Position(1),Rocket%Position(2),Rocket%Position(3)
        write (253) t,Rocket%Va(1),Rocket%Va(2),Rocket%Va(3),Rocket%Va_abs,Rocket%Mach,Rocket%Ve(1),Rocket%Ve(2),Rocket%Ve(3)
        write (257) t,Pa,Ta,rho,g,Cs,Vw(1),Vw(2)
      end if
      !--------------------------------------
      
      t = t + dt
    end do
    
    if (LogTable_sw == 1) then
      LandingRange((index_wind-1)*8+index_wangle,3) = Rocket%Position(1)
      LandingRange((index_wind-1)*8+index_wangle,4) = Rocket%Position(2)
      AltitudeTable(index_wind,index_wangle) = Position_top(3)
      MachTable(index_wind,index_wangle) = Mach_max
      VaMaxTable(index_wind,index_wangle) = Va_max
      VaApogeeTable(index_wind,index_wangle) = Va_top
      TimeApogeeTable(index_wind,index_wangle) = t_top
      TimeDecentTable(index_wind,index_wangle) = t
      TimeTSSSTable(index_wind,index_wangle) = t_sepa2
    end if
    !************** End of Main Calculate *****************

    print '(f10.2,f10.2,f10.2,f10.2,f10.2)',Vw_abs,Vw_angle,t_top,Position_top(3),t
    
    Vw_angle = Vw_angle + Vw_angle_delta
    index_wangle = index_wangle + 1
  end do
  !************** End of Wind_Angle *****************
  
  Vw_abs = Vw_abs + Vw_delta
  index_wind = index_wind + 1
end do
!************* End of Wind_Velocity ****************

!--------------------------------------
!-            Log Output
!--------------------------------------

if (LogTable_sw == 0) then
  if (Decent_sw == 0) then
    close(251) ; close(252) ; close(253) ; close(254) ; close(255) ; close(256) ; close(257)
  else if (Decent_sw == 1) then
    close(251) ; close(253) ; close(257)
  end if
else
  open (200,file='OutputTable/Landing_Range_v2.dat',status="replace",form="unformatted")
  open (201,file='OutputTable/Max_Altitude_table.dat',status="replace",form="unformatted")
  open (202,file='OutputTable/Mach_max_table.dat',status="replace",form="unformatted")
  open (203,file='OutputTable/Va_max_table.dat',status="replace",form="unformatted")
  open (204,file='OutputTable/Va_Top_table.dat',status="replace",form="unformatted")
  open (205,file='OutputTable/time_top_table.dat',status="replace",form="unformatted")
  open (206,file='OutputTable/time_Hard_Landing_table.dat',status="replace",form="unformatted")
  open (207,file='OutputTable/time_Para_Landing_table.dat',status="replace",form="unformatted")
  open (208,file='OutputTable/time_Second_Para_table.dat',status="replace",form="unformatted")

  write (200) LandingRange
  write (201) AltitudeTable
  write (202) MachTable
  write (203) VaMaxTable
  write (204) VaApogeeTable
  write (205) TimeApogeeTable
  write (206) TimeHardTable
  write (207) TimeDecentTable
  write (208) TimeTSSSTable
  
  close(200) ; close(201) ; close(202) ; close(203) ; close(204) ; close(205) ; close(206) ; close(207) ; close(208)
end if


print '(a,f5.2,a,f5.2,a)',"LaunchClear : ",V_lc,"[m/s] , ",Acc_lc,"[G]"
call cpu_time(t2)
print *, "CPU_Time:",(t2-t1)*1000.0d0," msec"
read *

end program Simulation