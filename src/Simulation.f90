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
use Rocket_Dynamics
use atmosphere1976 
use environment
implicit none


!**********************************************************************

call system_clock(t1)

!**********************************************************************
!                Data Input(Read) & Pre Setting
!**********************************************************************
print *, "Parameter Reading..."
call read_input
call set_InitialCondition

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
    !**********************************************************************
    !
    !                     Main Calculation (Ballistic)
    !
    !**********************************************************************
    call Initialize
    i = 1
    do while (Position(3) > 0.0d0)
      call VelocityOfWind_Vecter
      call Atmosphere_Setting
      if (Flight_Status > 0) then
        call calc_Airspeed
      end if
      call calc_Mass
      call calc_CG_IM
      call calc_Force
      call calc_Moment
      call Acc2Position
      if (Flight_Status <= 1) then
        omega = 0.0d0
      else
        call ODE_Solve
      end if
      call Quat2Euler

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
          if (Position(3) > Position_pre(3)) then
            Flight_Status = 1
          else
            t = 0.0d0
            if (Ve(3) < 0.0d0) Ve = 0.0d0
            Position(3) = Position_pre(3)
          end if
        case (1)
          if ((Position(3) / sin(deg2rad(abs(theta_0)))) > LL) then
            V_lc = Ve_abs
            Acc_lc = (acce_abs - g0) / g0
            Flight_Status = 2
          end if
        case (2)
          if (Mach > Mach_max) then
            Mach_max = Mach
            Va_max = Va_abs
          end if
          if (Position(3) < Position_pre(3)) then
            Position_top = Position_pre
            Ve_top = Ve_pre
            Va_top = abs_3axis(Va_pre)
            m_top = m ; t_top = t - dt
            Flight_Status = 3
          end if
      end select
      
      if (LogTable_sw == 0) call Output_Log(fileNum_output)
      
      t = t + dt
      i = i + 1
    end do
    
    if (LogTable_sw == 1) then
      LandingRange((index_wind-1)*8+index_wangle,1) = Position(1)
      LandingRange((index_wind-1)*8+index_wangle,2) = Position(2)
      TimeHardTable(index_wind,index_wangle) = t
      AltitudeTable(index_wind,index_wangle) = Position_top(3)
      MachTable(index_wind,index_wangle) = Mach_max
      VaMaxTable(index_wind,index_wangle) = Va_max
      VaApogeeTable(index_wind,index_wangle) = Va_top
      TimeApogeeTable(index_wind,index_wangle) = t_top
      VlcTable(index_wind,index_wangle) = V_lc
      AcclcTable(index_wind,index_wangle) = Acc_lc
    end if
    !************** End of Main Calculate *****************

    
    !**********************************************************************
    !
    !                Main Calculation (Parachute Decent)
    ! 
    !**********************************************************************
    t = t_top
    Position = Position_top
    Ve = Ve_top
    Va_abs = Va_top
    m = m_top
    Flight_Status = 3

    do while (Position(3) > 0.0d0)
      call VelocityOfWind_Vecter
      call Atmosphere_Setting
      !-----------Parachute Descent----------
      if (H_sepa > 0.0d0 .and. Position(3) <= H_sepa) then !TSSS ON and Under Altitude  
        CdS = CdS2 + CdS1
      else !TSSS OFF / Over Altitude on TSSS Altitude
        CdS = CdS1
        t_sepa2 = t
      end if  
      call Parachute_Aerodynamics 
      !--------------------------------------
      
      if (LogTable_sw == 0) call Output_Log(fileNum_output+1)
      
      t = t + dt
    end do
    
    if (LogTable_sw == 1) then
      LandingRange((index_wind-1)*8+index_wangle,3) = Position(1)
      LandingRange((index_wind-1)*8+index_wangle,4) = Position(2)
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

if (LogTable_sw == 0) then
  close(fileNum_output) ; close(fileNum_output+1)
else
  call Output_Table
end if

print '(a,f5.2,a,f5.2,a)',"LaunchClear : ",V_lc,"[m/s] , ",Acc_lc,"[G]"
print *, ""
call system_clock(t2,t2_rate,t2_max)
if (t2 < t1) then
  t2_diff = (t2_max - t1) + t2 + 1
else
  t2_diff = t2 - t1
end if
print *, "Calculation_Time:",t2_diff / dble(t2_rate)," sec"


contains

!**********************************************************************
!                       Personal Subroutine
!**********************************************************************
subroutine read_input
  real(8) :: Mach_inp,Cd_inp
  real(8) :: thrust_inp

  !------------------------------------
  !-             Rocket
  !------------------------------------
  open (fileNum_input,file='rocket_param.inp',status='old')
  read (fileNum_input,*) l
  read (fileNum_input,*) d
  read (fileNum_input,*) ms
  read (fileNum_input,*) lcgs
  read (fileNum_input,*) lcgox_0
  read (fileNum_input,*) lcgf
  read (fileNum_input,*) ltank
  read (fileNum_input,*) Is
  read (fileNum_input,*) Ir
  read (fileNum_input,*) lcp
  read (fileNum_input,*) Clp
  read (fileNum_input,*) Cmq
  read (fileNum_input,*) CNa
  read (fileNum_input,*) M_fin
  read (fileNum_input,*) CdS1
  read (fileNum_input,*) CdS2
  read (fileNum_input,*) freq
  read (fileNum_input,*) de
  read (fileNum_input,*) Isp
  read (fileNum_input,*) mox_0
  read (fileNum_input,*) mf_b
  read (fileNum_input,*) mf_a
  read (fileNum_input,*) lf
  read (fileNum_input,*) df1
  read (fileNum_input,*) df2
  read (fileNum_input,*) mf_dot
  read (fileNum_input,*) Pa_0
  read (fileNum_input,*) Ta_0
  read (fileNum_input,*) g0
  read (fileNum_input,*) Hw
  read (fileNum_input,*) Wh
  read (fileNum_input,*) T_sepa
  read (fileNum_input,*) H_sepa
  read (fileNum_input,*) theta_0
  read (fileNum_input,*) psi_0
  read (fileNum_input,*) LL
  close(fileNum_input)

  !- よくある符号エラー回避
  Clp = Clp * (0.5d0 - 0.5d0 * sign(1.0d0,Clp)) &
    & - Clp * (0.5d0 + 0.5d0 * sign(1.0d0,Clp))
  Cmq = Cmq * (0.5d0 - 0.5d0 * sign(1.0d0,Cmq)) &
    & - Cmq * (0.5d0 + 0.5d0 * sign(1.0d0,Cmq))
  theta_0 = theta_0 * (0.5d0 - 0.5d0 * sign(1.0d0,theta_0)) &
        & - theta_0 * (0.5d0 + 0.5d0 * sign(1.0d0,theta_0))

  !------------------------------------
  !-           Thrust Read
  !-推力履歴は必ず作動開始からのデータであること
  !-質量減少はthrustがあれば発生するようになっている
  !------------------------------------
  open (fileNum_input+1,file='thrust.inp',status='old')
  eof = 0 ; index_burn = 0
  do while(eof >= 0)
    index_burn = index_burn + 1
    read (fileNum_input+1,*,iostat = eof) thrust_inp
    call append(thrust,thrust_inp)
  end do
  close(fileNum_input+1)

  !------------------------------------
  !-            Switch
  !------------------------------------

  open (fileNum_input+2,file='switch.inp',status='old')
  read (fileNum_input+2,*) LogTable_sw
  read (fileNum_input+2,*) Vw_min ; read (fileNum_input+2,*) Vw_max ; read (fileNum_input+2,*) Vw_delta
  read (fileNum_input+2,*) Vw_angle_min ; read (fileNum_input+2,*) Vw_angle_max ; read (fileNum_input+2,*) Vw_angle_delta
  close(fileNum_input+2)

  if (LogTable_sw == 0) then
  ! Log出力時は強制単一条件
    Vw_max = Vw_min
    Vw_angle_max = Vw_angle_min
  
    write(filename,'("OutputLog/","BallisticLog",".dat")')
    open (fileNum_output,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputLog/","DecentLog",".dat")')
    open (fileNum_output+1,file=filename,status="replace",form="unformatted")
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

  !------------------------------------
  !-           Mach-Cd Read
  !-使う前にsplineの最後の引数を0で呼び出し
  !-save属性でスプラインに必要な変数が保持される
  !-
  !------------------------------------
  open (fileNum_input+3,file='Mach_Cd.inp',status='old')
  read (fileNum_input+3,*) !- 1行目のスキップ
  eof = 0
  do while(eof >= 0)
    n_base = n_base + 1
    read (fileNum_input+3,*,iostat = eof) Mach_inp,Cd_inp
    call append(Mach_base,Mach_inp)
    call append(Cd_base,Cd_inp)
  end do
  n_base = n_base - 1
  call spline(Mach_base,Cd_base,n_base-1,Mach_inp,Cd_inp,0)
  close(fileNum_input+3)

end subroutine read_input

subroutine set_InitialCondition
  !------------------------------------
  !-          Pre Setting
  !- 単位変換や以降変更のないものの計算
  !------------------------------------
  dt = 1.0d0 / freq
  Cnr = Cmq ! Pitch == Yaw(軸対称のロケット)

  !- Diameter to Area
  S = 0.25 * d**2 * PI
  Ae = 0.25d0 * (de * 1.0d-3)**2 * PI

  !- from end to from nose
  lcgf = l - lcgf
  lcgox_0 = l - lcgox_0

  Pa_0 = Pa_0 * 1.0d3 !- kPa to Pa
  Ta_0 = Ta_0 + 273.15d0 !- degC to Kelvin
  rho_0 = Pa_0 / (R*Ta_0)

  !- initial value save
  lcgp_0 = (lcgox_0 * mox_0 + lcgf * mf_b) / (mox_0 + mf_b)
  lcg_0 = (((mox_0 + mf_b) * lcgp_0) + (ms * lcgs)) / (ms + (mf_b + mox_0))

end subroutine set_InitialCondition

subroutine Initialize
  real(8) :: length
    
  t = 0.0d0
  Ta = Ta_0 ; Pa = Pa_0 ; rho = rho_0
  mox = mox_0 ; mf = mf_b
  Va = 0.0d0 ; Va_pre = 0.0d0 ; Va_abs = 0.0d0
  Mach = 0.0d0 ; Mach_pre = 0.0d0 ; Q = 0.0d0
  acce = 0.0d0 ; acce_pre = 0.0d0 ; Ve = 0.0d0 ; Ve_pre = 0.0d0 ; Position = 0.0d0 ; Position_pre = 0.0d0
  omega = 0.0d0 ; omega_pre = 0.0d0
  fai = 0.0d0 ; theta = deg2rad(theta_0) ; psi = deg2rad(psi_0) ; alpha = 0.0d0 ; beta = 0.0d0

  call Euler2Quat
  call set_DCM

  length = (l - lcg_0) * cos(deg2rad(abs(theta_0)))
  if (psi_0 >= 0.0d0 .and. psi_0 < 90.0d0) then
    Position(1) = length * cos(deg2rad(psi_0))
    Position(2) = length * sin(deg2rad(psi_0))
  else if (psi_0 >= 90.0d0 .and. psi_0 < 270.0d0) then
    Position(1) = length * cos(deg2rad(180.0d0 - psi_0))
    Position(2) = length * sin(deg2rad(180.0d0 - psi_0))
  else if (psi_0>= 270.0d0 .and. psi_0< 360.0d0) then
    Position(1) = length * cos(deg2rad(360.0d0 - psi_0))
    Position(2) = length * sin(deg2rad(360.0d0 - psi_0))
  end if
  Position(3) = (l - lcg_0) * sin(deg2rad(abs(theta_0)))
  
  Position_top = 0.0d0
  Ve_top = 0.0d0
  t_top = 0.0d0 ; Va_top = 0.0d0 ; m_top = 0.0d0
  V_lc = 0.0d0 ; Va_max = 0.0d0 ; Mach_max = 0.0d0
  
  Flight_Status = 0

  
  
end subroutine Initialize


subroutine Output_Log(filenum)
  integer,intent(in) :: filenum

  write (fileNum) t,Pa,Ta,rho,g,Cs,Vw(1),Vw(2),mf,mox,mp,m,lcgox,lcgp,lcg,Dx,F(1),F(2),F(3),Acce(1),Acce(2),Acce(3), &
                & Va(1),Va(2),Va(3),Va_abs,Mach,Ve(1),Ve(2),Ve(3),theta,psi,fai,rad2deg(alpha),rad2deg(beta), &
                & Position(1),Position(2),Position(3)
  
end subroutine Output_Log

subroutine Output_Table
  if (LogTable_sw == 1) then !- 表出力ON
    write(filename,'("OutputTable/","Landing_Range_v2",".dat")')
    open (fileNum_output,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","Max_Altitude",".dat")')
    open (fileNum_output+1,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","Mach_max",".dat")')
    open (fileNum_output+2,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","Va_max",".dat")')
    open (fileNum_output+3,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","Va_Top",".dat")')
    open (fileNum_output+4,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","time_top",".dat")')
    open (fileNum_output+5,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","time_Hard_Landing",".dat")')
    open (fileNum_output+6,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","time_Para_Landing",".dat")')
    open (fileNum_output+7,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","time_Second_Para",".dat")')
    open (fileNum_output+8,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","LaunchClear_Velocity",".dat")')
    open (fileNum_output+9,file=filename,status="replace",form="unformatted")
    write(filename,'("OutputTable/","LaunchClear_Acceleration",".dat")')
    open (fileNum_output+10,file=filename,status="replace",form="unformatted")

    write (fileNum_output) LandingRange
    write (fileNum_output+1) AltitudeTable
    write (fileNum_output+2) MachTable
    write (fileNum_output+3) VaMaxTable
    write (fileNum_output+4) VaApogeeTable
    write (fileNum_output+5) TimeApogeeTable
    write (fileNum_output+6) TimeHardTable
    write (fileNum_output+7) TimeDecentTable
    write (fileNum_output+8) TimeTSSSTable
    write (fileNum_output+9) VlcTable
    write (fileNum_output+10) AcclcTable

    do j = 0,10
      close(fileNum_output+j)
    end do

  end if
end subroutine Output_Table

end program Simulation