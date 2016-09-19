module Rocket_Dynamics
use standard_collection
use Rocket_Class
implicit none

contains
!**********************************************************************
!                                 Body
!**********************************************************************
!------------------------------------
!-         Mass Calculate
!------------------------------------

subroutine Mass_Calc(Rocket,g0,dt)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: g0,dt

  Rocket%mp_dot = Rocket%thrust / (Rocket%Isp * g0)
  Rocket%mox_dot = Rocket%mp_dot - Rocket%mf_dot

  if (Rocket%mf > Rocket%mf_a) then
    Rocket%mf = Rocket%mf - Rocket%mf_dot * dt
  end if
  if (Rocket%mox > 0.0d0) then
    Rocket%mox = Rocket%mox - Rocket%mox_dot * dt
  end if

  Rocket%mp = Rocket%mf + Rocket%mox
  Rocket%m = Rocket%ms + Rocket%mp
  
end subroutine Mass_Calc

!------------------------------------
!-    Moment of Inertia Calculate
!------------------------------------

subroutine MI_Calc(Rocket)
  type(Rocket_Type),intent(inout) :: Rocket
  
  !--- Center of Gravity---
  Rocket%lcgox = Rocket%lcgox_0 - 0.5d0 * (1.0d0 - (Rocket%mox / Rocket%mox_0)) * Rocket%ltank
  if (Rocket%mox <= 0.0d0) Rocket%lcgox = 0.0d0
  Rocket%lcgp = (Rocket%mf * Rocket%lcgf + Rocket%mox * Rocket%lcgox) / Rocket%mp
  Rocket%lcg = ((Rocket%mp * Rocket%lcgp) + (Rocket%ms * Rocket%lcgs)) / Rocket%m
  
  !--- Moment of Inertia ---
  !-タンク内酸化剤,液体になるのでまた別の考えが必要
  !Ipp = (mox*((0.5*ltank - (l - lcgox))**2 / 12.0d0 + (lcgox - lcgp)**2) + mf*(lcgf - lcgp)**2) + mp*(lcg - lcgp)**2
  
  !-燃料
  Rocket%Ifp = Rocket%mf * Rocket%lf / 12.0d0
  Rocket%Ifr = Rocket%mf * (((Rocket%df1 + Rocket%df2) / 4.0d0)&
                + ((Rocket%df1 + Rocket%df2) / 4.0d0) * (1.0d0 - (Rocket%mf / Rocket%mf_b)))
  
  !-全機
  Rocket%Ib = Rocket%Ifp + Rocket%mf * (Rocket%lcg - Rocket%lcgf)**2 + Rocket%Is * ((Rocket%ms + Rocket%mox) / Rocket%ms)&
               + (Rocket%ms + Rocket%mox) * (Rocket%lcg - Rocket%lcgs)**2
  Rocket%Ib(1) = Rocket%Ir + Rocket%Ifr !-ロールのみ上書きで書いちゃう
  
end subroutine MI_Calc

!**********************************************************************
!                            Translation
!**********************************************************************
!------------------------------------
!-         Load Calculate
!------------------------------------

subroutine Load_Calc(Rocket,Pa,Pa_0,eof)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: Pa,Pa_0
  integer :: eof
  
  Rocket%Dx = Rocket%Q * Rocket%Cd * Rocket%S
  Rocket%Ny = Rocket%Q * Rocket%CNa * Rocket%S * Rocket%beta
  Rocket%Nz = Rocket%Q * Rocket%CNa * Rocket%S * Rocket%alpha
  
  if (eof >= 0) then
    Rocket%thrust = Rocket%thrust + (Pa_0 - Pa) * Rocket%Ae
  end if

  Rocket%F(1) = Rocket%thrust - Rocket%Dx
  Rocket%F(2) = Rocket%Ny
  Rocket%F(3) = -Rocket%Nz
  
end subroutine Load_Calc


!------------------------------------
!-        Airspeed Calculate
!------------------------------------

subroutine Airspeed_Calc(Rocket,Vw,Cs,rho)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: Vw(:),Cs,rho
  
  Rocket%Va_pre = Rocket%Va
  Rocket%Va = matmul(Rocket%Ceb,(Rocket%Ve - Vw))
  Rocket%Va_abs = Abs_Vector(Rocket%Va)
  
  Rocket%alpha = atan(Rocket%Va(3) / Rocket%Va(1))
  Rocket%beta = asin(-Rocket%Va(2) / Rocket%Va_abs)
  
  Rocket%Mach = Rocket%Va_abs / Cs
  Rocket%Q = 0.5d0 * rho * Rocket%Va_abs**2
  
end subroutine Airspeed_Calc


!------------------------------------
!-         Position Integral
!------------------------------------

subroutine Acceleration_Integral(Rocket,g,dt)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: g,dt
  
  Rocket%acce_pre = Rocket%acce
  Rocket%acce = matmul(Rocket%Cbe,Rocket%F)
  Rocket%acce = Rocket%acce / Rocket%m
  Rocket%acce(3) = Rocket%acce(3) - g
  Rocket%acce_abs = Abs_Vector(Rocket%acce)
  
  Rocket%Ve_pre = Rocket%Ve
  Rocket%Ve = Rocket%Ve_pre + Integral_3D(Rocket%acce,Rocket%acce_pre,dt)
  
  Rocket%Ve_abs = Abs_Vector(Rocket%Ve)
  
end subroutine Acceleration_Integral


!------------------------------------
!-        Velocity Integral
!------------------------------------

subroutine Velocity_Integral(Rocket,dt)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: dt
  
  Rocket%Position_pre = Rocket%Position
  Rocket%Position = Rocket%position_pre + Integral_3D(Rocket%Ve,Rocket%Ve_pre,dt)
  
end subroutine Velocity_Integral


!------------------------------------
!-      Parachute Aerodynamics
!------------------------------------

subroutine Parachute_Aerodynamics(Rocket,rho,g,Vw,dt)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: rho,g,Vw(:),dt
  
  Rocket%Ve_pre = Rocket%Ve
  Rocket%acce(3) = (0.5d0 * rho * Rocket%Ve(3)**2 * Rocket%CdS - Rocket%m * g) / Rocket%m
  
  Rocket%Ve(1) = Vw(1) !横方向は等風速
  Rocket%Ve(2) = Vw(2)
  Rocket%Ve(3) = Rocket%Ve(3) + Rocket%acce(3) * dt
  
end subroutine Parachute_Aerodynamics

!**********************************************************************
!                              Rotation
!**********************************************************************
!------------------------------------
!-         Moment Calculate
!------------------------------------

subroutine Moment_Calc(Rocket)
  type(Rocket_Type),intent(inout) :: Rocket
  
  !Moment of Aerodynamic これの符号は軸の回転方向に対応
  Rocket%Ma(1) = Rocket%Q * Rocket%S * Rocket%CNa * 0.07d0 * deg2rad(Rocket%M_fin) ! 0.07[m]はフィンの圧力中心位置(径方向)。真面目に計算するならspanを諸元で呼ぶこと
  Rocket%Ma(2) = (Rocket%lcp - Rocket%lcg) * Rocket%F(3)
  Rocket%Ma(3) = -(Rocket%lcp - Rocket%lcg) * Rocket%F(2)
    
  !Moment of Aerodynamic Damping これの符号は角速度の方向と対応
  !omegaはあとの運動方程式内で掛ける
  Rocket%Ka(1) = Rocket%Q * Rocket%Clp * Rocket%S * Rocket%d**2 / (2.0d0 * Rocket%Va_abs)
  Rocket%Ka(2) = Rocket%Q * Rocket%Cmq * Rocket%S * Rocket%l**2 / (2.0d0 * Rocket%Va_abs)
  Rocket%Ka(3) = Rocket%Q * Rocket%Cnr * Rocket%S * Rocket%l**2 / (2.0d0 * Rocket%Va_abs)
  
  !Jet Dumping Moment これの符号は角速度の方向と対応
  
  !For Rossr theory
  !Don't use
  !k2_pre = k2
  !k2 = (lcg - lcgp)**2
  !k2 = (k2 - k2_pre) / dt
  
  Rocket%Kj(1) = 0.0d0 !-(Ipr_dot - mp_dot*0.5d0*(0.5d0*dno)**2) For Nozzle radius and Mass flow rate
  !Ignore Moment of inartia rate
  Rocket%Kj(2) = -(Rocket%mp_dot * ((Rocket%lcg - Rocket%lcgp)**2 - (Rocket%l - Rocket%lcgp)**2))
  Rocket%Kj(3) = -(Rocket%mp_dot * ((Rocket%lcg - Rocket%lcgp)**2 - (Rocket%l - Rocket%lcgp)**2))
  
end subroutine Moment_Calc
  
!------------------------------------
!-   Rotation Momentum Equation
!------------------------------------

function RotEq(Rocket,RK_Parameter)
  real(8) :: RotEq(3)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: RK_Parameter(3)
  
  RotEq(1) = ((Rocket%Ib(2) - Rocket%Ib(3)) * Rocket%omega(2) * Rocket%omega(3)&
             + (Rocket%Ma(1) + (Rocket%Ka(1) + Rocket%Kj(1)) * (Rocket%omega(1) + RK_Parameter(1)))) / Rocket%Ib(1)
  
  RotEq(2) = ((Rocket%Ib(3) - Rocket%Ib(1)) * Rocket%omega(1) * Rocket%omega(3)&
             + (Rocket%Ma(2) + (Rocket%Ka(2) + Rocket%Kj(2)) * (Rocket%omega(2) + RK_Parameter(2)))) / Rocket%Ib(2)
  
  RotEq(3) = ((Rocket%Ib(1) - Rocket%Ib(2)) * Rocket%omega(1) * Rocket%omega(2)&
             + (Rocket%Ma(3) + (Rocket%Ka(3) + Rocket%Kj(3)) * (Rocket%omega(3) + RK_Parameter(3)))) / Rocket%Ib(3)
  
end function RotEq

!------------------------------------
!- Angler Speed - ODE Solve for RK4
!------------------------------------

subroutine ODE_Solve(Rocket,dt)
  type(Rocket_Type),intent(inout) :: Rocket
  real(8),intent(in) :: dt
  real(8) :: d_omega(3,4),RK_Parameter(3),norm
  integer :: k

  !iteration1
  RK_Parameter = 0.0d0
  d_omega(:,1) = RotEq(Rocket,RK_Parameter)
  !iteration2
  do k = 1,3
    RK_Parameter(k) = 0.5d0 * dt * d_omega(k,1)
  end do
  d_omega(:,2) = RotEq(Rocket,RK_Parameter)
  !iteration3
  do k = 1,3
    RK_Parameter(k) = 0.5d0 * dt * d_omega(k,2)
  end do
  d_omega(:,3) = RotEq(Rocket,RK_Parameter)
  !iteration4
  do k = 1,3
    RK_Parameter(k) = dt * d_omega(k,3)
  end do
  d_omega(:,4) = RotEq(Rocket,RK_Parameter)
  
  
  Rocket%omega_pre = Rocket%omega
  Rocket%omega(1) = Rocket%omega_pre(1) + ((d_omega(1,1) + 2.0d0 * (d_omega(1,2) + d_omega(1,3)) + d_omega(1,4)) / 6.0d0) * dt
  Rocket%omega(2) = Rocket%omega_pre(2) + ((d_omega(2,1) + 2.0d0 * (d_omega(2,2) + d_omega(2,3)) + d_omega(2,4)) / 6.0d0) * dt
  Rocket%omega(3) = Rocket%omega_pre(3) + ((d_omega(3,1) + 2.0d0 * (d_omega(3,2) + d_omega(3,3)) + d_omega(3,4)) / 6.0d0) * dt
  
  Rocket%quat_pre = Rocket%quat
  Rocket%quat(1) = Rocket%quat_pre(1) + 0.5d0 * (Rocket%quat_pre(2) * Rocket%omega(3)&
                   - Rocket%quat_pre(3) * Rocket%omega(2) + Rocket%quat_pre(4) * Rocket%omega(1)) * dt
  Rocket%quat(2) = Rocket%quat_pre(2) + 0.5d0 * (-Rocket%quat_pre(1) * Rocket%omega(3)&
                   + Rocket%quat_pre(3) * Rocket%omega(1) + Rocket%quat_pre(4) * Rocket%omega(2)) * dt
  Rocket%quat(3) = Rocket%quat_pre(3) + 0.5d0 * (Rocket%quat_pre(1) * Rocket%omega(2)&
                   - Rocket%quat_pre(2) * Rocket%omega(1) + Rocket%quat_pre(4) * Rocket%omega(3)) * dt
  Rocket%quat(4) = Rocket%quat_pre(4) + 0.5d0 * (-Rocket%quat_pre(1) * Rocket%omega(1)&
                   - Rocket%quat_pre(2) * Rocket%omega(2) - Rocket%quat_pre(3) * Rocket%omega(3)) * dt

  norm = sqrt(Rocket%quat(1)**2 + Rocket%quat(2)**2 + Rocket%quat(3)**2 + Rocket%quat(4)**2)
  Rocket%quat = Rocket%quat / norm
  
  call DCM_Set(Rocket%Cbe,Rocket%Ceb,Rocket%quat)
  
end subroutine ODE_Solve


!------------------------------------
!-   Quarternion to Euler Angle Convert
!------------------------------------

subroutine Quat2Euler(Ceb,theta,psi,fai)
  real(8),intent(in) :: Ceb(:,:)
  real(8),intent(out) :: theta,psi,fai
  
  theta = -asin(Ceb(1,3))
  psi = atan2(Ceb(1,2),Ceb(1,1))
  fai = atan2(Ceb(2,3),Ceb(3,3))
  
  theta = rad2deg(theta)
  psi = rad2deg(psi)
  if (psi <= 0.0d0) then
    psi = 360.0d0 + psi
  end if
  fai = rad2deg(fai)
  if (fai <= 0.0d0) then
    fai = 360.0d0 + fai
  end if
  
end subroutine Quat2Euler


!------------------------------------
!-        Euler to Quaternion
!------------------------------------

subroutine Euler2Quat(theta,psi,fai,Ceb,Cbe,quat)
  real(8),intent(in) :: theta,psi,fai
  real(8),intent(out) :: Ceb(:,:),Cbe(:,:),quat(:)
  integer :: quat_case
  real(8) :: norm
  
  Ceb(1,1) = cos(psi)*cos(theta)
  Ceb(2,1) = -sin(psi)*cos(fai) + cos(psi)*sin(theta)*sin(fai)
  Ceb(3,1) = sin(psi)*sin(fai) + cos(psi)*sin(theta)*cos(fai)
  Ceb(1,2) = sin(psi)*cos(theta)
  Ceb(2,2) = cos(psi)*cos(fai) + sin(psi)*sin(theta)*sin(fai)
  Ceb(3,2) = -cos(psi)*sin(fai) + sin(psi)*sin(theta)*cos(fai)
  Ceb(1,3) = -sin(theta)
  Ceb(2,3) = cos(theta)*sin(fai)
  Ceb(3,3) = cos(theta)*cos(fai)
  
  quat(1) = 0.5d0*sqrt(1.0d0+Ceb(1,1)-Ceb(2,2)-Ceb(3,3))
  quat(2) = 0.5d0*sqrt(1.0d0-Ceb(1,1)+Ceb(2,2)-Ceb(3,3))
  quat(3) = 0.5d0*sqrt(1.0d0-Ceb(1,1)-Ceb(2,2)+Ceb(3,3))
  quat(4) = 0.5d0*sqrt(1.0d0+Ceb(1,1)+Ceb(2,2)+Ceb(3,3))

  call maxcheck(quat,quat_case)

  select case(quat_case)
  case (1)
    quat(1) = 0.5d0*sqrt(1.0d0+Ceb(1,1)-Ceb(2,2)-Ceb(3,3))
    quat(2) = (Ceb(1,2)+Ceb(2,1))/(4.0d0*quat(1))
    quat(3) = (Ceb(3,1)+Ceb(1,3))/(4.0d0*quat(1))
    quat(4) = (Ceb(2,3)-Ceb(3,2))/(4.0d0*quat(1))
  case (2)
    quat(2) = 0.5d0*sqrt(1.0d0-Ceb(1,1)+Ceb(2,2)-Ceb(3,3))
    quat(1) = (Ceb(1,2)+Ceb(2,1))/(4.0d0*quat(2))
    quat(3) = (Ceb(2,3)+Ceb(3,2))/(4.0d0*quat(2))
    quat(4) = (Ceb(3,1)-Ceb(1,3))/(4.0d0*quat(2))
  case (3)
    quat(3) = 0.5d0*sqrt(1.0d0-Ceb(1,1)-Ceb(2,2)+Ceb(3,3))
    quat(1) = (Ceb(3,1)+Ceb(1,3))/(4.0d0*quat(3))
    quat(2) = (Ceb(2,3)+Ceb(3,2))/(4.0d0*quat(3))
    quat(4) = (Ceb(1,2)-Ceb(2,1))/(4.0d0*quat(3))
  case (4)
    quat(4) = 0.5d0*sqrt(1.0d0+Ceb(1,1)+Ceb(2,2)+Ceb(3,3))
    quat(1) = (Ceb(2,3)-Ceb(3,2))/(4.0d0*quat(4))
    quat(2) = (Ceb(3,1)-Ceb(1,3))/(4.0d0*quat(4))
    quat(3) = (Ceb(1,2)-Ceb(2,1))/(4.0d0*quat(4))
  end select
  
  norm = sqrt(quat(1)**2 + quat(2)**2 + quat(3)**2 + quat(4)**2)
  quat = quat / norm
  
  call DCM_Set(Cbe,Ceb,quat)
  
end subroutine Euler2Quat

!------------------------------------
!-           DCM Setting
!------------------------------------

subroutine DCM_Set(Cbe,Ceb,quat)
  real(8),intent(in) :: quat(:)
  real(8),intent(out) :: Cbe(:,:),Ceb(:,:)
  
  Cbe(1,1) = quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4)
  Cbe(2,1) = 2.0d0*(quat(1)*quat(2) + quat(3)*quat(4))
  Cbe(3,1) = 2.0d0*(quat(1)*quat(3) - quat(2)*quat(4))
  Cbe(1,2) = 2.0d0*(quat(1)*quat(2) - quat(3)*quat(4))
  Cbe(2,2) = quat(2)*quat(2) - quat(1)*quat(1) - quat(3)*quat(3) + quat(4)*quat(4)
  Cbe(3,2) = 2.0d0*(quat(2)*quat(3) + quat(1)*quat(4))
  Cbe(1,3) = 2.0d0*(quat(1)*quat(3) + quat(2)*quat(4))
  Cbe(2,3) = 2.0d0*(quat(2)*quat(3) - quat(1)*quat(4))
  Cbe(3,3) = quat(3)*quat(3) - quat(1)*quat(1) - quat(2)*quat(2) + quat(4)*quat(4)

  Ceb(1,1) = quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4)
  Ceb(2,1) = 2.0d0*(quat(1)*quat(2) - quat(3)*quat(4))
  Ceb(3,1) = 2.0d0*(quat(1)*quat(3) + quat(2)*quat(4))
  Ceb(1,2) = 2.0d0*(quat(1)*quat(2) + quat(3)*quat(4))
  Ceb(2,2) = quat(2)*quat(2) - quat(1)*quat(1) - quat(3)*quat(3) + quat(4)*quat(4)
  Ceb(3,2) = 2.0d0*(quat(2)*quat(3) - quat(1)*quat(4))
  Ceb(1,3) = 2.0d0*(quat(1)*quat(3) - quat(2)*quat(4))
  Ceb(2,3) = 2.0d0*(quat(2)*quat(3) + quat(1)*quat(4))
  Ceb(3,3) = quat(3)*quat(3) - quat(1)*quat(1) - quat(2)*quat(2) + quat(4)*quat(4)
  
end subroutine DCM_Set

end module Rocket_Dynamics