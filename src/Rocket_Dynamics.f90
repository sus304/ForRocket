module Rocket_Dynamics
use standard_collection
implicit none

contains

!**********************************************************************
!                                 Body
!**********************************************************************
!------------------------------------
!-         Mass Calculate
!------------------------------------
subroutine calc_Mass
  ! 推力履歴は必ず作動開始からのデータであること
  ! 質量減少はthrustがあれば発生するようになっている
  
  if (i < index_burn) mp_dot = thrust(i) / (Isp * g0)
  mox_dot = (mp_dot - mf_dot) * (0.5d0 + 0.5d0 * sign(1.0d0,mp_dot - mf_dot - 1.0d-6)) !- thrustが負だと酸化剤が増加するので回避

  mf = (mf - mf_dot * dt) * (0.5d0 + 0.5d0 * sign(1.0d0,mf - mf_a - 1.0d-6)) &
     +               mf_a * (0.5d0 - 0.5d0 * sign(1.0d0,mf - mf_a - 1.0d-6))
  mox = (mox - mox_dot * dt) * (0.5d0 + 0.5d0 * sign(1.0d0,mox - 1.0d-6))

  mp = mf + mox
  m = ms + mp

end subroutine calc_Mass

!------------------------------------
!-    Moment of Inertia Calculate
!------------------------------------
subroutine calc_CG_IM  
  !--- Center of Gravity---
  lcgox = (lcgox_0 - 0.5d0 * (1.0d0 - (mox / mox_0)) * ltank) * (0.5d0 + 0.5d0 * sign(1.0d0,mox - 1.0d-6))
  lcgp = (mf * lcgf + mox * lcgox) / mp
  lcg = ((mp * lcgp) + (ms * lcgs)) / m
  
  !--- Moment of Inertia ---
  !-タンク内酸化剤,液体になるのでまた別の考えが必要
  !Ipp = (mox*((0.5*ltank - (l - lcgox))**2 / 12.0d0 + (lcgox - lcgp)**2) + mf*(lcgf - lcgp)**2) + mp*(lcg - lcgp)**2
  
  !- Fuel
  Ifp = mf * lf / 12.0d0
  Ifr = mf * (((df1 + df2) / 4.0d0) + ((df1 + df2) / 4.0d0) * (1.0d0 - (mf / mf_b)))
  
  !-Loading
  Ib = Ifp + mf * (lcg - lcgf)**2 + Is * ((ms + mox) / ms) + (ms + mox) * (lcg - lcgs) * (lcg - lcgs)
  Ib(1) = Ir + Ifr !-軸対称でピッチヨーは同じなのでロールのみ上書きで書いちゃう
  
end subroutine calc_CG_IM

!**********************************************************************
!                            Translation
!**********************************************************************
!------------------------------------
!-         Load Calculate
!------------------------------------
subroutine calc_Force
  real(8) :: thrust_correction
  
  Cd = getMachCd(Mach)
  Dx = Q * Cd * S
  Ny = Q * CNa * S * beta
  Nz = Q * CNa * S * alpha
  
  if (i < index_burn) then
    thrust_correction = (thrust(i) + (Pa_0 - Pa) * Ae)
  else
    thrust_correction = 0.0d0
  end if

  F(1) = thrust_correction - Dx
  F(2) = Ny
  F(3) = -Nz
  
end subroutine calc_Force


!------------------------------------
!-        Airspeed Calculate
!------------------------------------
subroutine calc_Airspeed
  
  Va_pre = Va
  Va = matmul(Ceb,(Ve - Vw))
  Va_abs = abs_3axis(Va)
  
  alpha = atan2(Va(3),Va(1))
  beta = asin(-Va(2) / Va_abs)
  
  Mach = Va_abs / Cs
  Q = 0.5d0 * rho * Va_abs * Va_abs
  
end subroutine calc_Airspeed


!------------------------------------
!-            Integral
!------------------------------------
subroutine Acc2Position
  
  acce_pre = acce
  Ve_pre = Ve
  Position_pre = Position

  acce = matmul(Cbe,F)
  acce = acce / m
  acce(3) = acce(3) - g
  acce_abs = abs_3axis(acce)
  
  Ve = Ve_pre + 0.5d0 * (acce + acce_pre) * dt
  Ve_abs = abs_3axis(Ve)
  
  Position = Position_pre + 0.5d0 * (Ve + Ve_pre) * dt
  
end subroutine Acc2Position


!------------------------------------
!-      Parachute Aerodynamics
!------------------------------------
subroutine Parachute_Aerodynamics
  
  Ve_pre = Ve
  acce(3) = (0.5d0 * rho * Ve(3) * Ve(3) * CdS - m * g) / m
  
  Ve(1) = Vw(1) !横方向は等風速
  Ve(2) = Vw(2)
  Ve(3) = Ve(3) + acce(3) * dt
  Ve_abs = abs_3axis(Ve)
  
  Position = Position + 0.5d0 * (Ve + Ve_pre) * dt 
  
end subroutine Parachute_Aerodynamics

!**********************************************************************
!                              Rotation
!**********************************************************************
!------------------------------------
!-         Moment Calculate
!------------------------------------
subroutine calc_Moment
  
  !Moment of Aerodynamic これの符号は軸の回転方向に対応
  Ma(1) = Q * S * CNa * 0.07d0 * deg2rad(M_fin) ! 0.07[m]はフィンの圧力中心位置(径方向)。真面目に計算するならspanを諸元で呼ぶこと
  Ma(2) = (lcp - lcg) * F(3)
  Ma(3) = -(lcp - lcg) * F(2)
    
  !Moment of Aerodynamic Damping これの符号は角速度の方向と対応
  !omegaはあとの運動方程式内で掛ける
  Ka(1) = Q * Clp * S * d**2 / (2.0d0 * Va_abs)
  Ka(2) = Q * Cmq * S * l**2 / (2.0d0 * Va_abs)
  Ka(3) = Q * Cnr * S * l**2 / (2.0d0 * Va_abs)
  
  !Jet Dumping Moment これの符号は角速度の方向と対応
  
  !For Rossr theory
  !Don't use
  !k2_pre = k2
  !k2 = (lcg - lcgp)**2
  !k2 = (k2 - k2_pre) / dt
  
  Kj(1) = 0.0d0 !-(Ipr_dot - mp_dot*0.5d0*(0.5d0*dno)**2) For Nozzle radius and Mass flow rate
  !Ignore Moment of inartia rate
  Kj(2) = -(mp_dot * ((lcg - lcgp)**2 - (l - lcgp)**2))
  Kj(3) = -(mp_dot * ((lcg - lcgp)**2 - (l - lcgp)**2))
  
end subroutine calc_Moment
  
!------------------------------------
!-   Rotation Momentum Equation
!------------------------------------
function RotEq(RKs)
  real(8) :: RotEq(3)
  real(8),intent(in) :: RKs(3)
  
  RotEq(1) = ((Ib(2) - Ib(3)) * omega(2) * omega(3) + (Ma(1) + (Ka(1) + Kj(1)) * (omega(1) + RKs(1)))) / Ib(1)
  RotEq(2) = ((Ib(3) - Ib(1)) * omega(1) * omega(3) + (Ma(2) + (Ka(2) + Kj(2)) * (omega(2) + RKs(2)))) / Ib(2)
  RotEq(3) = ((Ib(1) - Ib(2)) * omega(1) * omega(2) + (Ma(3) + (Ka(3) + Kj(3)) * (omega(3) + RKs(3)))) / Ib(3)
  
end function RotEq

!------------------------------------
!- Angler Speed - ODE Solve for RK4
!------------------------------------
subroutine ODE_Solve
  real(8) :: dt2
  real(8) :: RKs(3) !- RK4用のパラメータ
  real(8) :: omega_(3)
  real(8) :: omega_matrix(4,4)
  real(8) :: norm
  integer :: k

  dt2 = 0.5d0 * dt
  omega_ = 0.0
  !- iteration1
  RKs = 0.0d0
  RKs = RotEq(RKs)
  omega_ = RKs
  !- iteration2
  RKs = RKs * dt2
  RKs = RotEq(RKs)
  omega_ = omega_ + 2.0d0 * RKs
  !- iteration3
  RKs = RKs * dt2
  RKs = RotEq(RKs)
  omega_ = omega_ + 2.0d0 * RKs
  !- iteration4
  RKs = RKs * dt
  RKs = RotEq(RKs)
  omega_ = omega_ + RKs

  omega = omega + (omega_ / 6.0d0) * dt

  omega_matrix(1,:) = (/0.0d0      ,omega_(3)  ,-omega_(2) ,omega_(1)/)
  omega_matrix(2,:) = (/-omega_(3) ,0.0d0      ,omega_(1)  ,omega_(2)/)
  omega_matrix(3,:) = (/omega_(2)  ,-omega_(1) ,0.0d0      ,omega_(3)/)
  omega_matrix(4,:) = (/-omega_(1) ,-omega_(2) ,-omega_(3) ,0.0d0    /)

  quat = quat + 0.5d0 * matmul(omega_matrix,quat) * dt
  norm = sqrt(quat(1)**2 + quat(2)**2 + quat(3)**2 + quat(4)**2)
  quat = quat / norm
  
  call set_DCM
  
end subroutine ODE_Solve


!------------------------------------
!- Quarternion to Euler Angle Convert
!------------------------------------
subroutine Quat2Euler
  
  theta = -asin(Ceb(1,3))
  psi = atan2(Ceb(1,2),Ceb(1,1))
  fai = atan2(Ceb(2,3),Ceb(3,3))
  
  theta = rad2deg(theta)
  psi = rad2deg(psi)
  psi =             psi * (0.5d0 + 0.5d0 * sign(1.0d0,psi + 1.0d-6)) &
      + (psi + 360.0d0) * (0.5d0 - 0.5d0 * sign(1.0d0,psi + 1.0d-6))
  fai = rad2deg(fai)
  fai =             fai * (0.5d0 + 0.5d0 * sign(1.0d0,fai + 1.0d-6)) &
      + (fai + 360.0d0) * (0.5d0 - 0.5d0 * sign(1.0d0,fai + 1.0d-6))
  
end subroutine Quat2Euler

!------------------------------------
!-        Euler to Quaternion
!------------------------------------
subroutine Euler2Quat
  integer :: quat_max(1),quat_case
  real(8) :: norm
  
  Ceb(1,:) = (/cos(psi)*cos(theta)                            ,sin(psi)*cos(theta)                            ,-sin(theta)        /)
  Ceb(2,:) = (/-sin(psi)*cos(fai)+cos(psi)*sin(theta)*sin(fai),cos(psi)*cos(fai)+sin(psi)*sin(theta)*sin(fai) ,cos(theta)*sin(fai)/)
  Ceb(3,:) = (/sin(psi)*sin(fai)+cos(psi)*sin(theta)*cos(fai) ,-cos(psi)*sin(fai)+sin(psi)*sin(theta)*cos(fai),cos(theta)*cos(fai)/)
  
  quat(1) = 0.5d0 * sqrt(1.0d0 + Ceb(1,1) - Ceb(2,2) - Ceb(3,3))
  quat(2) = 0.5d0 * sqrt(1.0d0 - Ceb(1,1) + Ceb(2,2) - Ceb(3,3))
  quat(3) = 0.5d0 * sqrt(1.0d0 - Ceb(1,1) - Ceb(2,2) + Ceb(3,3))
  quat(4) = 0.5d0 * sqrt(1.0d0 + Ceb(1,1) + Ceb(2,2) + Ceb(3,3))

  quat_max = maxloc(quat)
  quat_case = quat_max(1)

  select case(quat_case)
  case (1)
    quat(1) = 0.5d0 * sqrt(1.0d0 + Ceb(1,1) - Ceb(2,2) - Ceb(3,3))
    quat(2) = (Ceb(1,2) + Ceb(2,1)) / (4.0d0 * quat(1))
    quat(3) = (Ceb(3,1) + Ceb(1,3)) / (4.0d0 * quat(1))
    quat(4) = (Ceb(2,3) - Ceb(3,2)) / (4.0d0 * quat(1))
  case (2)
    quat(2) = 0.5d0 * sqrt(1.0d0 - Ceb(1,1) + Ceb(2,2) - Ceb(3,3))
    quat(1) = (Ceb(1,2) + Ceb(2,1)) / (4.0d0 * quat(2))
    quat(3) = (Ceb(2,3) + Ceb(3,2)) / (4.0d0 * quat(2))
    quat(4) = (Ceb(3,1) - Ceb(1,3)) / (4.0d0 * quat(2))
  case (3)
    quat(3) = 0.5d0 * sqrt(1.0d0 - Ceb(1,1) - Ceb(2,2) + Ceb(3,3))
    quat(1) = (Ceb(3,1) + Ceb(1,3)) / (4.0d0 * quat(3))
    quat(2) = (Ceb(2,3) + Ceb(3,2)) / (4.0d0 * quat(3))
    quat(4) = (Ceb(1,2) - Ceb(2,1)) / (4.0d0 * quat(3))
  case (4)
    quat(4) = 0.5d0 * sqrt(1.0d0 + Ceb(1,1) + Ceb(2,2) + Ceb(3,3))
    quat(1) = (Ceb(2,3) - Ceb(3,2)) / (4.0d0 * quat(4))
    quat(2) = (Ceb(3,1) - Ceb(1,3)) / (4.0d0 * quat(4))
    quat(3) = (Ceb(1,2) - Ceb(2,1)) / (4.0d0 * quat(4))
  end select
  
  norm = sqrt(quat(1)**2 + quat(2)**2 + quat(3)**2 + quat(4)**2)
  quat = quat / norm
  
end subroutine Euler2Quat

!------------------------------------
!-           DCM Setting
!------------------------------------

subroutine set_DCM
  
  Cbe(1,1) = quat(1) * quat(1) - quat(2) * quat(2) - quat(3) * quat(3) + quat(4) * quat(4)
  Cbe(2,1) = 2.0d0 * (quat(1) * quat(2) + quat(3) * quat(4))
  Cbe(3,1) = 2.0d0 * (quat(1) * quat(3) - quat(2) * quat(4))
  Cbe(1,2) = 2.0d0 * (quat(1) * quat(2) - quat(3) * quat(4))
  Cbe(2,2) = quat(2) * quat(2) - quat(1) * quat(1) - quat(3) * quat(3) + quat(4) * quat(4)
  Cbe(3,2) = 2.0d0 * (quat(2) * quat(3) + quat(1) * quat(4))
  Cbe(1,3) = 2.0d0 * (quat(1) * quat(3) + quat(2) * quat(4))
  Cbe(2,3) = 2.0d0 * (quat(2) * quat(3) - quat(1) * quat(4))
  Cbe(3,3) = quat(3) * quat(3) - quat(1) * quat(1) - quat(2) * quat(2) + quat(4) * quat(4)

  Ceb(1,1) = quat(1) * quat(1) - quat(2) * quat(2) - quat(3) * quat(3) + quat(4) * quat(4)
  Ceb(2,1) = 2.0d0 * (quat(1) * quat(2) - quat(3) * quat(4))
  Ceb(3,1) = 2.0d0 * (quat(1) * quat(3) + quat(2) * quat(4))
  Ceb(1,2) = 2.0d0 * (quat(1) * quat(2) + quat(3) * quat(4))
  Ceb(2,2) = quat(2) * quat(2) - quat(1) * quat(1) - quat(3) * quat(3) + quat(4) * quat(4)
  Ceb(3,2) = 2.0d0 * (quat(2) * quat(3) - quat(1) * quat(4))
  Ceb(1,3) = 2.0d0 * (quat(1) * quat(3) - quat(2) * quat(4))
  Ceb(2,3) = 2.0d0 * (quat(2) * quat(3) + quat(1) * quat(4))
  Ceb(3,3) = quat(3) * quat(3) - quat(1) * quat(1) - quat(2) * quat(2) + quat(4) * quat(4)
  
end subroutine set_DCM

end module Rocket_Dynamics