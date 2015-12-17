module Rotation
use standard_collection
implicit none

contains

!------------------------------------
!-         Moment Calculate
!------------------------------------

subroutine Moment_Calc(Q,S,M_fin,CNa,lcp,lcg,F,Clp,Cmq,Cnr,d,l,Va_syn,mp_dot,lcgp,Ma,Ka,Kj)
  real(8),intent(in) :: Q,S,CNa,M_fin,lcp,lcg,F(:),Clp,Cmq,Cnr,d,l,Va_syn,mp_dot,lcgp
  real(8),intent(out) :: Ma(:),Ka(:),Kj(:)
  
  !Moment of Aerodynamic
  Ma(1) = Q*S*CNa*0.07d0*deg2rad(M_fin) ! 0.07 は フ ィ ン の 圧 力 中 心 位 置(径 方 向)。真 面 目 に 計 算 す る な ら spanを 諸 元 で 呼 ぶ こ と
  Ma(2) = (lcp - lcg)*F(3)
  Ma(3) = -(lcp - lcg)*F(2)
  
  !計算の都合上omegaはあとのルンゲクッタでかける
  
  !Moment of Aerodynamic Damping
  Ka(1) = Q*Clp*S*d**2 / (2.0d0*Va_syn)
  Ka(2) = Q*Cmq*S*l**2 / (2.0d0*Va_syn)
  Ka(3) = Q*Cnr*S*l**2 / (2.0d0*Va_syn)
  
  !Jet Dumping Moment
  
  !For Rossr theory
  !Don't use
  !k2_pre = k2
  !k2 = (lcg - lcgp)**2
  !k2 = (k2 - k2_pre) / dt
  
  Kj(1) = 0.0d0 !-(Ipr_dot - mp_dot*0.5d0*(0.5d0*dno)**2) For Nozzle radius and Mass flow rate
  !Ignore Moment of inartia rate
  Kj(2) = -(mp_dot*((lcg - lcgp)**2 - (l-lcgp)**2))
  Kj(3) = -(mp_dot*((lcg - lcgp)**2 - (l-lcgp)**2))
  
end subroutine Moment_Calc
  

!------------------------------------
!-   Rotation Momentum Equation
!------------------------------------

function RotEq(Ib11,Ib12,Ib2,Ma,Ka,Kj,omega11,omega12,omega2)
  real(8) :: RotEq
  real(8) :: Ib11,Ib12,Ib2,Ma,Ka,kj,omega11,omega12,omega2
  
  RotEq = ((Ib11 - Ib12)*omega11*omega12 + (Ma + (Ka + Kj)*omega2)) / Ib2
  
end function RotEq


!------------------------------------
!- Angler Speed - ODE Solve for RK4
!------------------------------------

subroutine ODE_Solve(Ib,Ma,Ka,Kj,omega,omega_pre,Ceb,Cbe,quat,quat_pre,dt)
  real(8),intent(in) :: Ib(:),Ma(:),Ka(:),Kj(:),dt
  real(8),intent(out) :: omega_pre(:),Ceb(:,:),Cbe(:,:),quat_pre(:)
  real(8),intent(inout) :: omega(:),quat(:)
  real(8) :: dX(4),dY(4),dZ(4),norm
  
  dX(1) = RotEq(Ib(2),Ib(3),Ib(1),Ma(1),Ka(1),Kj(1),omega(2),omega(3),omega(1))
  dX(2) = RotEq(Ib(2),Ib(3),Ib(1),Ma(1),Ka(1),Kj(1),omega(2),omega(3),omega(1) + 0.5d0*dt*dX(1))
  dX(3) = RotEq(Ib(2),Ib(3),Ib(1),Ma(1),Ka(1),Kj(1),omega(2),omega(3),omega(1) + 0.5d0*dt*dX(2))
  dX(4) = RotEq(Ib(2),Ib(3),Ib(1),Ma(1),Ka(1),Kj(1),omega(2),omega(3),omega(1) + dt*dX(3))
  
  dY(1) = RotEq(Ib(3),Ib(1),Ib(2),Ma(2),Ka(2),Kj(2),omega(1),omega(3),omega(2))
  dY(2) = RotEq(Ib(3),Ib(1),Ib(2),Ma(2),Ka(2),Kj(2),omega(1),omega(3),omega(2) + 0.5d0*dt*dY(1))
  dY(3) = RotEq(Ib(3),Ib(1),Ib(2),Ma(2),Ka(2),Kj(2),omega(1),omega(3),omega(2) + 0.5d0*dt*dY(2))
  dY(4) = RotEq(Ib(3),Ib(1),Ib(2),Ma(2),Ka(2),Kj(2),omega(1),omega(3),omega(2) + dt*dY(3))
  
  dZ(1) = RotEq(Ib(1),Ib(2),Ib(3),Ma(3),Ka(3),Kj(3),omega(1),omega(2),omega(3))
  dZ(2) = RotEq(Ib(1),Ib(2),Ib(3),Ma(3),Ka(3),Kj(3),omega(1),omega(2),omega(3) + 0.5d0*dt*dZ(1))
  dZ(3) = RotEq(Ib(1),Ib(2),Ib(3),Ma(3),Ka(3),Kj(3),omega(1),omega(2),omega(3) + 0.5d0*dt*dZ(2))
  dZ(4) = RotEq(Ib(1),Ib(2),Ib(3),Ma(3),Ka(3),Kj(3),omega(1),omega(2),omega(3) + dt*dZ(3))
  
  omega_pre = omega
  
  omega(1) = omega_pre(1) + ((dX(1) + 2.0d0*dX(2) + 2.0d0*dX(3) + dX(4)) / 6.0d0)*dt
  omega(2) = omega_pre(2) + ((dY(1) + 2.0d0*dY(2) + 2.0d0*dY(3) + dY(4)) / 6.0d0)*dt
  omega(3) = omega_pre(3) + ((dZ(1) + 2.0d0*dZ(2) + 2.0d0*dZ(3) + dZ(4)) / 6.0d0)*dt
  
  quat_pre = quat
  
  quat(1) = quat_pre(1) + 0.5d0*(quat_pre(2)*omega(3) - quat_pre(3)*omega(2) + quat_pre(4)*omega(1))*dt
  quat(2) = quat_pre(2) + 0.5d0*(-quat_pre(1)*omega(3) + quat_pre(3)*omega(1) + quat_pre(4)*omega(2))*dt
  quat(3) = quat_pre(3) + 0.5d0*(quat_pre(1)*omega(2) - quat_pre(2)*omega(1) + quat_pre(4)*omega(3))*dt
  quat(4) = quat_pre(4) + 0.5d0*(-quat_pre(1)*omega(1) - quat_pre(2)*omega(2) - quat_pre(3)*omega(3))*dt

  norm = sqrt(quat(1)**2 + quat(2)**2 + quat(3)**2 + quat(4)**2)
  quat = quat / norm
  
  call DCM_Set(Cbe,Ceb,quat)
  
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

end module Rotation