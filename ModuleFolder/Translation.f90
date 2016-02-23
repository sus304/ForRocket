module Translation
use standard_collection
implicit none

contains

!------------------------------------
!-         Load Calculate
!------------------------------------

subroutine Load_Calc(Q,Cd,S,CNa,alpha,beta,eof,Dx,Ny,Nz,thrust,F)
  real(8),intent(in) :: Q,Cd,S,CNa,alpha,beta
  real(8),intent(out) :: Dx,Ny,Nz,thrust,F(:)
  integer,intent(inout) :: eof
  
  Dx = Q*Cd*S
  Ny = Q*CNa*S*beta
  Nz = Q*CNa*S*alpha

  if (eof >= 0) then
    read (62,*,iostat = eof) thrust
  else
    eof = -1
    thrust = 0.0d0
  end if

  F(1) = thrust - Dx
  F(2) = Ny
  F(3) = -Nz
  
end subroutine Load_Calc


!------------------------------------
!-        Airspeed Calculate
!------------------------------------

subroutine Airspeed_Calc(Ceb,Ve,Vw,Cs,rho,Va_pre,Va,alpha,beta,Mach,Q,Va_syn)
  real(8),intent(in) :: Ceb(:,:),Ve(:),Vw(:),Cs,rho
  real(8),intent(out) :: Va_syn,Va_pre(:),Va(:),alpha,beta,Mach,Q
  
  Va_pre = Va
  Va = matmul(Ceb,(Ve - Vw))
  Va_syn = sqrt((Va(1)*Va(1)) + (Va(2)*Va(2)) + (Va(3)*Va(3)))
  
  alpha = atan(Va(3) / Va(1)) ; beta = asin(-Va(2) / Va_syn)
  
  Mach = Va_syn / Cs
  Q = 0.5d0*rho*Va_syn**2
  
end subroutine Airspeed_Calc


!------------------------------------
!-         Position Integral
!------------------------------------

subroutine Acceleration_Integral(F,Cbe,m,g,acc_pre,acc_syn,Ve_pre,Ve_syn,acc,Ve,dt)
  real(8),intent(in) :: F(:),Cbe(:,:),m,g,dt
  real(8),intent(out) :: acc_pre(:),acc_syn,Ve_pre(:),Ve_syn
  real(8),intent(inout) :: acc(:),Ve(:)
  
  acc_pre = acc
  acc = matmul(Cbe,F)
  acc = acc / m
  acc(3) = acc(3) - g
  acc_syn = sqrt((acc(1)*acc(1)) + (acc(2)*acc(2)) + (acc(3)*acc(3)))
  
  Ve_pre = Ve
  
  Ve = Ve_pre + Integral_3D(acc,acc_pre,dt)
  
  Ve_syn = sqrt((Ve(1)*Ve(1)) + (Ve(2)*Ve(2)) + (Ve(3)*Ve(3)))
  
end subroutine Acceleration_Integral


!------------------------------------
!-        Velocity Integral
!------------------------------------

subroutine Velocity_Integral(Ve,Ve_pre,Position_pre,Position,dt)
  real(8),intent(in) :: Ve(:),Ve_pre(:),dt
  real(8),intent(out) :: Position_pre(:)
  real(8),intent(inout) :: Position(:)
  
  Position_pre = Position
  Position = position_pre + Integral_3D(Ve,Ve_pre,dt)
  
end subroutine Velocity_Integral


!------------------------------------
!-      Parachute Aerodynamics
!------------------------------------

subroutine Parachute_Aerodynamics(rho,CdS,m,g,Vw,acc,Ve,Ve_pre,dt)
  real(8),intent(in) :: rho,CdS,m,g,Vw(:),dt
  real(8),intent(out) :: acc(:)
  real(8),intent(inout) :: Ve(:),Ve_pre(:)
  
  Ve_pre = Ve
  acc(3) = (0.5d0*rho*Ve(3)**2*CdS - m*g) / m
  
  Ve(1) = Vw(1)
  Ve(2) = Vw(2)
  Ve(3) = Ve(3) + acc(3)*dt
  
end subroutine Parachute_Aerodynamics


end module Translation