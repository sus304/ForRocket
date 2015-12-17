module Environment
use standard_collection
use atmosphere1976 
implicit none

contains

!------------------------------------
!-       Atmosphere Calculate
!------------------------------------

subroutine Atmosphere_Setting(Ta_0,P_0,rho_0,g0,Z,g,Cs,Ta,P,rho)
  
  real(8),intent(in) :: Ta_0,P_0,rho_0,g0,Z
  real(8),intent(out) :: g,Cs
  real(8),intent(inout) :: Ta,P,rho 
  
  call Atmosphere(Z/1000.0d0,rho,P,Ta)
  Ta = Ta*Ta_0
  P = P*P_0
  rho = rho*rho_0

  g = g0*(Re / (Re + Z))**2

  Cs = sqrt(ganma*R*Ta)
  
end subroutine Atmosphere_Setting


!------------------------------------
!-    Velocity of Wind Calculate
!------------------------------------

subroutine VelocityOfWind_Vecter(Vw_syn,Vw_angle,Z,Hw,Wh,Vw)
  
  real(8),intent(in) :: Vw_syn,Vw_angle,Z,Hw,Wh
  real(8),intent(out) :: Vw(:)
  
  !風 向 は - を つ け て deg か ら の 風 に し て る
  
  Vw(1) = -Vw_syn*cos(deg2rad(Vw_angle))*(Z / Hw)**(1.0d0 / Wh)
  Vw(2) = -Vw_syn*sin(deg2rad(Vw_angle))*(Z / Hw)**(1.0d0 / Wh)
  Vw(3) = 0.0d0
  
end subroutine VelocityOfWind_Vecter


end module Environment