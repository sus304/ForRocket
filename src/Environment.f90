module Environment
use standard_collection
use atmosphere1976 
implicit none

contains

!------------------------------------
!-       Atmosphere Calculate
!- input
!- Ta :: Temperature Air [K]
!- Pa :: Pressure Air [Pa]
!- rho :: Dencity Air [kg/m^3]
!- g :: gravity
!- Z :: Altitude [m]
!- output
!- Ta,Pa,rho,g
!- Cs :: Sonic of Speed [m/s]
!- 高度100 kmまで対応
!------------------------------------
subroutine Atmosphere_Setting(Ta_0,Pa_0,rho_0,g0,Z,g,Cs,Ta,Pa,rho)
  real(8),intent(in) :: Ta_0,Pa_0,rho_0,g0,Z
  real(8),intent(out) :: g,Cs
  real(8),intent(inout) :: Ta,Pa,rho 
  
  call Atmosphere(Z/1000.0d0,rho,Pa,Ta) ! 出力は高度に対応した海面レベルからの係数
  Ta = Ta * Ta_0
  Pa = Pa * Pa_0
  rho = rho * rho_0

  g = g0 * (Re / (Re + Z))**2

  Cs = sqrt(gamma * R * Ta)
end subroutine Atmosphere_Setting

!------------------------------------
!-    Velocity of Wind Calculate
!- べき法則を利用した風速の高度変化
!- べき法則の適用可能な高度が限界
!- Vw :: Velocity of Wind
!------------------------------------
subroutine VelocityOfWind_Vecter(Vw_abs,Vw_angle,Z,Hw,Wh,Vw)
  
  real(8),intent(in) :: Vw_abs,Vw_angle,Z,Hw,Wh
  real(8),intent(out) :: Vw(:)
  
  !風向は-をつけてdegからの風にしてる
  Vw(1) = -Vw_abs*cos(deg2rad(Vw_angle))*(Z / Hw)**(1.0d0 / Wh)
  Vw(2) = -Vw_abs*sin(deg2rad(Vw_angle))*(Z / Hw)**(1.0d0 / Wh)
  Vw(3) = 0.0d0
  
end subroutine VelocityOfWind_Vecter

end module Environment