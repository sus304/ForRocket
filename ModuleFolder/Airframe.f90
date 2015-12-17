module Airframe
use standard_collection
implicit none

contains

!------------------------------------
!-         Mass Calculate
!------------------------------------

subroutine Mass_Calc(thrust,Isp,g0,mf_a,mf_dot,ms,mp_dot,mox_dot,mp,m,mf,mox,dt)
  real(8),intent(in) :: thrust,Isp,g0,mf_a,mf_dot,ms,dt
  real(8),intent(out) :: mp_dot,mox_dot,mp,m
  real(8),intent(inout) :: mf,mox
  
  mp_dot = thrust / (Isp*g0)
  mox_dot = mp_dot - mf_dot

  if (mf > mf_a) then
    mf = mf - mf_dot*dt
  end if
  if (mox > 0.0d0) then
    mox = mox - mox_dot*dt
  end if

  mp = mf + mox
  m = ms + mp
  
end subroutine Mass_Calc

!------------------------------------
!-    Moment of Inertia Calculate
!------------------------------------

subroutine MI_Calc(lcgox_0,mox,mox_0,ltank,mf,lcgf,mp,ms,lcgs,m,lf,df1,df2,mf_b,Is,Ir,lcgox,lcgp,lcg,Ifp,Ifr,Ib)
  real(8),intent(in) :: lcgox_0,mox,mox_0,ltank,mf,lcgf,mp,ms,lcgs,m,lf,df1,df2,mf_b,Is,Ir
  real(8),intent(out) :: lcgox,lcgp,lcg,Ifp,Ifr,Ib(:)
  
  lcgox = lcgox_0 - 0.5d0*(1.0d0 - (mox / mox_0))*ltank
  if (mox <= 0.0d0) lcgox = 0.0d0
  lcgp = (mf*lcgf + mox*lcgox) / mp
  lcg = ((mp*lcgp) + (ms*lcgs)) / m
  
  !Ipp = (mox*((0.5*ltank - (l - lcgox))**2 / 12.0d0 + (lcgox - lcgp)**2) + mf*(lcgf - lcgp)**2) + mp*(lcg - lcgp)**2
  
  !IM Fuel Pitch
  Ifp = mf*lf / 12.0d0
  Ifr = mf*(((df1 + df2) / 4.0d0) + ((df1 + df2) / 4.0d0)*(1.0d0 - (mf / mf_b)))
  
  Ib = Ifp + mf*(lcg - lcgf)**2 + Is*((ms + mox) / ms) + (ms + mox)*(lcg - lcgs)**2
  Ib(1) = Ir + Ifr
  
end subroutine MI_Calc


end module Airframe