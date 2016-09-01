module Rocket_Class
implicit none

type Rocket_Type
!------------------------------------
!-            Structure 
!------------------------------------
real(8) :: ms,m !-----空虚質量,全機質量---
real(8) :: d,S
real(8) :: l,lcgs,lcg,lcp,ltank,lcg_0
real(8) :: Is,Ir,Ib(3) !----空虚ピッチ慣性モーメント(Is),ロール慣性モーメント(Ir),全機での3軸慣性モーメント-----
real(8) :: Clp,Cmq,Cnr,Cd,CNa,M_fin
real(8) :: CdS1,CdS2,CdS !--1段目,2段目パラシュート抗力特性,計算時に代入するパラシュート抗力特性

!------------------------------------
!-             Engine
!------------------------------------
real(8) :: mox,mox_0,mf,mf_b,mf_a,mox_dot,mf_dot,mp,mp_dot
real(8) :: lcgp,lcgox,lcgf,lcgp_0,lcgox_0
real(8) :: Ifp,Ifr !--燃料ピッチ慣性モーメント,ロール慣性モーメント
real(8) :: lf,df1,df2 !--燃料長さ,燃料外径,内径
real(8) :: Isp
real(8) :: thrust = 0.0d0
real(8) :: de,Ae !-ノズル

!------------------------------------
!-           Translation
!------------------------------------
real(8) :: Q,Dx,Ny,Nz,F(3)
real(8) :: Position(3),Ve(3),Va(3),acce(3)
real(8) :: Ve_abs,Va_abs,acce_abs !--ベクトル合成値--
real(8) :: Position_pre(3),Ve_pre(3),Va_pre(3),acce_pre(3),Q_pre,Mach_pre !--保存/比較用1step前
real(8) :: Mach

real(8) :: T_sepa,H_sepa !--1段目分離時刻(X+t[sec]),2段目開傘高度[m]

!------------------------------------
!-             Rotation
!------------------------------------
real(8) :: Ma(3),Ka(3),Kj(3) !--空力モーメント,空力減衰モーメント,ジェットダンピングモーメント
real(8) :: omega(3),omega_pre(3)
real(8) :: theta,psi,fai,alpha,beta
real(8) :: theta_0,psi_0

!------------------------------------
!-            Quaternion
!------------------------------------
real(8) :: quat(4),quat_pre(4)
real(8),dimension(3,3) :: Cbe,Ceb !-- body-->earth,earth-->body --

end type Rocket_Type

end module Rocket_Class