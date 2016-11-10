module Rocket_Class
implicit none

type Rocket_Type
!------------------------------------
!-    All loading (subscript:none)
!------------------------------------
real(8) :: l            ! 機体全長 :: length
real(8) :: m            ! 全機質量 :: mass
real(8) :: d,S          ! 機体直径,機体断面積
real(8) :: lcg          ! 全機重心位置(from nose) :: length center of gravity
real(8) :: lcg_0        ! 初期全機重心位置(from nose)
real(8) :: lcp          ! 圧力中心位置(from nose) :: length center of pressure
real(8) :: Ib(3)        ! 全機3軸慣性モーメント(xb,yb,zb) :: Inartia moment of body
real(8) :: Clp,Cmq,Cnr  ! 減衰モーメント係数(Clp:ロール,ピッチ,ヨー)
real(8) :: CNa,M_fin    ! 法線力係数,フィン取り付け角(機軸に対する)
real(8) :: Cd           ! 抗力係数

!------------------------------------
!-      Structure (subscript:s)
!------------------------------------
real(8) :: ms     ! 空虚質量
real(8) :: lcgs   ! 空虚重心位置(from nose)
real(8) :: Is,Ir  ! 空虚ピッチ慣性モーメント,ロール慣性モーメント(Iroll)

!------------------------------------
!-            Recovery
!------------------------------------
real(8) :: CdS1,CdS2       ! 1段目,2段目パラシュート抗力特性 :: Cd * S
real(8) :: CdS             ! 計算時に代入するパラシュート抗力特性
real(8) :: T_sepa,H_sepa   ! 1段目分離時刻(X+t[sec]),2段目開傘高度[m]


!------------------------------------
!-             Engine
!- (Oxidizer   subscript:ox)
!- (Fuel       subscript:f )
!- (Propellant subscript:p )
!------------------------------------
real(8) :: mox,mox_dot        ! 酸化剤質量,酸化剤質量流量
real(8) :: mox_0              ! 初期酸化剤質量
real(8) :: lcgox              ! 酸化剤重心位置(from nose)
real(8) :: lcgox_0
real(8) :: ltank              ! 酸化剤タンク長さ
real(8) :: mf                 ! 燃料質量
real(8) :: mf_b,mf_a,mf_dot   ! mf_before,mf_after
real(8) :: lcgf               ! 燃料重心位置(from nose)
real(8) :: Ifp,Ifr            ! 燃料ピッチ慣性モーメント,ロール慣性モーメント
real(8) :: lf,df1,df2         ! 燃料長さ,燃料外径,内径
real(8) :: mp,mp_dot          ! 推進剤質量
real(8) :: lcgp               ! 推進剤重心位置(from nose)
real(8) :: lcgp_0

real(8) :: Isp,It             ! 比推力,全力積
real(8),allocatable :: thrust(:)
real(8) :: de,Ae              ! ノズル出口 :: diameter exit

!------------------------------------
!-           Translation
!- (earth plane  subscript:e)
!- (body         subscript:b)
!- (Air          subscript:a)
!------------------------------------
real(8) :: Q              ! 動圧
real(8) :: Q_pre          ! 保存/比較用1step前
real(8) :: Dx,Ny,Nz       ! 各機体軸に対する空気力
real(8) :: F(3)           ! 機体座標系における外力
real(8) :: Position(3)    ! 座標(xe,ye,ze)
real(8) :: Position_pre(3)
real(8) :: Ve(3),Va(3)    ! 対地速度,対気速度
real(8) :: Ve_abs,Va_abs  ! ベクトル合成値
real(8) :: Ve_pre(3),Va_pre(3)
real(8) :: acce(3)        ! 対地加速度
real(8) :: acce_abs
real(8) :: acce_pre(3)
real(8) :: Mach           ! Mach Number
real(8) :: Mach_pre

!------------------------------------
!-             Rotation
!------------------------------------
real(8) :: Ma(3),Ka(3),Kj(3)      ! 空力モーメント,空力減衰モーメント,ジェットダンピングモーメント
real(8) :: omega(3),omega_pre(3)  ! 角速度
real(8) :: theta,psi,fai          ! ピッチ角,ヨー角,ロール角
real(8) :: alpha,beta             ! 迎角,横滑り角
real(8) :: theta_0,psi_0          ! 打ち上げ射角,打ち上げ方位角

!------------------------------------
!-            Quaternion
!------------------------------------
real(8) :: quat(4),quat_pre(4)    ! quaternion
real(8),dimension(3,3) :: Cbe,Ceb ! DCM body-->earth , earth-->body


end type Rocket_Type

end module Rocket_Class