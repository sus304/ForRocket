module Standard_Collection
implicit none

real(8),parameter :: PI = 3.1415926535898d0 , R = 287.1d0 , gamma = 1.4d0
real(8),parameter :: Re = 6356766.0d0 !---地球半径--
real(8),parameter :: GT = 0.0065d0 !-------気温減衰率K/m--------

!**********************************************************************
!                        Program Variable
!**********************************************************************
integer :: t1,t2,t2_rate,t2_max,t2_diff !- 計算時間測定用
real(8) :: t
real(8) :: dt
real(8) :: freq                !- 推力履歴のサンプリング周波数[Hz]
integer :: i,j
integer :: Flight_Status = 0   !- 飛翔フェーズ
integer :: index_wind,index_wangle
integer :: index_burn          !- 推力履歴のサイズ
integer :: fileNum_input = 100 !- 入力ファイル装置番号
integer :: fileNum_output = 200!- 出力ファイル装置番号
character :: filename*128
integer :: eof = 0

!- Mach-Cdスプライン用
real(8),allocatable :: Mach_base(:),Cd_base(:)
integer :: n_base = 0

real(8) :: temp !- 一時変数

!------------------------------------
!-    All loading (subscript:none)
!------------------------------------
real(8) :: l            !- 機体全長 :: length
real(8) :: m            !- 全機質量 :: mass
real(8) :: d,S          !- 機体直径,機体断面積
real(8) :: lcg          !- 全機重心位置(from nose) :: length center of gravity
real(8) :: lcg_0        !- 初期全機重心位置(from nose)
real(8) :: lcp          !- 圧力中心位置(from nose) :: length center of pressure
real(8) :: Ib(3)        !- 全機3軸慣性モーメント(xb,yb,zb) :: Inartia moment of body
real(8) :: Clp,Cmq,Cnr  !- 減衰モーメント係数(Clp:ロール,ピッチ,ヨー)
real(8) :: CNa,M_fin    !- 法線力係数,フィン取り付け角(機軸に対する)
real(8) :: Cd           !- 抗力係数

!------------------------------------
!-      Structure (subscript:s)
!------------------------------------
real(8) :: ms     !- 空虚質量
real(8) :: lcgs   !- 空虚重心位置(from nose)
real(8) :: Is,Ir  !- 空虚ピッチ慣性モーメント,ロール慣性モーメント(Iroll)

!------------------------------------
!-            Recovery
!------------------------------------
real(8) :: CdS1,CdS2       !- 1段目,2段目パラシュート抗力特性 :: Cd * S
real(8) :: CdS             !- 計算時に代入するパラシュート抗力特性
real(8) :: T_sepa,H_sepa   !- 1段目分離時刻(X+t[sec]),2段目開傘高度[m]


!------------------------------------
!-             Engine
!- (Oxidizer   subscript:ox)
!- (Fuel       subscript:f )
!- (Propellant subscript:p )
!------------------------------------
real(8) :: mox,mox_dot        !- 酸化剤質量,酸化剤質量流量
real(8) :: mox_0              !- 初期酸化剤質量
real(8) :: lcgox              !- 酸化剤重心位置(from nose)
real(8) :: lcgox_0
real(8) :: ltank              !- 酸化剤タンク長さ
real(8) :: mf                 !- 燃料質量
real(8) :: mf_b,mf_a,mf_dot   !- mf_before,mf_after
real(8) :: lcgf               !- 燃料重心位置(from nose)
real(8) :: Ifp,Ifr            !- 燃料ピッチ慣性モーメント,ロール慣性モーメント
real(8) :: lf,df1,df2         !- 燃料長さ,燃料外径,内径
real(8) :: mp,mp_dot          !- 推進剤質量
real(8) :: lcgp               !- 推進剤重心位置(from nose)
real(8) :: lcgp_0

real(8) :: Isp,It             !- 比推力,全力積
real(8),allocatable :: thrust(:)
real(8) :: de,Ae              !- ノズル出口 :: diameter exit

!------------------------------------
!-           Translation
!- (earth plane  subscript:e)
!- (body         subscript:b)
!- (Air          subscript:a)
!------------------------------------
real(8) :: Q              !- 動圧
real(8) :: Q_pre          !- 保存/比較用1step前
real(8) :: Dx,Ny,Nz       !- 各機体軸に対する空気力
real(8) :: F(3)           !- 機体座標系における外力
real(8) :: Position(3)    !- 座標(xe,ye,ze)
real(8) :: Position_pre(3)
real(8) :: Ve(3),Va(3)    !- 対地速度,対気速度
real(8) :: Ve_abs,Va_abs  !- ベクトル合成値
real(8) :: Ve_pre(3),Va_pre(3)
real(8) :: acce(3)        !- 対地加速度
real(8) :: acce_abs
real(8) :: acce_pre(3)
real(8) :: Mach           !- Mach Number
real(8) :: Mach_pre

!------------------------------------
!-             Rotation
!------------------------------------
real(8) :: Ma(3),Ka(3),Kj(3)      !- 空力モーメント,空力減衰モーメント,ジェットダンピングモーメント
real(8) :: omega(3),omega_pre(3)  !- 角速度
real(8) :: theta,psi,fai          !- ピッチ角,ヨー角,ロール角
real(8) :: alpha,beta             !- 迎角,横滑り角
real(8) :: theta_0,psi_0          !- 打ち上げ射角,打ち上げ方位角

!------------------------------------
!-            Quaternion
!------------------------------------
real(8) :: quat(4),quat_pre(4)    !- quaternion
real(8),dimension(3,3) :: Cbe,Ceb !- DCM body-->earth , earth-->body

!------------------------------------
!-            Condition
!------------------------------------
real(8) :: Vw_abs,Vw_angle        !- 風速,風向
real(8) :: Vw(3)                  !- (xe,ye,ze)の風速
real(8) :: Hw,Wh                  !- 風速測定高度,高度係数
real(8) :: Pa,Ta,rho,g,Cs         !- Air Pressure,Air Temperature,Air Dencity,Gravity,Speed of Sonic 
real(8) :: Pa_0,Ta_0,rho_0,g0
real(8) :: LL                     !- ランチャレール長[m]

!------------------------------------
!-              Save
!------------------------------------
real(8) :: t_top,Position_top(3),Va_top,Ve_top(3),m_top
real(8) :: V_lc,Acc_lc,Va_max,Mach_max
real(8) :: t_sepa2

!**********************************************************************
!                        Switch Variable
!**********************************************************************
integer :: LogTable_sw !- 0:Log/1:Table
real(8) :: Vw_min,Vw_max,Vw_delta
real(8) :: Vw_angle_min,Vw_angle_max,Vw_angle_delta

real(8) :: LandingRange(56,4)
real(8),dimension(7,8) :: AltitudeTable,MachTable,VaMaxTable,VaApogeeTable,VlcTable,AcclcTable
real(8),dimension(7,8) :: TimeApogeeTable,TimeHardTable,TimeDecentTable,TimeTSSSTable

contains

!**********************************************************************
!                       Universal Subroutine
!**********************************************************************
function deg2rad (x)
  real(8) :: deg2rad,x
  deg2rad = x*PI / 180.0d0
end function

function rad2deg (x)
  real(8) :: rad2deg,x
  rad2deg = x*180.0d0 / PI
end function

function deg2kelvin (x)
  real(8) :: deg2kelvin,x
  deg2kelvin = x + 273.15
end function

function getMachCd(Mach)
  real(8) :: getMachCd
  real(8),intent(in) :: Mach
  real(8) :: Cd
  call spline(Mach_base,Cd_base,n_base-1,Mach,Cd,1)
  getMachCd = Cd
end function getMachCd

!------------------------------------
!-        配列への要素追加
!-pythonのappendに近いもの
!-遅い
!------------------------------------
subroutine append(toarray,value)
  real(8),allocatable,intent(inout) :: toarray(:) !- 追加先の配列
  real(8),intent(in) :: value !- 追加する値
  real(8),allocatable :: temp(:)
  integer row_size
  integer :: i
  
  !- 追加先の配列が割付されていない場合,1行で割り付けて追加
  if (.not. allocated(toarray)) then
    allocate(toarray(1))
    toarray(1) = value
    return
  else !- 末尾に追加
    row_size = size(toarray)
    allocate(temp(row_size))
    temp = toarray
    deallocate(toarray)
    allocate(toarray(row_size+1))
    do i = 1,row_size
      toarray(i) = temp(i)
    end do
    toarray(row_size+1) = value
    return
  end if
end subroutine append

function abs_3axis(Vector)
  real(8),intent(in) :: Vector(3)
  real(8) :: abs_3axis
  abs_3axis = sqrt(Vector(1) * Vector(1) + Vector(2) * Vector(2) + Vector(3) * Vector(3))
end function abs_3axis


!-----------------------------------------------------------------------------!
!Construct cubic spline interpolation from a given data set yp(xp) 
!yp(i): given data values 
!xp(i): given points
!     where i=0,1,2,...nd 
!     np: maximum number of data point (total number of points is np+1)
!
!
!y: interpolated value
!x: interpolation location 
!
!mode == 0: initial setting
!mode == 1: spline interpolation
!-----------------------------------------------------------------------------!
subroutine spline(xp,yp,np,x,y,mode)
  integer,intent(in) :: np !data number
  integer :: mode,i,i1,i2
  real(8) :: x,y
  real(8),intent(in) :: xp(0:np),yp(0:np)
  real(8),allocatable :: h(:),a_mat(:),b_mat(:),c_mat(:),d_mat(:)
  real(8),allocatable,save :: a(:),b(:),c(:)!,x_save(:),y_save(;)
  
  ! Coefficient determination
  ! mode == 0
  if (mode == 0) then
    allocate(h(0:np),a_mat(0:np),b_mat(0:np),c_mat(0:np),d_mat(0:np))
    allocate(a(0:np-1),b(0:np-1),c(0:np))
    
    do i = 0,np-1
      h(i) = xp(i+1) - xp(i)
    end do
    h(np) = xp(np) - xp(np-1)
    
    b_mat(0) = 2.0d0*h(0)
    c_mat(0) = h(0)
    d_mat(0) = 3.0d0*(yp(1) - yp(0))
    do i = 1,np-1
      a_mat(i) = h(i)
      b_mat(i) = 2.0d0*(xp(i+1) - xp(i-1))
      c_mat(i) = h(i-1)
      d_mat(i) = (3.0d0*(yp(i) - yp(i-1))*h(i) / h(i-1)) + (3.0d0*(yp(i+1) - yp(i))*h(i-1) / h(i))
    end do
    a_mat(np) = h(np)
    b_mat(np) = 2.0d0*(h(np))
    d_mat(np) = 3.0d0*(yp(np) - yp(np-1))
    
    call tdma(a_mat,b_mat,c_mat,d_mat,np+1,c)
    
    do i = 0,np-1
      b(i) = (3.0d0*(yp(i+1) - yp(i)) - h(i)*(c(i+1) + 2.0d0*c(i))) / h(i)**2
      a(i) = (c(i+1) - c(i) - 2.0d0*b(i)*h(i)) / (3.0d0*h(i)**2)
    end do
    return
  end if
  
  ! spline interpolation
  ! mode == 0
  if (x <= xp(1)) then ! outside of the domain - lower
    i1 = 0
  else
    if (x >= xp(np-1)) then ! outside of the domain - upper
      i1 = np - 1
    else
      i1 = 1 ; i2 = np - 1
      do while (i2 - i1 > 1)
        i = (i1 + i2) / 2
        if (x < xp(i)) then
          i2 = i
        else
          i1 = i
        end if
      end do
    end if
  end if
  
  y = (a(i1)*(x - xp(i1))**2 + b(i1)*(x - xp(i1)) + c(i1))*(x - xp(i1)) + yp(i1)
  
end subroutine spline

!------------------------------------------------------------------!
!Tridiagonal matrix algorithm (TDMA)
!a: lower diagonal
!b: main diagonal
!c: upper diagonal
!d: source vector
!x: solution vector
!
!------------------------------------------------------------------!
subroutine tdma(a,b,c,d,n,x)
  real(8),intent(in) :: a(:),b(:),c(:),d(:)
  real(8),intent(out) :: x(:)
  real(8),allocatable :: G(:),H(:)
  real(8) :: temp
  integer,intent(in) :: n
  integer :: i
  
  allocate(G(n),H(n))
  
  !forward elimination
  G(1) = -c(1) / b(1)
  H(1) = d(1) / b(1)
  do i = 2,n
    temp = b(i) + a(i) * G(i-1)
    G(i) = -c(i) / temp
    H(i) = (d(i) - a(i) * H(i-1)) / temp
  end do
  
  !backward substitution
  x(n) = H(n)
  do i = n-1,1,-1
    x(i) = G(i) * x(i+1) + H(i)
  end do
end subroutine tdma

end module Standard_Collection